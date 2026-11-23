/**
 * @file    pwm_motor_practice.c
 * @brief   模块二练习：TIM1 PWM Mode2 双路电机控制 + 安全状态机
 *
 * 学习目标：
 *   1. 理解 PWM Mode2 与 Mode1 的区别（CCR计算方式相反）
 *   2. 掌握双路电机的前进/左转/右转/停止控制
 *   3. 实现 PWM 安全状态管理（三种控制模式 + 渐变限制）
 *
 * 对应项目文件：Drivers/BSP/TIM/tim.c + Users/main.c
 *
 * 硬件：
 *   TIM1_CH1 → PE9 → 左电机 H 桥信号
 *   TIM1_CH2 → PE11 → 右电机 H 桥信号
 *   PWM频率：1kHz（Period=1000，Prescaler=83）
 *
 * 验证方法：
 *   用示波器测 PE9/PE11，
 *   调用 Motor_Forward() 应看到两路 70% 占空比方波
 */

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>

/* ====================================================================
 * PWM 参数（与 tim.h 一致）
 * ==================================================================== */
#define PWM_FREQUENCY    1000    /* 1kHz */
#define PWM_RESOLUTION   1000    /* 分辨率（ARR值）*/
/* 计算预分频：84MHz / (1000Hz × 1000) - 1 = 83 */
#define PWM_PRESCALER    83

#define PWM_CHANNEL_1    1
#define PWM_CHANNEL_2    2

/* ====================================================================
 * 控制模式枚举（与 main.c 一致）
 * ==================================================================== */
typedef enum {
    PWM_MODE_MANUAL = 0,   /* 手动：立即响应 */
    PWM_MODE_PID   = 1,   /* PID：渐变平滑 */
    PWM_MODE_GPS   = 2    /* GPS：渐变平滑 */
} PWM_Control_Mode_t;

typedef struct {
    float left_pwm;
    float right_pwm;
    uint32_t last_update_time;
} PWM_State_t;

static TIM_HandleTypeDef htim1_local;
static PWM_State_t       pwm_state;
static PWM_Control_Mode_t current_mode = PWM_MODE_MANUAL;

/* ====================================================================
 * Step1：TIM1 PWM 初始化
 * ==================================================================== */

/**
 * @brief  TIM1 PWM 初始化（Mode2，双通道，PE9/PE11）
 *
 * PWM Mode2 重点理解：
 *   计数器从 0 数到 ARR（1000），计数期间：
 *   Mode1：CNT < CCR 时高电平 → duty = CCR/ARR
 *   Mode2：CNT < CCR 时低电平 → duty = (ARR-CCR)/ARR = 1 - CCR/ARR
 *
 *   所以 Mode2 下：
 *     想要 0% 占空比（停转）：CCR = ARR = 1000（全低电平）
 *     想要 70% 占空比（正转）：CCR = 1000 × (1-0.7) = 300
 *     想要 100% 占空比（全速）：CCR = 0（全高电平）
 */
void TIM1_PWM_Init_Practice(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* 配置 PE9/PE11 为 TIM1_CH1/CH2 复用推挽输出 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_11;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &gpio);

    /* TIM1 基本配置 */
    htim1_local.Instance               = TIM1;
    htim1_local.Init.Prescaler         = PWM_PRESCALER;        /* 84MHz/(83+1)=1MHz */
    htim1_local.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1_local.Init.Period            = PWM_RESOLUTION - 1;    /* ARR=999，计0~999 */
    htim1_local.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1_local.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim1_local);

    /* PWM 通道配置（Mode2）*/
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM2;    /* ← 关键：Mode2 */
    oc.Pulse      = 1000;               /* 初始 CCR=ARR → 0% 占空比（停转）*/
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1_local, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1_local, &oc, TIM_CHANNEL_2);

    /* 启动两路 PWM 输出 */
    HAL_TIM_PWM_Start(&htim1_local, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1_local, TIM_CHANNEL_2);

    /* 初始化状态 */
    pwm_state.left_pwm  = 0.0f;
    pwm_state.right_pwm = 0.0f;
}

/* ====================================================================
 * Step2：设置占空比（Mode2 公式）
 * ==================================================================== */

/**
 * @brief  设置单路 PWM 占空比
 * @param  channel  PWM_CHANNEL_1 或 PWM_CHANNEL_2
 * @param  duty     占空比（0.0~100.0，单位%）
 *
 * Mode2 公式：CCR = ARR × (1 - duty/100)
 */
void TIM1_PWM_SetDuty_Practice(uint8_t channel, float duty)
{
    if (duty < 0.0f)   duty = 0.0f;
    if (duty > 100.0f) duty = 100.0f;

    /* Mode2 换算：duty越高，CCR越小 */
    uint16_t ccr = (uint16_t)(PWM_RESOLUTION * (1.0f - duty / 100.0f));

    if (channel == PWM_CHANNEL_1) {
        __HAL_TIM_SET_COMPARE(&htim1_local, TIM_CHANNEL_1, ccr);
    } else if (channel == PWM_CHANNEL_2) {
        __HAL_TIM_SET_COMPARE(&htim1_local, TIM_CHANNEL_2, ccr);
    }
}

/* ====================================================================
 * Step3：安全 PWM 设置（含渐变保护）
 * ==================================================================== */

/**
 * @brief  安全设置双路 PWM（含模式判断和渐变限制）
 * @param  left   左电机占空比（0~100%）
 * @param  right  右电机占空比（0~100%）
 */
void PWM_Safe_SetDuty_Practice(float left, float right)
{
    /* 限幅 */
    if (left  < 0.0f) left  = 0.0f;
    if (left  > 100.0f) left  = 100.0f;
    if (right < 0.0f) right = 0.0f;
    if (right > 100.0f) right = 100.0f;

    if (current_mode == PWM_MODE_MANUAL) {
        /* 手动模式：立即生效（操作响应性优先）*/
        pwm_state.left_pwm  = left;
        pwm_state.right_pwm = right;
        TIM1_PWM_SetDuty_Practice(PWM_CHANNEL_1, left);
        TIM1_PWM_SetDuty_Practice(PWM_CHANNEL_2, right);
        return;
    }

    /* PID/GPS 模式：限制单次变化 ≤ 30%（防电机剧烈跳变）*/
    float left_diff  = left  - pwm_state.left_pwm;
    float right_diff = right - pwm_state.right_pwm;

    if (fabsf(left_diff) > 30.0f) {
        left = pwm_state.left_pwm + (left_diff > 0 ? 30.0f : -30.0f);
    }
    if (fabsf(right_diff) > 30.0f) {
        right = pwm_state.right_pwm + (right_diff > 0 ? 30.0f : -30.0f);
    }

    pwm_state.left_pwm  = left;
    pwm_state.right_pwm = right;
    TIM1_PWM_SetDuty_Practice(PWM_CHANNEL_1, left);
    TIM1_PWM_SetDuty_Practice(PWM_CHANNEL_2, right);
}

/**
 * @brief  切换 PWM 控制模式
 */
void PWM_Switch_Mode_Practice(PWM_Control_Mode_t new_mode)
{
    current_mode = new_mode;
}

/* ====================================================================
 * Step4：运动控制接口
 * ==================================================================== */

void Motor_Forward(void)
{
    PWM_Switch_Mode_Practice(PWM_MODE_MANUAL);
    PWM_Safe_SetDuty_Practice(70.0f, 70.0f);
}

void Motor_TurnLeft(void)
{
    PWM_Switch_Mode_Practice(PWM_MODE_MANUAL);
    PWM_Safe_SetDuty_Practice(70.0f, 0.0f);   /* 左轮转，右轮停 */
}

void Motor_TurnRight(void)
{
    PWM_Switch_Mode_Practice(PWM_MODE_MANUAL);
    PWM_Safe_SetDuty_Practice(0.0f, 70.0f);   /* 左轮停，右轮转 */
}

void Motor_Stop(void)
{
    PWM_Switch_Mode_Practice(PWM_MODE_MANUAL);
    PWM_Safe_SetDuty_Practice(0.0f, 0.0f);
}

/* ====================================================================
 * 测试函数（在 main 里调用）
 * ==================================================================== */
void Motor_Test(void)
{
    TIM1_PWM_Init_Practice();

    /* 前进 2 秒 */
    Motor_Forward();
    HAL_Delay(2000);

    /* 左转 1 秒 */
    Motor_TurnLeft();
    HAL_Delay(1000);

    /* 右转 1 秒 */
    Motor_TurnRight();
    HAL_Delay(1000);

    /* 停止 */
    Motor_Stop();
}

/* ====================================================================
 * PWM Mode2 占空比对照表（供查阅）：
 *
 * duty%  |  CCR（ARR=1000）| 含义
 * --------|-----------------|-----
 *   0%   |   1000          | 全低，电机停
 *  25%   |    750          | 四分之一速度
 *  50%   |    500          | 半速
 *  70%   |    300          | 七成速（项目手动模式）
 *  100%  |      0          | 全速
 *
 * 思考题：
 *
 * Q1：为什么 Mode2 初始化时 Pulse=1000 而不是 0？
 *     A：Mode2下，Pulse（CCR）=ARR=1000 时，CNT永远不会 < CCR（因为CNT最大=ARR-1=999），
 *        输出始终为低电平（0%占空比）。如果设 Pulse=0，则 CNT 永远 >= CCR，
 *        输出始终为高电平（100%占空比），电机全速旋转！上电就全速是很危险的。
 *
 * Q2：为什么手动模式下不做渐变？
 *     A：手动模式是人工遥控，操作者期望立即响应（按下停止就立刻停）。
 *        而PID/GPS模式下，控制量是算法计算的，偶发突变可能是计算噪声，
 *        需要渐变保护防止电机损坏。
 *
 * Q3：H桥驱动原理是什么，为什么一路 PWM 可以控制一个直流电机？
 *     A：H桥由4个开关（MOSFET或晶体管）组成桥形电路。
 *        PWM 信号控制对角线两个开关的通断频率 → 调节电机两端的平均电压。
 *        这里每路 PWM 控制一个方向的转速（不反转），适合差速转向的无人船。
 * ==================================================================== */
