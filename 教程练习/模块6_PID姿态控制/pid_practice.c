/**
 * @file    pid_practice.c
 * @brief   模块六练习：PID 姿态控制（偏航角闭环 + 角度连续性处理）
 *
 * 学习目标：
 *   1. 理解 PID 三个分量各自的作用
 *   2. 掌握基于时间的积分微分计算（与采样周期无关）
 *   3. 理解角度连续性问题及 ±180° 边界处理
 *   4. 掌握积分限幅（防 windup）
 *   5. 实现偏航角控制映射到左右电机差速
 *
 * 对应项目文件：Drivers/BSP/pid/pid.c
 *
 * 验证方法：
 *   1. 设目标偏航角=90°，缓慢旋转IMU，观察电机如何调整
 *   2. 改 Kp 从 0.5→5.0，观察振荡变化
 *   3. 用 pid_gains:2.0:0.1:0.5 命令动态调整参数
 */

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ====================================================================
 * PID 控制器数据结构
 * ==================================================================== */
typedef struct {
    /* 参数（外部设置）*/
    float kp, ki, kd;
    float output_max, output_min;  /* 输出限幅 */
    float integral_max;             /* 积分限幅 */

    /* 内部状态（不要外部直接修改）*/
    float setpoint;      /* 目标值 */
    float last_error;    /* 上次误差（用于微分）*/
    float integral;      /* 积分累积 */
    uint32_t last_time;  /* 上次调用时间（ms）*/

    bool integral_enable;  /* 积分项开关 */
} PID_t;

/* 姿态控制器（包含偏航角PID）*/
typedef struct {
    PID_t yaw_pid;
    float target_yaw;
    bool  yaw_enable;
} AttCtrl_t;

/* 电机输出 */
typedef struct {
    float left_motor_pwm;
    float right_motor_pwm;
} MotionOutput_t;

/* ====================================================================
 * 工具函数：角度归一化
 * ==================================================================== */

/**
 * @brief  将角度限制在 [-180, +180] 范围
 *
 * 为什么需要这个？
 *   IMU 输出 Yaw 范围通常是 -180° ~ +180°
 *   目标角度经过加减后可能超出这个范围
 *   归一化确保误差计算正确
 */
float Normalize_Angle(float angle)
{
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief  计算两角度间的最短路径差值
 * @retval 范围 [-180, +180]，正=顺时针，负=逆时针
 *
 * 例：从 170° 转到 -170°，最短路径是 20°（逆时针），而非 -340°
 */
float Angle_Diff(float target, float current)
{
    return Normalize_Angle(target - current);
}

/* ====================================================================
 * PID 控制器实现
 * ==================================================================== */

/**
 * @brief  初始化 PID 控制器
 */
void PID_Init_Practice(PID_t *pid, float kp, float ki, float kd,
                       float out_max, float out_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_max  = out_max;
    pid->output_min  = out_min;
    pid->integral_max = fabsf(out_max) * 0.5f;  /* 积分限幅 = 输出限幅的一半 */

    pid->setpoint        = 0.0f;
    pid->last_error      = 0.0f;
    pid->integral        = 0.0f;
    pid->last_time       = 0;
    pid->integral_enable = true;
}

/**
 * @brief  PID 计算（基于时间的积分/微分）
 * @param  pid           PID 控制器
 * @param  measured      当前测量值（如当前偏航角）
 * @param  current_time  当前时间戳（ms，HAL_GetTick()）
 * @retval PID 输出值（已限幅）
 *
 * 关键设计：基于时间 dt 而非固定周期
 *   积分 += error × dt（秒）  → 无论采样多快，积分量都正确
 *   微分  = (error - last_error) / dt
 *
 * 注意：第一次调用时 last_time=0，dt 被赋予默认值 10ms
 */
float PID_Calculate_Practice(PID_t *pid, float measured, uint32_t current_time)
{
    /* 计算时间间隔 dt（秒）*/
    float dt;
    if (pid->last_time == 0) {
        dt = 0.01f;   /* 首次调用，假设 10ms */
    } else {
        dt = (float)(current_time - pid->last_time) / 1000.0f;
        if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;  /* 异常时间保护 */
    }

    /* 计算误差（注意：对角度要用 Angle_Diff！）*/
    float error = Normalize_Angle(pid->setpoint - measured);

    /* P项：比例 */
    float P = pid->kp * error;

    /* I项：积分（含限幅防 windup）*/
    float I = 0.0f;
    if (pid->integral_enable) {
        pid->integral += error * dt;  /* 积分累积（乘以时间→量纲正确）*/
        /* 积分限幅（防止积分饱和）*/
        if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
        if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
        I = pid->ki * pid->integral;
    }

    /* D项：微分（对误差变化率，不是测量值变化率，避免设定值跳变导致微分尖峰）*/
    float D = pid->kd * (error - pid->last_error) / dt;

    /* 总输出 = P + I + D，并限幅 */
    float output = P + I + D;
    if (output >  pid->output_max) output =  pid->output_max;
    if (output <  pid->output_min) output =  pid->output_min;

    /* 更新状态 */
    pid->last_error = error;
    pid->last_time  = current_time;

    return output;
}

/**
 * @brief  重置 PID（模式切换时调用，清除历史积分）
 */
void PID_Reset_Practice(PID_t *pid)
{
    pid->last_error = 0.0f;
    pid->integral   = 0.0f;
    pid->last_time  = 0;
}

/* ====================================================================
 * 姿态控制器：偏航角 → 左右电机 PWM
 * ==================================================================== */

/**
 * @brief  初始化姿态控制器（使用项目默认参数）
 */
void AttCtrl_Init_Practice(AttCtrl_t *ctrl)
{
    /* 偏航角 PID 默认参数（项目实测值）*/
    PID_Init_Practice(&ctrl->yaw_pid,
                      2.0f,   /* Kp：每1°误差产生2%的转向差 */
                      0.1f,   /* Ki：消除稳态偏差（积分项） */
                      0.5f,   /* Kd：预测趋势，防止超调 */
                      50.0f,  /* 输出上限（左右轮差值最大±50%）*/
                      -50.0f);/* 输出下限 */
    ctrl->target_yaw = 0.0f;
    ctrl->yaw_enable = false;
}

/**
 * @brief  更新姿态控制，返回左右电机 PWM
 * @param  ctrl          姿态控制器
 * @param  current_yaw   IMU 当前偏航角（degree）
 * @param  current_time  当前时间戳（ms）
 * @retval 左右电机的 PWM 占空比（%）
 *
 * 控制策略：
 *   基础前进速度 = 20%（确保船一直在前进）
 *   转向分量 = PID输出（正=左轮加速/右轮减速=右转，负=反之）
 *   left  = base + turn
 *   right = base - turn
 */
MotionOutput_t AttCtrl_Update_Practice(AttCtrl_t *ctrl,
                                       float current_yaw,
                                       uint32_t current_time)
{
    MotionOutput_t out = {0};
    float base_pwm = 20.0f;  /* 基础前进速度 */

    if (!ctrl->yaw_enable) {
        out.left_motor_pwm  = 0.0f;
        out.right_motor_pwm = 0.0f;
        return out;
    }

    /* 设置 PID 目标值（setpoint）*/
    ctrl->yaw_pid.setpoint = ctrl->target_yaw;

    /* 计算 PID 输出（转向分量）*/
    float turn = PID_Calculate_Practice(&ctrl->yaw_pid, current_yaw, current_time);

    /* 映射到左右电机 */
    out.left_motor_pwm  = base_pwm + turn;
    out.right_motor_pwm = base_pwm - turn;

    /* 限幅（0~100%）*/
    if (out.left_motor_pwm  > 100.0f) out.left_motor_pwm  = 100.0f;
    if (out.left_motor_pwm  <   0.0f) out.left_motor_pwm  =   0.0f;
    if (out.right_motor_pwm > 100.0f) out.right_motor_pwm = 100.0f;
    if (out.right_motor_pwm <   0.0f) out.right_motor_pwm =   0.0f;

    return out;
}

/* ====================================================================
 * 参数调试建议（仿真测试）
 * ==================================================================== */
void PID_TuningDemo(void)
{
    AttCtrl_t ctrl;
    AttCtrl_Init_Practice(&ctrl);
    ctrl.yaw_enable = true;
    ctrl.target_yaw = 90.0f;  /* 目标：正东方向（假设北=0）*/

    printf("=== PID 调参演示 ===\r\n");
    printf("目标偏航角：90°\r\n\r\n");

    /* 模拟船从 0° 转向 90° 的过程（仅演示，无真实IMU）*/
    float simulated_yaw = 0.0f;
    float prev_yaw = 0.0f;

    for (int step = 0; step < 30; step++) {
        uint32_t t = step * 20;   /* 模拟20ms步长 */
        MotionOutput_t out = AttCtrl_Update_Practice(&ctrl, simulated_yaw, t);

        float diff = out.left_motor_pwm - out.right_motor_pwm;
        /* 简化模拟：转向差值越大，偏航角变化越快 */
        simulated_yaw += diff * 0.3f;
        simulated_yaw = Normalize_Angle(simulated_yaw);

        printf("t=%3dms  Yaw=%.1f°  L=%.1f%%  R=%.1f%%  diff=%.1f\r\n",
               t, simulated_yaw, out.left_motor_pwm, out.right_motor_pwm, diff);

        if (fabsf(Angle_Diff(90.0f, simulated_yaw)) < 2.0f) {
            printf("到达目标！\r\n");
            break;
        }
    }
}

/* ====================================================================
 * 思考题：
 *
 * Q1：Kp 太大会发生什么？
 *     A：P项 = Kp × error，Kp过大时：
 *        误差10° → 转向分量=50%（大幅差速）→ 过冲到-10°
 *        → 又大幅纠正 → 来回振荡（震荡/Oscillation）。
 *        正确调法：先只用P项，调到"响应快但不振荡"，再加D项减振，最后加I项消稳态误差。
 *
 * Q2：为什么用"误差的积分"而不是"测量值的积分"？
 *     A：积分项的目的是消除稳态误差（当error→0时，之前的积分累积能产生持续的输出）。
 *        如果对测量值积分，测量值是0时输出也是0，无法消除稳态误差。
 *        积分误差而不是积分测量值，才能正确达到目的。
 *
 * Q3：积分限幅设为 output_max × 0.5 是什么意思？
 *     A：Ki × integral_max = 0.1 × 25 = 2.5（积分项最大贡献2.5%转向）。
 *        这样积分项对总输出最多贡献一半，不会因为积分饱和导致响应变慢。
 *        如果积分无限制，在长时间偏差下会越来越大，等到误差消除后
 *        积分项还很大，导致严重超调。
 *
 * Q4：为什么每次切换到 PID 模式前要调用 PID_Reset？
 *     A：积分项记录的是历史误差的累积。切换前如果在手动控制，
 *        船可能在大偏差状态，积分已经很大了。
 *        如果不重置，切到PID后积分项立刻产生大输出，电机剧烈跳变。
 *        Reset 清零积分，让 PID 从"零状态"开始干净地控制。
 * ==================================================================== */
