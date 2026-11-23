/**
 * @file    sensor_practice.c
 * @brief   模块三练习：DS18B20 温度 + TDS 水质 + 距离传感器
 *
 * 学习目标：
 *   1. 掌握 DS18B20 1-Wire 时序（精确微秒延时是关键）
 *   2. 理解 TDS ADC 的三步换算（ADC→电压→温补→曲线→ppm）
 *   3. 理解串口型距离传感器的异步接收模式
 *
 * 对应项目文件：
 *   Drivers/BSP/ds18b20/ds18b20.c
 *   Drivers/BSP/tds/tds.c
 *   Drivers/BSP/ultrasonic/ultrasonic.c
 *   Drivers/BSP/laser/laser.c
 *
 * 验证方法：
 *   DS18B20：手捂传感器，温度应升高
 *   TDS：放入纯水（~0ppm），放入盐水（>500ppm）
 *   超声波：正对墙壁，移近移远观察距离变化
 */

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>

/* ====================================================================
 * 练习A：DS18B20 1-Wire 温度传感器
 * ==================================================================== */

/* DS18B20 引脚定义 */
#define DS18B20_PORT   GPIOB
#define DS18B20_PIN    GPIO_PIN_0
#define DS18B20_CLK_EN() __HAL_RCC_GPIOB_CLK_ENABLE()

/* 命令字节 */
#define CMD_SKIP_ROM       0xCC  /* 跳过ROM匹配（总线只有1个DS18B20时使用）*/
#define CMD_CONVERT_T      0x44  /* 启动温度转换 */
#define CMD_READ_SCRATCH   0xBE  /* 读取暂存寄存器（温度结果在前2字节）*/

/* 辅助：切换引脚方向 */
static void ds_set_output(void) {
    GPIO_InitTypeDef g = {0};
    g.Pin   = DS18B20_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS18B20_PORT, &g);
}

static void ds_set_input(void) {
    GPIO_InitTypeDef g = {0};
    g.Pin  = DS18B20_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;   /* 模拟外部上拉（实际电路应有4.7kΩ外部上拉）*/
    HAL_GPIO_Init(DS18B20_PORT, &g);
}

#define DS_HIGH()  HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET)
#define DS_LOW()   HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET)
#define DS_READ()  HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)

/* 注意：需要精确微秒延时，这里假设有 delay_us() 函数（DWT实现）*/
extern void delay_us(uint32_t us);
extern void delay_ms(uint32_t ms);

/**
 * @brief  DS18B20 复位，检测存在脉冲
 * @retval 1=检测到芯片，0=未检测到
 *
 * 时序：
 *   主机拉低 ≥480μs → 释放总线 → 等待15-60μs后DS18B20拉低（存在脉冲）
 *   → 等到60-240μs结束
 */
uint8_t DS18B20_Reset_Practice(void)
{
    ds_set_output();
    DS_LOW();          /* 主机拉低，产生复位脉冲 */
    delay_us(500);     /* 拉低500μs（规范要求≥480μs）*/

    DS_HIGH();         /* 释放总线 */
    ds_set_input();    /* 切换为输入，准备读存在脉冲 */
    delay_us(70);      /* 等待DS18B20响应（15~60μs内会拉低）*/

    uint8_t present = (DS_READ() == GPIO_PIN_RESET) ? 1 : 0;

    delay_us(410);     /* 等待存在脉冲结束 */
    return present;
}

/**
 * @brief  向 DS18B20 写1字节（LSB first）
 *
 * 写'1'：拉低2μs → 释放（高）→ 保持60μs
 * 写'0'：拉低62μs → 释放 → 保持2μs
 */
void DS18B20_WriteByte_Practice(uint8_t byte)
{
    ds_set_output();
    for (int i = 0; i < 8; i++) {
        DS_LOW();               /* 启动写时隙：拉低 */
        delay_us(2);
        if (byte & 0x01) {
            DS_HIGH();          /* 写'1'：立即释放 */
            delay_us(60);
        } else {
            /* 写'0'：继续保持低电平 */
            delay_us(60);
            DS_HIGH();          /* 时隙结束才释放 */
        }
        byte >>= 1;
        delay_us(2);            /* 时隙间恢复时间 */
    }
}

/**
 * @brief  从 DS18B20 读1字节（LSB first）
 *
 * 读时隙：主机拉低2μs → 释放 → 15μs内读取总线状态
 */
uint8_t DS18B20_ReadByte_Practice(void)
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        ds_set_output();
        DS_LOW();
        delay_us(2);            /* 启动读时隙 */
        ds_set_input();         /* 释放总线，准备读 */
        data >>= 1;
        if (DS_READ() == GPIO_PIN_SET) {
            data |= 0x80;       /* 读到高电平→bit=1，放到最高位 */
        }
        delay_us(60);           /* 读时隙总长≥60μs */
    }
    return data;
}

/**
 * @brief  读取温度（完整流程，耗时约760ms！）
 * @retval 温度值（°C），-273.15表示错误
 */
float DS18B20_ReadTemp_Practice(void)
{
    if (!DS18B20_Reset_Practice()) return -273.15f;

    DS18B20_WriteByte_Practice(CMD_SKIP_ROM);    /* 跳过ROM */
    DS18B20_WriteByte_Practice(CMD_CONVERT_T);   /* 启动转换 */
    delay_ms(760);    /* ← 关键：必须等760ms！12位精度ADC转换时间 */

    if (!DS18B20_Reset_Practice()) return -273.15f;

    DS18B20_WriteByte_Practice(CMD_SKIP_ROM);
    DS18B20_WriteByte_Practice(CMD_READ_SCRATCH);

    uint8_t lsb = DS18B20_ReadByte_Practice();
    uint8_t msb = DS18B20_ReadByte_Practice();

    /* 拼装成16位有符号整数，乘以分辨率 */
    int16_t raw = (int16_t)((msb << 8) | lsb);
    /* 12位分辨率：1 LSB = 0.0625°C（1/16°C）*/
    return raw * 0.0625f;
}

/* ====================================================================
 * 练习B：TDS 水质传感器（ADC + 温度补偿）
 * ==================================================================== */

#define TDS_VREF        3.3f    /* 参考电压 */
#define TDS_ADC_RES     4096.0f /* 12位ADC满量程 */
#define TDS_K_DEFAULT   0.692f  /* 出厂校准系数（用标准液校准后修改）*/

static ADC_HandleTypeDef hadc1_tds;
static float tds_k_value = TDS_K_DEFAULT;

/**
 * @brief  ADC1 初始化（PA0，单次轮询）
 */
void TDS_Init_Practice(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin  = GPIO_PIN_0;
    g.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &g);

    hadc1_tds.Instance                   = ADC1;
    hadc1_tds.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1_tds.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1_tds.Init.ScanConvMode          = DISABLE;
    hadc1_tds.Init.ContinuousConvMode    = DISABLE;
    hadc1_tds.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1_tds.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1_tds.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1_tds.Init.NbrOfConversion       = 1;
    HAL_ADC_Init(&hadc1_tds);

    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = ADC_CHANNEL_0;
    ch.Rank         = 1;
    ch.SamplingTime = ADC_SAMPLETIME_480CYCLES;  /* 480周期，约22.8μs @21MHz */
    HAL_ADC_ConfigChannel(&hadc1_tds, &ch);
}

/**
 * @brief  读取一次 ADC（轮询）
 * @retval ADC原始值（0~4095），0xFFFF表示失败
 */
static uint16_t TDS_ReadADC_Once(void)
{
    if (HAL_ADC_Start(&hadc1_tds) != HAL_OK) return 0xFFFF;
    if (HAL_ADC_PollForConversion(&hadc1_tds, 10) != HAL_OK) return 0xFFFF;
    return (uint16_t)HAL_ADC_GetValue(&hadc1_tds);
}

/**
 * @brief  读取 ADC 均值（10次平均，消除随机噪声）
 */
static uint16_t TDS_ReadADC_Avg(uint8_t times)
{
    uint32_t sum = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < times; i++) {
        uint16_t v = TDS_ReadADC_Once();
        if (v != 0xFFFF) { sum += v; count++; }
    }
    return (count > 0) ? (uint16_t)(sum / count) : 0xFFFF;
}

/**
 * @brief  获取 TDS 值（含温度补偿）
 * @param  temperature  当前水温（°C），传入 DS18B20 读取的温度
 * @retval TDS值（ppm），-1表示错误
 *
 * 三步换算：
 *   ① ADC原始值 → 采样电压
 *   ② 采样电压 / 温度系数 → 25°C等效电压
 *   ③ 三次多项式 → TDS(ppm)，再乘以校准系数k
 */
float TDS_GetValue_Practice(float temperature)
{
    uint16_t adc = TDS_ReadADC_Avg(10);
    if (adc == 0xFFFF) return -1.0f;

    /* 步骤1：ADC → 电压 */
    float voltage = (float)adc / TDS_ADC_RES * TDS_VREF;

    /* 步骤2：温度补偿
     * 水的电导率随温度变化约2%/°C，以25°C为基准
     * coefficient = 1 + 0.02 × (T - 25)
     * 补偿后电压 = 实测电压 / coefficient（折算到25°C时的等效电压）*/
    if (temperature < -270.0f) temperature = 25.0f;  /* 温度无效时用25°C */
    float coeff = 1.0f + 0.02f * (temperature - 25.0f);
    if (coeff < 0.01f) coeff = 1.0f;
    float v_comp = voltage / coeff;

    /* 步骤3：三次多项式曲线拟合（TDS传感器标定曲线）*/
    if (v_comp < 0.0f) v_comp = 0.0f;
    float tds_curve = 66.71f * powf(v_comp, 3.0f)
                    - 127.93f * powf(v_comp, 2.0f)
                    + 428.7f  * v_comp;

    /* 步骤4：乘以校准系数（用标准液校准后设置）*/
    float tds = tds_curve * tds_k_value;
    return (tds < 0.0f) ? 0.0f : tds;
}

/* ====================================================================
 * 测试函数
 * ==================================================================== */
void Sensor_Test(void)
{
    DS18B20_CLK_EN();
    TDS_Init_Practice();

    while (1) {
        /* 读温度（约760ms）*/
        float temp = DS18B20_ReadTemp_Practice();

        /* 读TDS（依赖温度补偿）*/
        float tds = TDS_GetValue_Practice(temp);

        printf("Temp=%.2f°C  TDS=%.1f ppm\r\n", temp, tds);
        /* 不需要额外延时，DS18B20已经耗时760ms */
    }
}

/* ====================================================================
 * 思考题：
 *
 * Q1：DS18B20 读温度为什么要 760ms？能不能缩短？
 *     A：DS18B20 内部 12 位 ADC 转换需要 750ms（手册规定）。
 *        可以改用 9 位精度（93.75ms），精度降为 0.5°C/LSB，
 *        修改配置寄存器第5/6位（[R1 R0]）可设置精度。
 *        对水温监测来说，0.5°C 精度通常够用。
 *
 * Q2：TDS 传感器为什么要温度补偿？
 *     A：水的电导率（TDS传感器实际测量的是电导率）随温度变化明显：
 *        温度每升高1°C，电导率约增加2%。
 *        不补偿的话，夏天河水（25°C）和冬天河水（5°C）即使TDS相同，
 *        读数会相差约40%。温度补偿把不同温度的测量值"折算"到25°C基准。
 *
 * Q3：三次多项式系数 66.71, -127.93, 428.7 是怎么来的？
 *     A：用已知浓度的标准液（如0ppm、500ppm、1000ppm）测量对应电压，
 *        用最小二乘法拟合出三次多项式。这个公式是传感器厂家或网上
 *        TDS 模块的常用拟合结果，k_value 是进一步微调的乘法校正。
 *
 * Q4：超声波模块和激光模块都是串口发数据，怎么区分用哪个？
 *     A：超声波：价格低，测量范围大（几厘米到几米），但精度低（~1cm），
 *              受声波反射角度影响，水面测距有误差。
 *        激光：精度高（~1mm），不受反射角影响，但价格高，远距离衰减大。
 *        项目两个都用，让上位机可以选择哪个更可靠。
 * ==================================================================== */
