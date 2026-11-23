/**
 * @file    imu_i2c_practice.c
 * @brief   模块五练习：软件I2C（Bit-Bang）+ IMU 姿态数据读取
 *
 * 学习目标：
 *   1. 理解软件 I2C 的位操作时序（开始/停止/发送/接收/ACK）
 *   2. 掌握开漏输出的概念（SDA 为何需要开漏）
 *   3. 理解为什么本项目用软件 I2C 而非硬件 I2C
 *   4. 掌握 IMU 数据的使用方式（三轴角度/加速度/陀螺仪）
 *
 * 对应项目文件：
 *   Drivers/BSP/iic/IOI2C.c
 *   Drivers/BSP/IMU/IMU.c
 *
 * 硬件：
 *   SCL → GPIOB PIN10（时钟线，主机输出）
 *   SDA → GPIOB PIN11（数据线，双向开漏）
 *   IMU：WIT Motion（维特智能）系列（I2C地址 0x50）
 *
 * 验证方法：
 *   转动IMU，角度值应实时变化（偏航角 angle[2] 随旋转变化）
 */

#include "stm32f4xx_hal.h"
#include <stdio.h>

/* ====================================================================
 * 软件 I2C 引脚定义
 * ==================================================================== */
#define IIC_SCL_PORT  GPIOB
#define IIC_SCL_PIN   GPIO_PIN_10
#define IIC_SDA_PORT  GPIOB
#define IIC_SDA_PIN   GPIO_PIN_11

/* 控制宏 */
#define SCL_H()   HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET)
#define SCL_L()   HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET)
#define SDA_H()   HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET)
#define SDA_L()   HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_RESET)
#define SDA_R()   HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN)  /* 读取SDA */

/* 延时（软件I2C时序用，约几微秒）*/
static void iic_delay(void)
{
    for (volatile int i = 0; i < 20; i++);  /* 调整数值改变I2C速度 */
}

/* ====================================================================
 * 软件 I2C 基本操作
 *
 * 关键概念：开漏输出（Open-Drain）
 *   SDA 线上多个设备都连接（主机+从机），任一设备拉低都能把总线拉低。
 *   但"拉高"是靠外部上拉电阻，不是主动驱动高电平。
 *   这就是 I2C 的"线与"特性：任一设备为低 → 总线为低。
 *
 *   STM32 GPIO_MODE_OUTPUT_OD = 开漏输出：
 *     写0 → 主动拉低（输出低）
 *     写1 → 高阻态（依赖上拉电阻拉高）
 *     读  → 读取总线实际电平
 * ==================================================================== */

static void SDA_SetOutput(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = IIC_SDA_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_OD;  /* 开漏输出 */
    g.Pull  = GPIO_NOPULL;          /* 外部上拉，不用内部 */
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC_SDA_PORT, &g);
}

static void SDA_SetInput(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = IIC_SDA_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(IIC_SDA_PORT, &g);
}

/**
 * @brief  I2C 初始化（引脚配置）
 */
void IIC_Init_Practice(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* SCL：推挽输出（主机单向驱动时钟）*/
    GPIO_InitTypeDef g = {0};
    g.Pin   = IIC_SCL_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC_SCL_PORT, &g);

    SDA_SetOutput();  /* SDA：开漏输出 */

    SCL_H();  /* 初始状态：SCL/SDA 均为高（总线空闲）*/
    SDA_H();
}

/**
 * @brief  I2C 起始条件（START）
 * 时序：SCL 高时，SDA 从高跳变为低
 *
 *  SDA: ──────╮
 *             │
 *  SCL: ──────│─────
 *           START
 */
void IIC_Start_Practice(void)
{
    SDA_SetOutput();
    SDA_H(); iic_delay();
    SCL_H(); iic_delay();
    SDA_L(); iic_delay();  /* SDA拉低（START条件）*/
    SCL_L(); iic_delay();  /* SCL拉低，准备发数据 */
}

/**
 * @brief  I2C 停止条件（STOP）
 * 时序：SCL 高时，SDA 从低跳变为高
 */
void IIC_Stop_Practice(void)
{
    SDA_SetOutput();
    SCL_L(); SDA_L(); iic_delay();
    SCL_H(); iic_delay();
    SDA_H(); iic_delay();  /* SDA拉高（STOP条件）*/
}

/**
 * @brief  发送一个字节并等待 ACK
 * @param  byte  要发送的字节
 * @retval 0=收到ACK（正常），1=NACK（错误）
 *
 * 时序（每bit）：
 *   SCL低时设置SDA → SCL拉高（从机在高时采样）→ SCL拉低
 */
uint8_t IIC_Send_Byte_Practice(uint8_t byte)
{
    SDA_SetOutput();
    for (int i = 7; i >= 0; i--) {
        SCL_L();
        if ((byte >> i) & 0x01)
            SDA_H();
        else
            SDA_L();
        iic_delay();
        SCL_H();  /* 上升沿：从机采样SDA */
        iic_delay();
    }
    /* 等待 ACK */
    SCL_L();
    SDA_H();       /* 释放SDA，让从机控制 */
    SDA_SetInput();
    iic_delay();
    SCL_H();
    iic_delay();
    uint8_t ack = SDA_R();  /* 0=ACK，1=NACK */
    SCL_L();
    return ack;
}

/**
 * @brief  接收一个字节
 * @param  ack  接收后是否发 ACK（1=发ACK继续，0=发NACK结束）
 * @retval 接收到的字节
 */
uint8_t IIC_Read_Byte_Practice(uint8_t ack)
{
    uint8_t data = 0;
    SDA_SetInput();
    for (int i = 7; i >= 0; i--) {
        SCL_L(); iic_delay();
        SCL_H(); iic_delay();
        if (SDA_R()) data |= (1 << i);
    }
    /* 发 ACK 或 NACK */
    SCL_L();
    SDA_SetOutput();
    if (ack) SDA_L(); else SDA_H();
    iic_delay();
    SCL_H(); iic_delay();
    SCL_L(); iic_delay();
    return data;
}

/* ====================================================================
 * IMU 数据读取（封装层）
 * ==================================================================== */

/* IMU I2C 地址（WIT Motion）*/
#define IMU_ADDR   0x50

/* WIT IMU 寄存器（数据寄存器起始地址）*/
#define IMU_REG_ACC    0x34   /* 加速度寄存器组起始 */
#define IMU_REG_GYRO   0x37   /* 陀螺仪寄存器组起始 */
#define IMU_REG_ANGLE  0x3D   /* 角度寄存器组起始 */

typedef struct {
    float acc[3];     /* 加速度 X/Y/Z（g）*/
    float gyro[3];    /* 角速度 X/Y/Z（°/s）*/
    float angle[3];   /* 横滚/俯仰/偏航角（度），angle[2]=Yaw */
} IMU_Data_Simple_t;

/**
 * @brief  读取 IMU 寄存器（多字节）
 * @param  reg_addr  寄存器起始地址
 * @param  buf       接收缓冲区
 * @param  len       字节数
 */
static void IMU_ReadRegs(uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
    /* I2C 写寄存器地址 */
    IIC_Start_Practice();
    IIC_Send_Byte_Practice((IMU_ADDR << 1) | 0);  /* 写地址 */
    IIC_Send_Byte_Practice(reg_addr);              /* 寄存器地址 */

    /* 重复START，切换为读模式 */
    IIC_Start_Practice();
    IIC_Send_Byte_Practice((IMU_ADDR << 1) | 1);  /* 读地址 */

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = IIC_Read_Byte_Practice(i < len - 1 ? 1 : 0);  /* 最后一字节NACK */
    }
    IIC_Stop_Practice();
}

/**
 * @brief  读取 IMU 所有姿态数据
 * @param  data  输出数据结构体
 *
 * WIT Motion 数据格式（每轴2字节，小端，有符号）：
 *   原始值 / 32768 × 量程 = 实际值
 *   加速度量程：±16g
 *   角速度量程：±2000°/s
 *   角度量程：±180°
 */
void IMU_ReadData_Practice(IMU_Data_Simple_t *data)
{
    uint8_t buf[6];
    int16_t raw[3];

    /* 读加速度（6字节：XL XH YL YH ZL ZH）*/
    IMU_ReadRegs(IMU_REG_ACC, buf, 6);
    for (int i = 0; i < 3; i++) {
        raw[i] = (int16_t)((buf[i*2+1] << 8) | buf[i*2]);
        data->acc[i] = (float)raw[i] / 32768.0f * 16.0f;
    }

    /* 读角速度 */
    IMU_ReadRegs(IMU_REG_GYRO, buf, 6);
    for (int i = 0; i < 3; i++) {
        raw[i] = (int16_t)((buf[i*2+1] << 8) | buf[i*2]);
        data->gyro[i] = (float)raw[i] / 32768.0f * 2000.0f;
    }

    /* 读角度（Roll=angle[0], Pitch=angle[1], Yaw=angle[2]）*/
    IMU_ReadRegs(IMU_REG_ANGLE, buf, 6);
    for (int i = 0; i < 3; i++) {
        raw[i] = (int16_t)((buf[i*2+1] << 8) | buf[i*2]);
        data->angle[i] = (float)raw[i] / 32768.0f * 180.0f;
    }
}

/* ====================================================================
 * 测试函数
 * ==================================================================== */
void IMU_Test_Practice(void)
{
    IIC_Init_Practice();

    IMU_Data_Simple_t imu;
    while (1) {
        IMU_ReadData_Practice(&imu);
        printf("ACC: %.3f %.3f %.3f g\r\n",    imu.acc[0], imu.acc[1], imu.acc[2]);
        printf("GYRO:%.1f %.1f %.1f deg/s\r\n", imu.gyro[0], imu.gyro[1], imu.gyro[2]);
        printf("ANGLE:Roll=%.1f Pitch=%.1f Yaw=%.1f\r\n",
               imu.angle[0], imu.angle[1], imu.angle[2]);
        HAL_Delay(200);
    }
}

/* ====================================================================
 * 思考题：
 *
 * Q1：为什么 I2C 的 SDA 要用开漏输出，而不是推挽输出？
 *     A：I2C 总线上可能挂多个从设备，大家共用同一根 SDA 线。
 *        如果用推挽输出：主机输出高，从机同时输出低 → 总线短路！
 *        开漏输出：任何设备都只能"拉低"不能"推高"，
 *        高电平靠外部上拉电阻维持。多个设备可以安全共存。
 *
 * Q2：项目为什么不用 STM32 硬件 I2C？
 *     A：STM32F4 硬件 I2C 有已知问题：在某些条件下（如从机掉电复位）
 *        会进入死锁状态（SCL/SDA一直被拉低），必须复位整个MCU才能恢复。
 *        软件 I2C 完全由GPIO控制，可以随时强制停止和重试，不会死锁。
 *        缺点是速度慢（几十kbps），但对于低速IMU（几十Hz更新率）够用。
 *
 * Q3：IMU 的 Yaw（偏航角）精度如何，能用于导航吗？
 *     A：WIT Motion IMU 通常用磁力计融合的9轴算法，Yaw 精度约±2°。
 *        短期（几分钟）稳定性好，长期会有漂移（陀螺仪积分误差）。
 *        用于本项目的航向控制（转向几十度）足够，但不适合长时间累积导航。
 *        长距离精确导航需要结合 GPS 修正偏航角漂移。
 *
 * Q4：I2C 地址 0x50 是7位还是8位？
 *     A：0x50 是 7 位地址（0101000b）。
 *        发送时需要在最低位加读写位：
 *        写：(0x50 << 1) | 0 = 0xA0
 *        读：(0x50 << 1) | 1 = 0xA1
 *        代码里 (IMU_ADDR << 1) | rw 就是这个换算。
 * ==================================================================== */
