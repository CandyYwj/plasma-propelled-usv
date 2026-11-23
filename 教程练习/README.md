# 无人船项目练习文件索引

对应教程：`无人船项目手把手学习教程.md`

---

## 文件目录

```
教程练习/
├── 模块1_多路UART通信/
│   └── uart_dma_practice.c      UART DMA + 空闲中断接收
├── 模块2_PWM双路电机/
│   └── pwm_motor_practice.c     TIM1 PWM Mode2 + 安全状态机
├── 模块3_传感器驱动/
│   └── sensor_practice.c        DS18B20 1-Wire + TDS ADC
├── 模块4_中值滤波/
│   └── median_filter_practice.c 环形缓冲 + 插入排序中值滤波
├── 模块5_IMU软件I2C/
│   └── imu_i2c_practice.c       软件 I2C + WIT IMU 读取
├── 模块6_PID姿态控制/
│   └── pid_practice.c           时间基 PID + 偏航角控制
├── 模块7_GPS导航/
│   └── gps_nav_practice.c       NMEA 解析 + Haversine + 航向角
└── 模块8_主控状态机/
    └── state_machine_practice.c 命令解析 + 报告帧 + 主循环调度
```

---

## 各模块学习要点

| 模块 | 关键概念 | 对应项目文件 |
|------|----------|-------------|
| 1 多路UART | DMA + 空闲中断，3步接收模式 | Drivers/BSP/uart8/uart8.c |
| 2 PWM电机 | Mode2 公式：CCR = ARR×(1-duty/100) | Drivers/BSP/TIM/tim.c |
| 3 传感器 | DS18B20 760ms 转换，TDS 温度补偿 | Drivers/BSP/ds18b20/，tds/ |
| 4 中值滤波 | 环形缓冲，冷启动处理 | Drivers/BSP/filtering/ |
| 5 软件I2C | 开漏输出，位操作时序 | Drivers/BSP/iic/，IMU/ |
| 6 PID控制 | 基于 dt 的积分微分，积分限幅 | Drivers/BSP/pid/pid.c |
| 7 GPS导航 | DDMM.MMMM→度，Haversine，atan2 | Drivers/BSP/GPS/，navigation/ |
| 8 状态机 | 三模式互斥，命令解析，报告帧 | Users/main.c |

---

## 使用方法

每个练习文件都是**独立的**，包含：
- 完整的数据结构定义
- 学习目标注释
- 核心函数实现（与项目原文件逻辑对应）
- 测试/演示函数（可直接调用打印结果）
- 思考题及解答（理解设计背后的原因）

建议学习顺序：按模块编号 1→8，每个模块：
1. 先读注释理解原理
2. 对照项目原始代码（路径见文件顶部）
3. 手动输入一遍（不要复制粘贴）
4. 回答思考题，验证理解

---

## 硬件速查

| 外设 | 引脚/资源 | 备注 |
|------|-----------|------|
| 左电机 PWM | PE9 → TIM1_CH1 | PWM Mode2，1kHz |
| 右电机 PWM | PE11 → TIM1_CH2 | PWM Mode2，1kHz |
| DS18B20 | GPIOB PIN0 | 1-Wire，需 4.7kΩ 上拉 |
| TDS ADC | PA0 → ADC1_CH0 | 12位，480周期采样 |
| IMU SCL | GPIOB PIN10 | 软件 I2C 时钟 |
| IMU SDA | GPIOB PIN11 | 软件 I2C 数据（开漏） |
| GPS UART | USART6 | DMA + 空闲中断 |
| 上位机 UART | UART8 | DMA + 空闲中断 |
| 超声波 UART | UART1 | 串口型传感器 |
| 激光 UART | UART2 | 串口型传感器 |
