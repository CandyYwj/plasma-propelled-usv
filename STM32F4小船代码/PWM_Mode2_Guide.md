# PWM模式2使用说明

## 概述
本系统使用STM32的PWM模式2来驱动电机，PWM模式2具有特定的工作特性和占空比计算方式。

## PWM模式2特性

### 工作原理
- **CNT < CCR时**：输出有效电平（高电平）
- **CNT >= CCR时**：输出无效电平（低电平）
- **占空比公式**：占空比 = (Period - Compare) / Period

### 与PWM模式1的区别
| 特性 | PWM模式1 | PWM模式2 |
|------|----------|----------|
| CNT < CCR | 输出高电平 | 输出高电平 |
| CNT >= CCR | 输出低电平 | 输出低电平 |
| 占空比计算 | Compare/Period | (Period-Compare)/Period |
| Compare值 | 值越大占空比越大 | 值越小占空比越大 |

## 配置参数

### 基本参数
```c
#define PWM_FREQUENCY    1000    // PWM频率 1kHz
#define PWM_RESOLUTION   1000    // PWM分辨率 1000级
#define PWM_CHANNEL_1    1       // 通道1
#define PWM_CHANNEL_2    2       // 通道2
```

### 硬件连接
- **PE9** → TIM1_CH1 → 通道1 → 左电机
- **PE11** → TIM1_CH2 → 通道2 → 右电机

## 占空比设置

### 百分比方式设置
```c
// 设置通道1为50%占空比
TIM1_PWM_SetDuty(PWM_CHANNEL_1, 50.0f);

// 设置通道2为30%占空比
TIM1_PWM_SetDuty(PWM_CHANNEL_2, 30.0f);
```

### 原始值方式设置
```c
// PWM模式2下，原始值与占空比的关系
// 原始值 = PWM_RESOLUTION * (1 - 占空比/100)
// 例如：50%占空比 = 1000 * (1 - 50/100) = 500

// 设置通道1为50%占空比（原始值500）
TIM1_PWM_SetDutyRaw(PWM_CHANNEL_1, 500);

// 设置通道2为30%占空比（原始值700）
TIM1_PWM_SetDutyRaw(PWM_CHANNEL_2, 700);
```

## 占空比对照表

| 占空比 | 原始值 | 说明 |
|--------|--------|------|
| 0% | 1000 | 电机停止 |
| 10% | 900 | 低速 |
| 25% | 750 | 中低速 |
| 50% | 500 | 中速 |
| 75% | 250 | 中高速 |
| 100% | 0 | 全速 |

## 实际应用示例

### 1. 前进控制
```c
// 双通道50%占空比前进
TIM1_PWM_SetDuty(PWM_CHANNEL_1, 50.0f);  // 左电机50%
TIM1_PWM_SetDuty(PWM_CHANNEL_2, 50.0f);  // 右电机50%
```

### 2. 左转控制
```c
// 左转：左电机50%，右电机0%
TIM1_PWM_SetDuty(PWM_CHANNEL_1, 50.0f);  // 左电机50%
TIM1_PWM_SetDuty(PWM_CHANNEL_2, 0.0f);   // 右电机停止
```

### 3. 右转控制
```c
// 右转：左电机0%，右电机50%
TIM1_PWM_SetDuty(PWM_CHANNEL_1, 0.0f);   // 左电机停止
TIM1_PWM_SetDuty(PWM_CHANNEL_2, 50.0f);  // 右电机50%
```

### 4. 停止控制
```c
// 停止所有电机
TIM1_PWM_SetDuty(PWM_CHANNEL_1, 0.0f);   // 左电机停止
TIM1_PWM_SetDuty(PWM_CHANNEL_2, 0.0f);   // 右电机停止
```

## PID控制中的PWM应用

### 差速控制
```c
// 基础前进PWM
float base_pwm = 25.0f;

// 转向控制（PID输出）
float turn_pwm = yaw_error * 0.5f;

// 计算左右电机PWM（PWM模式2）
float left_pwm = base_pwm + turn_pwm;
float right_pwm = base_pwm - turn_pwm;

// 应用PWM控制
TIM1_PWM_SetDuty(PWM_CHANNEL_1, left_pwm);
TIM1_PWM_SetDuty(PWM_CHANNEL_2, right_pwm);
```

## 注意事项

### 1. 占空比范围
- 有效范围：0.0% - 100.0%
- 超出范围会自动限制到边界值

### 2. 电机响应
- 不同电机可能有不同的启动阈值
- 建议最小占空比不低于10%

### 3. 电池电压影响
- 电池电压下降会影响电机转速
- 可能需要根据电池状态调整PWM值

### 4. 安全考虑
- 首次使用时建议从低占空比开始测试
- 确保电机连接正确，避免短路

## 调试建议

### 1. 测试步骤
```c
// 1. 测试停止状态
TIM1_PWM_SetDuty(PWM_CHANNEL_1, 0.0f);
TIM1_PWM_SetDuty(PWM_CHANNEL_2, 0.0f);

// 2. 测试低速前进
TIM1_PWM_SetDuty(PWM_CHANNEL_1, 10.0f);
TIM1_PWM_SetDuty(PWM_CHANNEL_2, 10.0f);

// 3. 测试转向
TIM1_PWM_SetDuty(PWM_CHANNEL_1, 20.0f);
TIM1_PWM_SetDuty(PWM_CHANNEL_2, 0.0f);
```

### 2. 常见问题
- **电机不转**：检查PWM输出和电机连接
- **转向相反**：交换左右电机连接
- **转速异常**：检查PWM频率和占空比设置

## 性能优化

### 1. PWM频率选择
- 1kHz：适合大多数应用
- 更高频率：减少电机噪音，但增加功耗
- 更低频率：降低功耗，但可能产生噪音

### 2. 分辨率选择
- 1000级：提供足够的控制精度
- 更高分辨率：更精细的控制，但增加计算量

### 3. 占空比优化
- 根据实际测试调整最佳占空比范围
- 考虑电机效率和电池寿命 