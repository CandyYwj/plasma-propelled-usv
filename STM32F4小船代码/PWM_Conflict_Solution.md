# PWM波形冲突问题解决方案

## 问题描述
在while(1)循环中调用PWM设置函数后，PWM波形出现异常，而在注释掉while(1)代码时PWM波形正常。

## 问题原因分析

### 1. 初始占空比设置错误
- **原问题**：PWM模式2下，初始Pulse=800，注释说是0%占空比，实际是20%占空比
- **解决方案**：修正为Pulse=1000，对应PWM模式2下的0%占空比

### 2. 多个控制模式冲突
- **原问题**：手动控制、PID控制、GPS导航可能同时修改PWM值
- **解决方案**：引入PWM控制模式管理，确保同一时间只有一个控制模式生效

### 3. 频繁的PWM设置调用
- **原问题**：PID和GPS导航循环每20ms调用一次PWM设置
- **解决方案**：添加模式检查，只在对应模式下才执行PWM设置

## 解决方案详细说明

### 1. PWM控制模式管理

```c
typedef enum {
    PWM_MODE_MANUAL = 0,    // 手动控制模式
    PWM_MODE_PID = 1,       // PID控制模式
    PWM_MODE_GPS = 2        // GPS导航模式
} PWM_Control_Mode_t;

PWM_Control_Mode_t current_pwm_mode = PWM_MODE_MANUAL;
```

### 2. 模式切换逻辑

#### 手动控制命令
```c
// 前进命令
current_pwm_mode = PWM_MODE_MANUAL;
pid_control_enable = 0;
gps_navigation_enable = 0;
TIM1_PWM_SetDuty(PWM_CHANNEL_1, 80.0f);
TIM1_PWM_SetDuty(PWM_CHANNEL_2, 80.0f);
```

#### PID控制命令
```c
// 启用PID控制
current_pwm_mode = PWM_MODE_PID;
pid_control_enable = 1;
gps_navigation_enable = 0;  // 关闭GPS导航
```

#### GPS导航命令
```c
// 开始GPS导航
current_pwm_mode = PWM_MODE_GPS;
gps_navigation_enable = 1;
pid_control_enable = 0;  // 关闭PID控制
```

### 3. 控制循环优化

```c
// 只在对应模式下执行控制
if ((pid_control_enable && current_pwm_mode == PWM_MODE_PID) || 
    (gps_navigation_enable && current_pwm_mode == PWM_MODE_GPS)) {
    
    // PID控制检查
    if (pid_control_enable && current_pwm_mode == PWM_MODE_PID) {
        // 执行PID控制
    }
    
    // GPS导航检查
    if (gps_navigation_enable && current_pwm_mode == PWM_MODE_GPS) {
        // 执行GPS导航控制
    }
}
```

## 修复的具体问题

### 1. TIM初始化修正
```c
// 修正前
sConfigOC.Pulse = 800;  // 错误：实际是20%占空比

// 修正后
sConfigOC.Pulse = 1000;  // 正确：PWM模式2下0%占空比
```

### 2. 控制模式优先级
- **手动控制**：最高优先级，立即生效
- **PID控制**：中等优先级，需要启用
- **GPS导航**：最低优先级，需要启用且有效目标

### 3. 状态同步
- 切换控制模式时，自动关闭其他模式
- 确保PWM值的一致性

## 测试建议

### 1. 基础测试
```c
// 测试停止状态
TIM1_PWM_SetDuty(PWM_CHANNEL_1, 0.0f);
TIM1_PWM_SetDuty(PWM_CHANNEL_2, 0.0f);
// 应该输出0%占空比波形
```

### 2. 手动控制测试
```c
// 发送命令：advance
// 应该输出80%占空比波形
// 其他控制模式应该被禁用
```

### 3. PID控制测试
```c
// 发送命令：pid_on
// 应该进入PID控制模式
// 手动控制应该被禁用
```

### 4. GPS导航测试
```c
// 发送命令：nav_start
// 应该进入GPS导航模式
// PID控制应该被禁用
```

## 注意事项

### 1. PWM模式2特性
- Compare值越小，占空比越大
- 占空比 = (Period - Compare) / Period
- 初始值1000对应0%占空比

### 2. 控制模式切换
- 切换模式时会自动停止电机
- 确保安全的状态转换

### 3. 调试建议
- 使用示波器观察PWM波形
- 检查控制模式状态
- 验证PWM值计算正确性

## 预期效果

修复后应该实现：
1. **稳定的PWM波形**：不再出现波形异常
2. **正确的控制模式切换**：避免多个模式冲突
3. **准确的占空比控制**：PWM值计算正确
4. **安全的操作**：模式切换时自动停止电机

## 验证方法

1. **示波器测试**：观察PWM波形是否稳定
2. **功能测试**：验证各控制模式正常工作
3. **冲突测试**：确保模式切换时无冲突
4. **长时间测试**：验证系统稳定性 