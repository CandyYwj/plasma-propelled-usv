#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// PID控制器结构体
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    
    float setpoint;     // 设定值
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分项
    float derivative;   // 微分项
    
    float output;       // 输出值
    float output_max;   // 输出最大值
    float output_min;   // 输出最小值
    
    float integral_max; // 积分限幅
    bool integral_enable; // 积分使能
    
    uint32_t last_time; // 上次计算时间
} PID_Controller_t;

// 姿态控制结构体
typedef struct {
    PID_Controller_t yaw_pid;    // 偏航角PID控制器
    PID_Controller_t pitch_pid;  // 俯仰角PID控制器
    PID_Controller_t roll_pid;   // 横滚角PID控制器
    
    float target_yaw;   // 目标偏航角
    float target_pitch; // 目标俯仰角
    float target_roll;  // 目标横滚角
    
    bool yaw_control_enable;   // 偏航控制使能
    bool pitch_control_enable; // 俯仰控制使能
    bool roll_control_enable;  // 横滚控制使能
} Attitude_Controller_t;

// 运动控制输出结构体
typedef struct {
    float left_motor_pwm;   // 左电机PWM占空比
    float right_motor_pwm;  // 右电机PWM占空比
    float forward_pwm;      // 前进PWM占空比
    float turn_pwm;         // 转向PWM占空比
} Motion_Output_t;

// 函数声明
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float output_max, float output_min);
void PID_SetSetpoint(PID_Controller_t *pid, float setpoint);
void PID_SetGains(PID_Controller_t *pid, float kp, float ki, float kd);
void PID_Reset(PID_Controller_t *pid);
float PID_Calculate(PID_Controller_t *pid, float input, uint32_t current_time);
void PID_SetIntegralLimit(PID_Controller_t *pid, float integral_max);
void PID_EnableIntegral(PID_Controller_t *pid, bool enable);

// 姿态控制函数
void Attitude_Controller_Init(Attitude_Controller_t *attitude_ctrl);
void Attitude_Controller_SetTarget(Attitude_Controller_t *attitude_ctrl, float yaw, float pitch, float roll);
void Attitude_Controller_Enable(Attitude_Controller_t *attitude_ctrl, bool yaw_enable, bool pitch_enable, bool roll_enable);
Motion_Output_t Attitude_Controller_Update(Attitude_Controller_t *attitude_ctrl, float current_yaw, float current_pitch, float current_roll, uint32_t current_time);

// 角度处理函数
float Normalize_Angle(float angle);
float Angle_Difference(float angle1, float angle2);

#endif /* __PID_H */ 
