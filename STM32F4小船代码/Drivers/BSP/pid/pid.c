#include "pid.h"
#include <math.h>
#include <string.h>

// PID控制器初始化
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float output_max, float output_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->integral_max = output_max * 0.5f; // 积分限幅为输出限幅的一半
    pid->integral_enable = true;
    pid->last_time = 0;
}

// 设置PID设定值
void PID_SetSetpoint(PID_Controller_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

// 设置PID参数
void PID_SetGains(PID_Controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

// 重置PID控制器
void PID_Reset(PID_Controller_t *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->last_time = 0;
}

// PID计算函数
float PID_Calculate(PID_Controller_t *pid, float input, uint32_t current_time)
{
    // 计算时间间隔（毫秒）
    float dt = 0.0f;
    if (pid->last_time != 0) {
        dt = (float)(current_time - pid->last_time) / 1000.0f; // 转换为秒
        if (dt <= 0.0f || dt > 1.0f) { // 防止异常时间间隔
            dt = 0.01f; // 默认10ms
        }
    } else {
        dt = 0.01f; // 第一次调用，假设10ms
    }
    
    // 计算误差
    pid->error = pid->setpoint - input;
    
    // 比例项
    float proportional = pid->kp * pid->error;
    
    // 积分项
    if (pid->integral_enable) {
        pid->integral += pid->error * dt;
        
        // 积分限幅
        if (pid->integral > pid->integral_max) {
            pid->integral = pid->integral_max;
        } else if (pid->integral < -pid->integral_max) {
            pid->integral = -pid->integral_max;
        }
    }
    float integral = pid->ki * pid->integral;
    
    // 微分项
    if (dt > 0.0f) {
        pid->derivative = (pid->error - pid->last_error) / dt;
    }
    float derivative = pid->kd * pid->derivative;
    
    // 计算输出
    pid->output = proportional + integral + derivative;
    
    // 输出限幅
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    
    // 更新状态
    pid->last_error = pid->error;
    pid->last_time = current_time;
    
    return pid->output;
}

// 设置积分限幅
void PID_SetIntegralLimit(PID_Controller_t *pid, float integral_max)
{
    pid->integral_max = integral_max;
}

// 使能/禁用积分项
void PID_EnableIntegral(PID_Controller_t *pid, bool enable)
{
    pid->integral_enable = enable;
    if (!enable) {
        pid->integral = 0.0f; // 禁用时清零积分项
    }
}

// 角度标准化到[-180, 180]度
float Normalize_Angle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

// 计算两个角度之间的差值（考虑角度连续性）
float Angle_Difference(float angle1, float angle2)
{
    float diff = angle1 - angle2;
    return Normalize_Angle(diff);
}

// 姿态控制器初始化
void Attitude_Controller_Init(Attitude_Controller_t *attitude_ctrl)
{
    // 初始化偏航角PID控制器（航向控制）
    PID_Init(&attitude_ctrl->yaw_pid, 2.0f, 0.1f, 0.5f, 50.0f, -50.0f);
    
    // 初始化俯仰角PID控制器（前后平衡）
    PID_Init(&attitude_ctrl->pitch_pid, 1.0f, 0.05f, 0.2f, 30.0f, -30.0f);
    
    // 初始化横滚角PID控制器（左右平衡）
    PID_Init(&attitude_ctrl->roll_pid, 1.0f, 0.05f, 0.2f, 30.0f, -30.0f);
    
    // 初始化目标角度
    attitude_ctrl->target_yaw = 0.0f;
    attitude_ctrl->target_pitch = 0.0f;
    attitude_ctrl->target_roll = 0.0f;
    
    // 默认只启用偏航控制
    attitude_ctrl->yaw_control_enable = true;
    attitude_ctrl->pitch_control_enable = false;
    attitude_ctrl->roll_control_enable = false;
}

// 设置姿态目标值
void Attitude_Controller_SetTarget(Attitude_Controller_t *attitude_ctrl, float yaw, float pitch, float roll)
{
    attitude_ctrl->target_yaw = Normalize_Angle(yaw);
    attitude_ctrl->target_pitch = Normalize_Angle(pitch);
    attitude_ctrl->target_roll = Normalize_Angle(roll);
    
    // 更新PID设定值
    PID_SetSetpoint(&attitude_ctrl->yaw_pid, attitude_ctrl->target_yaw);
    PID_SetSetpoint(&attitude_ctrl->pitch_pid, attitude_ctrl->target_pitch);
    PID_SetSetpoint(&attitude_ctrl->roll_pid, attitude_ctrl->target_roll);
}

// 使能/禁用姿态控制
void Attitude_Controller_Enable(Attitude_Controller_t *attitude_ctrl, bool yaw_enable, bool pitch_enable, bool roll_enable)
{
    attitude_ctrl->yaw_control_enable = yaw_enable;
    attitude_ctrl->pitch_control_enable = pitch_enable;
    attitude_ctrl->roll_control_enable = roll_enable;
    
    // 如果禁用某个控制，重置对应的PID
    if (!yaw_enable) {
        PID_Reset(&attitude_ctrl->yaw_pid);
    }
    if (!pitch_enable) {
        PID_Reset(&attitude_ctrl->pitch_pid);
    }
    if (!roll_enable) {
        PID_Reset(&attitude_ctrl->roll_pid);
    }
}

// 姿态控制器更新函数
Motion_Output_t Attitude_Controller_Update(Attitude_Controller_t *attitude_ctrl, float current_yaw, float current_pitch, float current_roll, uint32_t current_time)
{
    Motion_Output_t motion_output = {0};
    
    // 标准化当前角度
    current_yaw = Normalize_Angle(current_yaw);
    current_pitch = Normalize_Angle(current_pitch);
    current_roll = Normalize_Angle(current_roll);
    
    // 计算各轴的PID输出
    float yaw_output = 0.0f;
    float pitch_output = 0.0f;
    float roll_output = 0.0f;
    
    if (attitude_ctrl->yaw_control_enable) {
        yaw_output = PID_Calculate(&attitude_ctrl->yaw_pid, current_yaw, current_time);
    }
    
    if (attitude_ctrl->pitch_control_enable) {
        pitch_output = PID_Calculate(&attitude_ctrl->pitch_pid, current_pitch, current_time);
    }
    
    if (attitude_ctrl->roll_control_enable) {
        roll_output = PID_Calculate(&attitude_ctrl->roll_pid, current_roll, current_time);
    }
    
    // 运动学映射：将姿态控制输出映射到电机PWM
    // 假设：通道1控制左电机，通道2控制右电机
    
    // 基础前进PWM（可以根据需要调整）
    float base_pwm = 20.0f;
    
    // 转向控制（主要来自偏航控制）
    float turn_pwm = yaw_output;
    
    // 计算左右电机PWM
    motion_output.left_motor_pwm = base_pwm + turn_pwm;
    motion_output.right_motor_pwm = base_pwm - turn_pwm;
    
    // 添加俯仰和横滚控制的影响（如果需要）
    if (attitude_ctrl->pitch_control_enable) {
        motion_output.left_motor_pwm += pitch_output * 0.5f;
        motion_output.right_motor_pwm += pitch_output * 0.5f;
    }
    
    if (attitude_ctrl->roll_control_enable) {
        motion_output.left_motor_pwm += roll_output;
        motion_output.right_motor_pwm -= roll_output;
    }
    
    // PWM限幅
    if (motion_output.left_motor_pwm > 100.0f) {
        motion_output.left_motor_pwm = 100.0f;
    } else if (motion_output.left_motor_pwm < 0.0f) {
        motion_output.left_motor_pwm = 0.0f;
    }
    
    if (motion_output.right_motor_pwm > 100.0f) {
        motion_output.right_motor_pwm = 100.0f;
    } else if (motion_output.right_motor_pwm < 0.0f) {
        motion_output.right_motor_pwm = 0.0f;
    }
    
    // 保存中间值
    motion_output.forward_pwm = base_pwm;
    motion_output.turn_pwm = turn_pwm;
    
    return motion_output;
} 
