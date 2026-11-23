#include "sys.h"
#include "uart1.h"
#include "delay.h"
#include <stdio.h>   // 为了使用sprintf
#include <string.h>  // 为了使用strlen
#include <stdlib.h>  // 为了使用atof
#include <math.h>    // 为了使用fabs函数
#include "IMU.h"     // 添加IMU头文件
#include "filtering.h" // 包含滤波模块头文件
#include "pid.h"       // 包含PID控制模块头文件
#include "navigation.h" // 包含GPS导航模块头文件
#include "uart8.h" // 添加UART8头文件
#include "tim.h"   // 添加TIM PWM头文件

// TODO: 包含其他模块的头文件
#include "uart1.h" // 超声波模块
#include "ds18b20.h" // TMP模块
#include "tds.h" // TDS模块
#include "uart2.h" // 激光模块
#include "uart6.h" // GPS模块
#include "uart3.h" // 上位机通信模块
#include "ultrasonic.h"
#include "laser.h"      // 激光模块驱动
#include "gps.h"

// TODO: 定义用于存储传感器测量结果的变量
float current_temperature = -999.0f; // 初始值表示无效
float current_tds_value = -999.0f;   // 初始值表示无效
uint16_t current_ultrasonic_distance = 0xFFFF; // 初始值表示无效
uint16_t current_laser_distance = 0xFFFF;     // 初始值表示无效
// 其他传感器数据变量...

// 定义中值滤波器实例
MedianFilter_TypeDef ultrasonic_distance_filter;
MedianFilter_TypeDef laser_distance_filter;

// 控制连续测量的标志
uint8_t is_measuring_continuously = 0;

// PID控制相关变量
Attitude_Controller_t attitude_controller;
uint8_t pid_control_enable = 0;  // PID控制使能标志
uint32_t last_pid_time = 0;      // 上次PID计算时间

// GPS导航相关变量
Navigation_Controller_t navigation_controller;
uint8_t gps_navigation_enable = 0;  // GPS导航使能标志
uint32_t last_navigation_time = 0;  // 上次导航计算时间

// PWM控制状态管理
typedef enum {
    PWM_MODE_MANUAL = 0,    // 手动控制模式
    PWM_MODE_PID = 1,       // PID控制模式
    PWM_MODE_GPS = 2        // GPS导航模式
} PWM_Control_Mode_t;

PWM_Control_Mode_t current_pwm_mode = PWM_MODE_MANUAL;  // 当前PWM控制模式

// 添加PWM状态保护变量
typedef struct {
    float left_pwm;         // 左电机PWM值
    float right_pwm;        // 右电机PWM值
    uint32_t last_update_time; // 最后更新时间
    uint8_t is_locked;      // PWM锁定标志
} PWM_State_t;

PWM_State_t pwm_state = {0.0f, 0.0f, 0, 0};

// PWM安全设置函数
void PWM_Safe_SetDuty(float left_pwm, float right_pwm) {
    // 检查PWM值是否在合理范围内
    if (left_pwm < 0.0f) left_pwm = 0.0f;
    if (left_pwm > 100.0f) left_pwm = 100.0f;
    if (right_pwm < 0.0f) right_pwm = 0.0f;
    if (right_pwm > 100.0f) right_pwm = 100.0f;
    
    // 检查PWM变化是否过大（防止跳变）
    float left_diff = fabs(left_pwm - pwm_state.left_pwm);
    float right_diff = fabs(right_pwm - pwm_state.right_pwm);
    
    // 如果是手动控制模式，允许直接设置目标值（不进行渐变处理）
    if (current_pwm_mode == PWM_MODE_MANUAL) {
        // 直接设置目标PWM值
        pwm_state.left_pwm = left_pwm;
        pwm_state.right_pwm = right_pwm;
        pwm_state.last_update_time = HAL_GetTick();
        
        // 设置PWM输出
        TIM1_PWM_SetDuty(PWM_CHANNEL_1, left_pwm);
        TIM1_PWM_SetDuty(PWM_CHANNEL_2, right_pwm);
        return;
    }
    
    // 对于PID和GPS模式，保留渐变处理逻辑
    if (left_diff > 30.0f || right_diff > 30.0f) {
        // 如果是突然的大幅变化，进行渐变处理
        if (left_diff > 50.0f || right_diff > 50.0f) {
            // 限制单次变化幅度
            if (left_pwm > pwm_state.left_pwm) {
                left_pwm = pwm_state.left_pwm + 30.0f;
            } else {
                left_pwm = pwm_state.left_pwm - 30.0f;
            }
            if (right_pwm > pwm_state.right_pwm) {
                right_pwm = pwm_state.right_pwm + 30.0f;
            } else {
                right_pwm = pwm_state.right_pwm - 30.0f;
            }
        }
    }
    
    // 更新PWM状态
    pwm_state.left_pwm = left_pwm;
    pwm_state.right_pwm = right_pwm;
    pwm_state.last_update_time = HAL_GetTick();
    
    // 设置PWM输出
    TIM1_PWM_SetDuty(PWM_CHANNEL_1, left_pwm);
    TIM1_PWM_SetDuty(PWM_CHANNEL_2, right_pwm);
}

// PWM模式切换函数
void PWM_Switch_Mode(PWM_Control_Mode_t new_mode) {
    if (current_pwm_mode != new_mode) {
        current_pwm_mode = new_mode;
        pwm_state.is_locked = 0; // 解锁PWM
    }
}

// PWM状态初始化函数
void PWM_State_Init(void) {
    pwm_state.left_pwm = 0.0f;   // 初始PWM值0%
    pwm_state.right_pwm = 0.0f;  // 初始PWM值0%
    pwm_state.last_update_time = 0;
    pwm_state.is_locked = 0;
    current_pwm_mode = PWM_MODE_MANUAL;
    
    // 设置初始PWM输出为0%
    TIM1_PWM_SetDuty(PWM_CHANNEL_1, 0.0f);
    TIM1_PWM_SetDuty(PWM_CHANNEL_2, 0.0f);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    delay_init(); // 初始化延时函数 (DWT)

    // 初始化所有外设
    uart1_init(115200); // 超声波模块串口
    DS18B20_Init_GPIO(); // TMP模块GPIO
    TDS_Init();          // TDS模块ADC
    uart2_init(115200); // 激光模块串口初始化，波特率修改回115200bps
    MX_USART6_UART_Init(); // 串口6
    uart3_init(115200); // 上位机通信串口初始化 (原UART8)
    laser_init();       // 激光模块初始化
    IMU_Init_HAL();     // 初始化IMU

    uart8_init(115200); // 初始化UART8
    TIM1_PWM_Init();
    PWM_State_Init(); // 初始化PWM状态
    // 初始化中位值滤波器
    MedianFilter_Init(&ultrasonic_distance_filter);
    MedianFilter_Init(&laser_distance_filter);
    
    // 初始化PID姿态控制器
    Attitude_Controller_Init(&attitude_controller);
    
    // 初始化GPS导航控制器
    Navigation_Init(&navigation_controller);
	
	//注册（或设置）一个用于处理UART6数据接收的回调函数，并且在注册时，将huart6这个句柄的地址（&huart6）作为用户自定义数据传递过去。
	UART6_RegisterRxCallback(GPS_RxCpltCallback, &huart6);

	clrStruct(); // 清空Save_Data结构体
	
    // 用于存放发送报告的缓冲区
    char report_str[128];

    while(1)
    {
        /* 主循环任务调度 */
        // 1. 检查上位机命令接收完成标志 (来自UART8的空闲中断)
        if (uart8_get_rx_complete_flag())
        {
            // 获取接收到的命令
            uint16_t command_len = uart8_get_rx_len();
            uint8_t* command_buf = uart8_get_rx_buf();
            
            // 清除接收完成标志
            uart8_clear_rx_complete_flag();
            
            /* 解析接收到的命令 */
            // 检查命令是否为 "begin"
            if (command_len == strlen("begin") && strncmp((char*)command_buf, "begin", strlen("begin")) == 0)
            {
                /* 接收到 "begin" 命令，设置连续测量标志 */
                is_measuring_continuously = 1; // 设置标志，开始连续测量

                // 启动激光模块的连续测量
                laser_send_start_cmd(); // 仅发送一次启动指令

                // TODO: 触发其他传感器（超声波、GPS）的单次启动或配置，如果它们不是由begin直接触发

                // 可选：发送确认命令已接收的响应
                uart8_send_string("Begin command received. Starting continuous measurement.\r\n");

            }
            // TODO: 添加处理其他命令 (如 "stop") 的逻辑
            else if (command_len == strlen("end") && strncmp((char*)command_buf, "end", strlen("end")) == 0)
            {
                 /* 接收到 "end" 命令，清除连续测量标志 */
                 is_measuring_continuously = 0; // 清除标志，停止连续测量

                 // 发送激光模块停止指令
                 laser_send_stop_cmd();

                 // 可选：发送确认命令已停止的响应
                 uart8_send_string("End command received. Stopping continuous measurement.\r\n");
            }
            // 添加运动控制命令解析
            else if (command_len == strlen("advance") && strncmp((char*)command_buf, "advance", strlen("advance")) == 0)
            {
                /* 接收到 "advance" 命令，设置前进PWM */
                // 切换到手动控制模式
                PWM_Switch_Mode(PWM_MODE_MANUAL);
                pid_control_enable = 0;
                gps_navigation_enable = 0;
                
                PWM_Safe_SetDuty(70.0f, 70.0f);  // 通道1和通道2都设置为70%
            }
            else if (command_len == strlen("left") && strncmp((char*)command_buf, "left", strlen("left")) == 0)
            {
                /* 接收到 "left" 命令，设置左转PWM */
                // 切换到手动控制模式
                PWM_Switch_Mode(PWM_MODE_MANUAL);
                pid_control_enable = 0;
                gps_navigation_enable = 0;
                
                PWM_Safe_SetDuty(70.0f, 0.0f);  // 通道1设置为70%，通道2设置为0%
            }
            else if (command_len == strlen("right") && strncmp((char*)command_buf, "right", strlen("right")) == 0)
            {
                /* 接收到 "right" 命令，设置右转PWM */
                // 切换到手动控制模式
                PWM_Switch_Mode(PWM_MODE_MANUAL);
                pid_control_enable = 0;
                gps_navigation_enable = 0;
                
                PWM_Safe_SetDuty(0.0f, 70.0f);   // 通道1设置为0%，通道2设置为70%
            }
            else if (command_len == strlen("stop") && strncmp((char*)command_buf, "stop", strlen("stop")) == 0)
            {
                /* 接收到 "stop" 命令，停止电机 */
                // 切换到手动控制模式
                PWM_Switch_Mode(PWM_MODE_MANUAL);
                pid_control_enable = 0;
                gps_navigation_enable = 0;
                
                PWM_Safe_SetDuty(0.0f, 0.0f);   // 通道1和通道2都设置为0%
            }
            // PID控制命令
            else if (command_len == strlen("pid_on") && strncmp((char*)command_buf, "pid_on", strlen("pid_on")) == 0)
            {
                /* 启用PID控制 */
                PWM_Switch_Mode(PWM_MODE_PID);
                pid_control_enable = 1;
                gps_navigation_enable = 0;  // 关闭GPS导航
                Attitude_Controller_Enable(&attitude_controller, true, false, false); // 只启用偏航控制
                uart8_send_string("PID control enabled. Yaw control active.\r\n");
            }
            else if (command_len == strlen("pid_off") && strncmp((char*)command_buf, "pid_off", strlen("pid_off")) == 0)
            {
                /* 禁用PID控制 */
                PWM_Switch_Mode(PWM_MODE_MANUAL);
                pid_control_enable = 0;
                Attitude_Controller_Enable(&attitude_controller, false, false, false);
                // 停止电机
                PWM_Safe_SetDuty(0.0f, 0.0f);
            }
            else if (strncmp((char*)command_buf, "target_yaw:", strlen("target_yaw:")) == 0)
            {
                /* 设置目标偏航角 */
                float target_yaw = atof((char*)command_buf + strlen("target_yaw:"));
                Attitude_Controller_SetTarget(&attitude_controller, target_yaw, 0.0f, 0.0f);
                char response[64];
                sprintf(response, "Target yaw set to %.1f degrees.\r\n", target_yaw);
                uart8_send_string(response);
            }
            else if (strncmp((char*)command_buf, "pid_gains:", strlen("pid_gains:")) == 0)
            {
                /* 设置PID参数 Kp:Ki:Kd */
                char* gains_str = (char*)command_buf + strlen("pid_gains:");
                float kp, ki, kd;
                if (sscanf(gains_str, "%f:%f:%f", &kp, &ki, &kd) == 3) {
                    PID_SetGains(&attitude_controller.yaw_pid, kp, ki, kd);
                    char response[64];
                    sprintf(response, "PID gains set: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", kp, ki, kd);
                    uart8_send_string(response);
                } else {
                    uart8_send_string("Invalid PID gains format. Use: pid_gains:Kp:Ki:Kd\r\n");
                }
            }
            // GPS导航命令
            else if (strncmp((char*)command_buf, "nav_target:", strlen("nav_target:")) == 0)
            {
                /* 设置导航目标坐标 格式: nav_target:纬度,经度 */
                char* coord_str = (char*)command_buf + strlen("nav_target:");
                double target_lat, target_lon;
                if (sscanf(coord_str, "%lf,%lf", &target_lat, &target_lon) == 2) {
                    Navigation_SetTarget(&navigation_controller, target_lat, target_lon);
                    char response[128];
                    sprintf(response, "Navigation target set: Lat=%.6f, Lon=%.6f\r\n", target_lat, target_lon);
                    uart8_send_string(response);
                } else {
                    uart8_send_string("Invalid navigation target format. Use: nav_target:latitude,longitude\r\n");
                }
            }
            else if (command_len == strlen("nav_start") && strncmp((char*)command_buf, "nav_start", strlen("nav_start")) == 0)
            {
                /* 开始GPS导航 */
                if (navigation_controller.nav_state.target_position.is_valid) {
                    PWM_Switch_Mode(PWM_MODE_GPS);
                    gps_navigation_enable = 1;
                    pid_control_enable = 0;  // 关闭PID控制
                    Navigation_Start(&navigation_controller);
                    uart8_send_string("GPS navigation started.\r\n");
                } else {
                    uart8_send_string("No valid navigation target set. Use nav_target:lat,lon first.\r\n");
                }
            }
            else if (command_len == strlen("nav_stop") && strncmp((char*)command_buf, "nav_stop", strlen("nav_stop")) == 0)
            {
                /* 停止GPS导航 */
                PWM_Switch_Mode(PWM_MODE_MANUAL);
                gps_navigation_enable = 0;
                Navigation_Stop(&navigation_controller);
                // 停止电机
                PWM_Safe_SetDuty(0.0f, 0.0f);
            }
            else if (command_len == strlen("nav_status") && strncmp((char*)command_buf, "nav_status", strlen("nav_status")) == 0)
            {
                /* 查询导航状态 */
                char response[256];
                sprintf(response, "Nav Status: Active=%d, TargetReached=%d, Distance=%.1fm, Bearing=%.1f°\r\n",
                        navigation_controller.nav_state.navigation_active,
                        navigation_controller.nav_state.target_reached,
                        navigation_controller.nav_state.distance_to_target,
                        navigation_controller.nav_state.target_bearing);
                uart8_send_string(response);
            }
            else
            {   /* 接收到其他命令或无效命令 */
                // 发送错误响应给上位机
                uart8_send_string("Invalid command.\r\n");
            }

            // 清空UART8接收缓冲区 (DMA已在中断中自动重启)
            uart8_rx_clear();
        }

        // 2. 如果连续测量标志为真，执行测量和发送任务
        if (is_measuring_continuously)
        {
            /* 执行传感器测量和数据收集 */
            
            // 清空报告字符串缓冲区
            report_str[0] = '\0';

            /* 测量和读取传感器值 */
            // TDS和TMP已在各自模块函数中实现读取
            // 超声波需要发送测量指令然后等待接收和解析
            // 激光模块是异步接收，只需在主循环中尝试读取最新解析到的值

            /* 读取传感器原始值 */
            float raw_temperature = DS18B20_Read_Temperature(); // 从DS18B20获取原始温度
            float raw_tds_value = TDS_GetValue(raw_temperature);       // 从TDS模块获取原始TDS值，传入温度进行补偿
            uint16_t raw_ultrasonic_distance = ultrasonic_get_distance(); // 获取原始超声波距离
            uint16_t raw_laser_distance = laser_get_distance();         // 获取原始激光距离

            /* 更新传感器当前值和滤波 */
            current_temperature = raw_temperature; // 温度和TDS目前没有滤波
            current_tds_value = raw_tds_value;    

            // 更新超声波距离滤波器并获取滤波值
            uint16_t filtered_ultrasonic_distance = MedianFilter_Update(&ultrasonic_distance_filter, raw_ultrasonic_distance);
            current_ultrasonic_distance = filtered_ultrasonic_distance; // 使用滤波后的值更新当前值

            // 更新激光距离滤波器并获取滤波值
            uint16_t filtered_laser_distance = MedianFilter_Update(&laser_distance_filter, raw_laser_distance);
            current_laser_distance = filtered_laser_distance; // 使用滤波后的值更新当前值
			
            /* 构建发送给上位机的报告字符串 */

            // 添加TDS值到报告字符串
            if (current_tds_value > -990.0f) // 检查是否是有效值
            {
                sprintf(report_str + strlen(report_str), "TDS=%.1f\r\n", current_tds_value);
            }
            else
            {
                 sprintf(report_str + strlen(report_str), "Failed to read TDS.\r\n");
            }

            // 添加TMP值到报告字符串
            if (current_temperature > -200.0f)
            {
                 sprintf(report_str + strlen(report_str), "TMP=%.1f\r\n", current_temperature);
            }
            else
            {
                 sprintf(report_str + strlen(report_str), "Failed to read temperature.\r\n");
            }

            // 添加超声波距离值到报告字符串 (使用滤波后的值)
            if (current_ultrasonic_distance != 0xFFFF)
            {
                 sprintf(report_str + strlen(report_str), "distance=%d\r\n", current_ultrasonic_distance);
            }
            else
            {
                 sprintf(report_str + strlen(report_str), "Failed to read distance.\r\n");
            }
            
            // 添加激光距离值到报告字符串 (使用滤波后的值)
            if (current_laser_distance != 0xFFFF)
            {
                 sprintf(report_str + strlen(report_str), "laser=%d\r\n", current_laser_distance);
            }
            else
            {
                 sprintf(report_str + strlen(report_str), "Failed to read laser distance.\r\n");
            }
			
			// 读取IMU数据
            IMU_Data_t_HAL imu_data;
            IMU_ReadData_HAL(&imu_data);
            
            // 添加PID控制状态信息到报告（如果PID控制启用）
            if (pid_control_enable) {
                sprintf(report_str + strlen(report_str), "PID_STATUS:ON,%.1f\r\n", 
                        attitude_controller.target_yaw);
            }
            
            // 添加GPS导航状态信息到报告（如果GPS导航启用）
            if (gps_navigation_enable) {
                sprintf(report_str + strlen(report_str), "NAV_STATUS:ON,%.1fm,%.1f°\r\n", 
                        navigation_controller.nav_state.distance_to_target,
                        navigation_controller.nav_state.target_bearing);
            }
            
            // 添加IMU数据到报告字符串
            sprintf(report_str + strlen(report_str), "IMU_ACC:%.3f,%.3f,%.3f\r\n", 
                    imu_data.acc[0], imu_data.acc[1], imu_data.acc[2]);
            sprintf(report_str + strlen(report_str), "IMU_GYRO:%.3f,%.3f,%.3f\r\n", 
                    imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);
            sprintf(report_str + strlen(report_str), "IMU_ANGLE:%.3f,%.3f,%.3f\r\n", 
                    imu_data.angle[0], imu_data.angle[1], imu_data.angle[2]);
            sprintf(report_str + strlen(report_str), "IMU_MAG:%d,%d,%d\r\n", 
                    imu_data.mag[0], imu_data.mag[1], imu_data.mag[2]);
			
            // 添加GPS数据到报告字符串
            if (Save_Data.isParseData && Save_Data.isUsefull) {
                double f_latitude = Convert_to_degrees(Save_Data.latitude);
                double f_longitude = Convert_to_degrees(Save_Data.longitude);
                sprintf(report_str + strlen(report_str), "GPS_TIME=%s\r\n", Save_Data.UTCTime);
                sprintf(report_str + strlen(report_str), "GPS_LAT=%.6f%s\r\n", f_latitude, Save_Data.N_S);
                sprintf(report_str + strlen(report_str), "GPS_LON=%.6f%s\r\n", f_longitude, Save_Data.E_W);
            }
			
            // 发送整个报告字符串
            if (strlen(report_str) > 0) {
                 uart8_send_string(report_str);
            }

            // 添加帧结束标记
            uart8_send_string("end\r\n");

            // 控制发送频率
            HAL_Delay(200); // 每200ms发送一次数据帧
        }

        // 3. PID控制和GPS导航循环（独立于测量循环，更高频率）
        if ((pid_control_enable && current_pwm_mode == PWM_MODE_PID) || 
            (gps_navigation_enable && current_pwm_mode == PWM_MODE_GPS)) {
            uint32_t current_time = HAL_GetTick();
            
            // 每20ms执行一次控制（50Hz）
            if (current_time - last_pid_time >= 20) {
                // 读取IMU数据
                IMU_Data_t_HAL imu_data;
                IMU_ReadData_HAL(&imu_data);
                
                float target_yaw = imu_data.angle[2]; // 默认使用当前航向
                
                // GPS导航控制（优先级高于PID控制）
                if (gps_navigation_enable && current_pwm_mode == PWM_MODE_GPS && 
                    Save_Data.isParseData && Save_Data.isUsefull) {
                    // 获取当前GPS坐标
                    double current_lat = Convert_to_degrees(Save_Data.latitude);
                    double current_lon = Convert_to_degrees(Save_Data.longitude);
                    
                    // 如果是南半球，纬度取负值
                    if (Save_Data.N_S[0] == 'S') {
                        current_lat = -current_lat;
                    }
                    
                    // 如果是西半球，经度取负值
                    if (Save_Data.E_W[0] == 'W') {
                        current_lon = -current_lon;
                    }
                    
                    // 更新导航并获取目标航向
                    target_yaw = Navigation_Update(&navigation_controller, current_lat, current_lon, imu_data.angle[2]);
                    
                    // 检查是否到达目标
                    if (Navigation_IsTargetReached(&navigation_controller)) {
                        PWM_Switch_Mode(PWM_MODE_MANUAL);
                        gps_navigation_enable = 0;
                        PWM_Safe_SetDuty(0.0f, 0.0f); // 停止电机
                    }
                }
                // PID控制（当GPS导航未启用时）
                else if (pid_control_enable && current_pwm_mode == PWM_MODE_PID) {
                    // 执行PID姿态控制
                    Motion_Output_t motion_output = Attitude_Controller_Update(
                        &attitude_controller, 
                        imu_data.angle[2],  // Yaw角 (偏航)
                        imu_data.angle[1],  // Pitch角 (俯仰)
                        imu_data.angle[0],  // Roll角 (横滚)
                        current_time
                    );
                    
                    // 应用PID控制输出到电机
                    PWM_Safe_SetDuty(motion_output.left_motor_pwm, motion_output.right_motor_pwm);
                    
                    last_pid_time = current_time;
                    continue; // 跳过下面的GPS导航控制
                }
                
                // GPS导航控制输出
                if (gps_navigation_enable && current_pwm_mode == PWM_MODE_GPS) {
                    // 计算航向误差
                    float yaw_error = target_yaw - imu_data.angle[2];
                    
                    // 处理角度连续性
                    if (yaw_error > 180.0f) {
                        yaw_error -= 360.0f;
                    } else if (yaw_error < -180.0f) {
                        yaw_error += 360.0f;
                    }
                    
                    // 简单的航向控制
                    float base_pwm = 25.0f; // 基础前进PWM
                    float turn_pwm = yaw_error * 0.5f; // 航向控制增益
                    
                    // 根据距离调整速度
                    if (navigation_controller.nav_state.distance_to_target < navigation_controller.distance_slowdown_threshold) {
                        base_pwm = navigation_controller.min_speed_pwm;
                    }
                    
                    // 计算左右电机PWM
                    float left_pwm = base_pwm + turn_pwm;
                    float right_pwm = base_pwm - turn_pwm;
                    
                    // PWM限幅
                    if (left_pwm > 100.0f) left_pwm = 100.0f;
                    if (left_pwm < 0.0f) left_pwm = 0.0f;
                    if (right_pwm > 100.0f) right_pwm = 100.0f;
                    if (right_pwm < 0.0f) right_pwm = 0.0f;
                    
                    // 应用GPS导航控制输出到电机
                    PWM_Safe_SetDuty(left_pwm, right_pwm);
                }
                
                last_pid_time = current_time;
            }
        }
        
        // 4. 检查UART6接收完成标志 - GPS数据异步接收处理
        
        // 5. PWM状态监控（每5秒检查一次）
        static uint32_t last_pwm_check_time = 0;
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_pwm_check_time >= 5000) { // 每5秒检查一次
            // 检查PWM状态是否异常
            if (pwm_state.left_pwm < 0.0f || pwm_state.left_pwm > 100.0f ||
                pwm_state.right_pwm < 0.0f || pwm_state.right_pwm > 100.0f) {
                PWM_Safe_SetDuty(0.0f, 0.0f); // 重置到安全值（0%）
            }
            last_pwm_check_time = current_time;
        }
        
        // 短延时避免空转，等待中断或标志 (如果在连续测量模式下，上面HAL_Delay已控制频率)
        // 如果is_measuring_continuously为真，上面已经有HAL_Delay了，这里可以省略或调整
        if (!is_measuring_continuously) {
             HAL_Delay(10); // 例如延时10ms
        }
    }
}

