#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// 定义M_PI常量（如果编译器没有定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// GPS坐标结构体
typedef struct {
    double latitude;   // 纬度 (度)
    double longitude;  // 经度 (度)
    bool is_valid;     // 坐标是否有效
} GPS_Coordinate_t;

// 导航状态结构体
typedef struct {
    GPS_Coordinate_t current_position;  // 当前位置
    GPS_Coordinate_t target_position;   // 目标位置
    float target_bearing;               // 目标航向角 (度)
    float distance_to_target;           // 到目标的距离 (米)
    bool navigation_active;             // 导航是否激活
    bool target_reached;                // 是否到达目标
    float arrival_threshold;            // 到达阈值 (米)
} Navigation_State_t;

// 导航控制结构体
typedef struct {
    Navigation_State_t nav_state;
    float max_speed_pwm;        // 最大速度PWM
    float min_speed_pwm;        // 最小速度PWM
    float speed_control_kp;     // 速度控制比例系数
    float bearing_control_kp;   // 航向控制比例系数
    float distance_slowdown_threshold; // 开始减速的距离阈值
} Navigation_Controller_t;

// 函数声明
void Navigation_Init(Navigation_Controller_t *nav_ctrl);
void Navigation_SetTarget(Navigation_Controller_t *nav_ctrl, double target_lat, double target_lon);
void Navigation_Start(Navigation_Controller_t *nav_ctrl);
void Navigation_Stop(Navigation_Controller_t *nav_ctrl);
float Navigation_Update(Navigation_Controller_t *nav_ctrl, double current_lat, double current_lon, float current_yaw);
bool Navigation_IsActive(Navigation_Controller_t *nav_ctrl);
bool Navigation_IsTargetReached(Navigation_Controller_t *nav_ctrl);

// GPS计算函数
float Calculate_Bearing(double lat1, double lon1, double lat2, double lon2);
float Calculate_Distance(double lat1, double lon1, double lat2, double lon2);
float Normalize_Bearing(float bearing);
float Bearing_Difference(float bearing1, float bearing2);

// 坐标转换函数
double GPS_String_To_Double(const char* gps_str);
bool GPS_Coordinate_IsValid(double lat, double lon);

#endif /* __NAVIGATION_H */ 
