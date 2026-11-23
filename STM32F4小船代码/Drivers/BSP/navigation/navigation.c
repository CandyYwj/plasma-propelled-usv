#include "navigation.h"
#include <string.h>
#include <stdlib.h>

// 地球半径 (米)
#define EARTH_RADIUS 6371000.0

// 导航控制器初始化
void Navigation_Init(Navigation_Controller_t *nav_ctrl)
{
    // 初始化导航状态
    nav_ctrl->nav_state.current_position.latitude = 0.0;
    nav_ctrl->nav_state.current_position.longitude = 0.0;
    nav_ctrl->nav_state.current_position.is_valid = false;
    
    nav_ctrl->nav_state.target_position.latitude = 0.0;
    nav_ctrl->nav_state.target_position.longitude = 0.0;
    nav_ctrl->nav_state.target_position.is_valid = false;
    
    nav_ctrl->nav_state.target_bearing = 0.0f;
    nav_ctrl->nav_state.distance_to_target = 0.0f;
    nav_ctrl->nav_state.navigation_active = false;
    nav_ctrl->nav_state.target_reached = false;
    nav_ctrl->nav_state.arrival_threshold = 5.0f; // 5米到达阈值
    
    // 初始化控制参数
    nav_ctrl->max_speed_pwm = 40.0f;      // 最大速度40% PWM
    nav_ctrl->min_speed_pwm = 15.0f;      // 最小速度15% PWM
    nav_ctrl->speed_control_kp = 0.5f;    // 速度控制比例系数
    nav_ctrl->bearing_control_kp = 1.0f;  // 航向控制比例系数
    nav_ctrl->distance_slowdown_threshold = 20.0f; // 20米开始减速
}

// 设置导航目标
void Navigation_SetTarget(Navigation_Controller_t *nav_ctrl, double target_lat, double target_lon)
{
    if (GPS_Coordinate_IsValid(target_lat, target_lon)) {
        nav_ctrl->nav_state.target_position.latitude = target_lat;
        nav_ctrl->nav_state.target_position.longitude = target_lon;
        nav_ctrl->nav_state.target_position.is_valid = true;
        nav_ctrl->nav_state.target_reached = false;
    }
}

// 开始导航
void Navigation_Start(Navigation_Controller_t *nav_ctrl)
{
    if (nav_ctrl->nav_state.target_position.is_valid) {
        nav_ctrl->nav_state.navigation_active = true;
        nav_ctrl->nav_state.target_reached = false;
    }
}

// 停止导航
void Navigation_Stop(Navigation_Controller_t *nav_ctrl)
{
    nav_ctrl->nav_state.navigation_active = false;
    nav_ctrl->nav_state.target_reached = false;
}

// 检查导航是否激活
bool Navigation_IsActive(Navigation_Controller_t *nav_ctrl)
{
    return nav_ctrl->nav_state.navigation_active;
}

// 检查是否到达目标
bool Navigation_IsTargetReached(Navigation_Controller_t *nav_ctrl)
{
    return nav_ctrl->nav_state.target_reached;
}

// 计算两点间的方位角 (度)
float Calculate_Bearing(double lat1, double lon1, double lat2, double lon2)
{
    // 转换为弧度
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;
    
    // 计算方位角
    double delta_lon = lon2_rad - lon1_rad;
    double y = sin(delta_lon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon);
    
    double bearing_rad = atan2(y, x);
    float bearing_deg = (float)(bearing_rad * 180.0 / M_PI);
    
    return Normalize_Bearing(bearing_deg);
}

// 计算两点间的距离 (米)
float Calculate_Distance(double lat1, double lon1, double lat2, double lon2)
{
    // 转换为弧度
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;
    
    // 使用Haversine公式计算距离
    double delta_lat = lat2_rad - lat1_rad;
    double delta_lon = lon2_rad - lon1_rad;
    
    double a = sin(delta_lat/2) * sin(delta_lat/2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(delta_lon/2) * sin(delta_lon/2);
    
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double distance = EARTH_RADIUS * c;
    
    return (float)distance;
}

// 标准化方位角到[0, 360)度
float Normalize_Bearing(float bearing)
{
    while (bearing < 0.0f) {
        bearing += 360.0f;
    }
    while (bearing >= 360.0f) {
        bearing -= 360.0f;
    }
    return bearing;
}

// 计算两个方位角之间的差值（考虑角度连续性）
float Bearing_Difference(float bearing1, float bearing2)
{
    float diff = bearing1 - bearing2;
    
    // 处理角度连续性
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }
    
    return diff;
}

// 将GPS字符串转换为双精度浮点数
double GPS_String_To_Double(const char* gps_str)
{
    if (gps_str == NULL || strlen(gps_str) == 0) {
        return 0.0;
    }
    
    return atof(gps_str);
}

// 检查GPS坐标是否有效
bool GPS_Coordinate_IsValid(double lat, double lon)
{
    // 检查纬度范围 [-90, 90]
    if (lat < -90.0 || lat > 90.0) {
        return false;
    }
    
    // 检查经度范围 [-180, 180]
    if (lon < -180.0 || lon > 180.0) {
        return false;
    }
    
    // 检查是否为0坐标（通常表示无效）
    if (lat == 0.0 && lon == 0.0) {
        return false;
    }
    
    return true;
}

// 导航更新函数 - 返回目标航向角
float Navigation_Update(Navigation_Controller_t *nav_ctrl, double current_lat, double current_lon, float current_yaw)
{
    if (!nav_ctrl->nav_state.navigation_active || !nav_ctrl->nav_state.target_position.is_valid) {
        return current_yaw; // 返回当前航向，不改变
    }
    
    // 更新当前位置
    nav_ctrl->nav_state.current_position.latitude = current_lat;
    nav_ctrl->nav_state.current_position.longitude = current_lon;
    nav_ctrl->nav_state.current_position.is_valid = true;
    
    // 计算到目标的距离
    nav_ctrl->nav_state.distance_to_target = Calculate_Distance(
        current_lat, current_lon,
        nav_ctrl->nav_state.target_position.latitude,
        nav_ctrl->nav_state.target_position.longitude
    );
    
    // 检查是否到达目标
    if (nav_ctrl->nav_state.distance_to_target <= nav_ctrl->nav_state.arrival_threshold) {
        nav_ctrl->nav_state.target_reached = true;
        nav_ctrl->nav_state.navigation_active = false;
        return current_yaw; // 到达目标，保持当前航向
    }
    
    // 计算目标方位角
    nav_ctrl->nav_state.target_bearing = Calculate_Bearing(
        current_lat, current_lon,
        nav_ctrl->nav_state.target_position.latitude,
        nav_ctrl->nav_state.target_position.longitude
    );
    
    // 计算航向误差
    float bearing_error = Bearing_Difference(nav_ctrl->nav_state.target_bearing, current_yaw);
    
    // 简单的航向控制：直接返回目标方位角
    // 这里可以根据需要实现更复杂的控制算法
    return nav_ctrl->nav_state.target_bearing;
}
