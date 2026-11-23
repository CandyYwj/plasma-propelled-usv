/**
 * @file    gps_nav_practice.c
 * @brief   模块七练习：GPS NMEA 解析 + Haversine 距离 + 航向角计算
 *
 * 学习目标：
 *   1. 理解 NMEA GNRMC 语句格式，掌握字段提取
 *   2. 掌握 DDMM.MMMM → 十进制度 的坐标格式转换
 *   3. 理解 Haversine 球面距离公式（弧度→米）
 *   4. 掌握 atan2 航向角计算，理解 [0,360) 归一化
 *   5. 理解 Navigation_Update 如何把 GPS 坐标转换成 PID 目标偏航角
 *
 * 对应项目文件：
 *   Drivers/BSP/GPS/gps.c
 *   Drivers/BSP/navigation/navigation.c
 *
 * 验证方法：
 *   1. 喂入真实 GNRMC 字符串，检查解析出的纬度/经度是否正确
 *   2. 计算已知两点间距离（如北京→上海，约1067km）
 *   3. 给定当前位置和目标位置，验证航向角方向是否合理
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>   /* atof */
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

/* ====================================================================
 * 常量定义
 * ==================================================================== */
#define M_PI_PRACTICE    3.14159265358979323846
#define EARTH_RADIUS     6371000.0   /* 地球平均半径（米）*/

/* GNRMC 字段缓冲区长度 */
#define GPS_BUF_LEN      128
#define FIELD_LEN        32

/* ====================================================================
 * 数据结构
 * ==================================================================== */

/* GPS 原始数据（从 GNRMC 语句提取的字符串字段）*/
typedef struct {
    char utc_time[16];   /* hhmmss.ss */
    char latitude[16];   /* DDMM.MMMM */
    char ns[4];          /* "N" 或 "S" */
    char longitude[16];  /* DDDMM.MMMM */
    char ew[4];          /* "E" 或 "W" */
    bool is_valid;       /* 'A'=有效，'V'=无效 */
    bool is_parsed;      /* 已解析标志 */
} GPS_RawData_t;

/* GPS 位置（十进制度）*/
typedef struct {
    double latitude;
    double longitude;
    bool   valid;
} GPS_Position_t;

/* 导航状态 */
typedef struct {
    GPS_Position_t current;    /* 当前位置 */
    GPS_Position_t target;     /* 目标位置 */
    float target_bearing;      /* 目标航向 [0,360) */
    float distance_to_target;  /* 距目标距离（米）*/
    bool  nav_active;          /* 导航使能 */
    bool  target_reached;      /* 是否到达 */
    float arrival_threshold;   /* 到达阈值（米），默认5m */
} NavState_t;

/* ====================================================================
 * Part 1：NMEA GNRMC 字段提取
 * ==================================================================== */

/**
 * @brief  解析 GNRMC 语句，提取各字段
 * @param  sentence  原始 GNRMC 字符串（如 "$GNRMC,..."）
 * @param  raw       输出：解析后的字段结构
 *
 * GNRMC 语句格式（逗号分隔的字段）：
 *   $GNRMC,<时间>,<有效性>,<纬度>,<N/S>,<经度>,<E/W>,<速度>,<航向>,<日期>,...
 *   字段编号（从0起）：
 *     0: 语句类型 $GNRMC
 *     1: UTC时间 hhmmss.ss
 *     2: 有效性 A=有效 V=无效
 *     3: 纬度   DDMM.MMMM
 *     4: N/S
 *     5: 经度   DDDMM.MMMM
 *     6: E/W
 *     7: 地面速度（节）
 *     8: 地面航向
 *     9: 日期
 *
 * 解析策略：
 *   循环查找逗号，用字段编号控制把哪些字段复制到哪里
 */
void GPS_ParseGNRMC_Practice(const char *sentence, GPS_RawData_t *raw)
{
    if (sentence == NULL || raw == NULL) return;
    memset(raw, 0, sizeof(GPS_RawData_t));

    const char *ptr = sentence;
    char valid_flag = 'V';

    for (int field = 0; field <= 6; field++) {
        /* 找到下一个逗号（字段分隔符）*/
        const char *comma = strchr(ptr, ',');
        if (comma == NULL) return;  /* 格式错误 */

        /* 字段内容在 ptr ~ comma-1 */
        const char *field_start = ptr + 1;  /* 跳过前面的逗号（或 $ 语句头）*/
        if (field == 0) field_start = ptr;  /* 第0字段从头开始 */
        int field_len = (int)(comma - field_start);
        if (field_len < 0) field_len = 0;

        switch (field) {
            /* field 0: 语句类型，跳过 */
            case 1:  /* UTC时间 */
                if (field_len > 0 && field_len < (int)sizeof(raw->utc_time))
                    memcpy(raw->utc_time, field_start, field_len);
                break;
            case 2:  /* 有效性 */
                if (field_len > 0) valid_flag = *field_start;
                break;
            case 3:  /* 纬度 */
                if (field_len > 0 && field_len < (int)sizeof(raw->latitude))
                    memcpy(raw->latitude, field_start, field_len);
                break;
            case 4:  /* N/S */
                if (field_len > 0) {
                    raw->ns[0] = *field_start;
                    raw->ns[1] = '\0';
                }
                break;
            case 5:  /* 经度 */
                if (field_len > 0 && field_len < (int)sizeof(raw->longitude))
                    memcpy(raw->longitude, field_start, field_len);
                break;
            case 6:  /* E/W */
                if (field_len > 0) {
                    raw->ew[0] = *field_start;
                    raw->ew[1] = '\0';
                }
                break;
            default: break;
        }
        ptr = comma;  /* 移动到逗号位置，下次循环 +1 跳过 */
    }

    raw->is_valid  = (valid_flag == 'A');
    raw->is_parsed = true;
}

/* ====================================================================
 * Part 2：坐标格式转换 DDMM.MMMM → 十进制度
 * ==================================================================== */

/**
 * @brief  将 GPS 原始坐标字符串转换为十进制度
 * @param  nmea_coord  格式："DDMM.MMMM"（纬度）或 "DDDMM.MMMM"（经度）
 * @retval 十进制度（double）
 *
 * 转换原理：
 *   NMEA 格式：前N位是"度"，其余是"分.分小数"
 *   例：纬度 "3914.1234" = 39度 14.1234分
 *
 *   步骤：
 *     1. 整体转换为浮点数：3914.1234
 *     2. 整除100取度：3914 / 100 = 39（度部分）
 *     3. 除以100后减去度：3914.1234/100 - 39 = 0.141234 → × 100 = 14.1234（分部分）
 *     4. 分 / 60 转换为度小数：14.1234 / 60 = 0.235390
 *     5. 结果 = 39 + 0.235390 = 39.235390°
 *
 * 项目原始代码（gps.c: Convert_to_degrees）：
 *   double temp = atof(data);
 *   int degree = (int)(temp / 100);
 *   double f_degree = (temp / 100.0 - degree) * 100 / 60.0;
 *   return degree + f_degree;
 */
double GPS_ConvertToDegrees_Practice(const char *nmea_coord)
{
    if (nmea_coord == NULL || nmea_coord[0] == '\0') return 0.0;

    double temp_val = atof(nmea_coord);  /* 字符串→浮点 */
    int    degree   = (int)(temp_val / 100);             /* 取整：度 */
    double minutes  = (temp_val / 100.0 - degree) * 100; /* 分 */
    return degree + minutes / 60.0;                       /* 转换为十进制度 */
}

/**
 * @brief  将原始 GPS 数据转换为十进制坐标，考虑南北/东西半球
 * @param  raw   解析后的 GNRMC 字段
 * @param  pos   输出：GPS_Position_t（十进制度）
 */
void GPS_ToPosition_Practice(const GPS_RawData_t *raw, GPS_Position_t *pos)
{
    pos->valid = false;
    if (!raw->is_valid || !raw->is_parsed) return;

    pos->latitude  = GPS_ConvertToDegrees_Practice(raw->latitude);
    pos->longitude = GPS_ConvertToDegrees_Practice(raw->longitude);

    /* 南半球纬度取负 */
    if (raw->ns[0] == 'S') pos->latitude  = -pos->latitude;
    /* 西半球经度取负 */
    if (raw->ew[0] == 'W') pos->longitude = -pos->longitude;

    pos->valid = true;
}

/* ====================================================================
 * Part 3：Haversine 球面距离公式
 * ==================================================================== */

/**
 * @brief  计算地球表面两点间大圆距离（Haversine 公式）
 * @param  lat1, lon1  起点经纬度（十进制度）
 * @param  lat2, lon2  终点经纬度（十进制度）
 * @retval 距离（米）
 *
 * Haversine 公式推导：
 *
 *   设 Δlat = lat2-lat1，Δlon = lon2-lon1（均换算为弧度）
 *
 *   a = sin²(Δlat/2) + cos(lat1)·cos(lat2)·sin²(Δlon/2)
 *
 *   c = 2·atan2(√a, √(1-a))    （圆心角，单位：弧度）
 *
 *   d = R·c                     （距离，R=6371000m）
 *
 * 为什么用 Haversine 而不是欧几里得距离？
 *   欧几里得只适合平面，地球是球面。
 *   对于几公里范围内，误差也会有几米。
 *   Haversine 对球面精确，误差仅来自地球非完美球形（<0.3%）。
 *
 * 数值注意：
 *   a 应在 [0,1]，但浮点误差可能使 sqrt(1-a) 对接近极点的情况失效。
 *   实用中两点距离不超过几十公里时不需要担心这个问题。
 */
float GPS_CalculateDistance_Practice(double lat1, double lon1,
                                     double lat2, double lon2)
{
    /* 转换为弧度 */
    double lat1_r = lat1 * M_PI_PRACTICE / 180.0;
    double lon1_r = lon1 * M_PI_PRACTICE / 180.0;
    double lat2_r = lat2 * M_PI_PRACTICE / 180.0;
    double lon2_r = lon2 * M_PI_PRACTICE / 180.0;

    double d_lat = lat2_r - lat1_r;
    double d_lon = lon2_r - lon1_r;

    /* Haversine 核心计算 */
    double a = sin(d_lat / 2) * sin(d_lat / 2)
             + cos(lat1_r) * cos(lat2_r)
             * sin(d_lon / 2) * sin(d_lon / 2);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return (float)(EARTH_RADIUS * c);
}

/* ====================================================================
 * Part 4：航向角（方位角）计算
 * ==================================================================== */

/**
 * @brief  将方位角归一化到 [0, 360) 度
 */
static float NormalizeBearing(float b)
{
    while (b <    0.0f) b += 360.0f;
    while (b >= 360.0f) b -= 360.0f;
    return b;
}

/**
 * @brief  计算从点1到点2的正北为基准的方位角（顺时针）
 * @param  lat1, lon1  起点（十进制度）
 * @param  lat2, lon2  终点（十进制度）
 * @retval 方位角，范围 [0, 360)，0=正北，90=正东，180=正南，270=正西
 *
 * 球面方位角公式：
 *   y = sin(Δlon) · cos(lat2)
 *   x = cos(lat1) · sin(lat2) − sin(lat1) · cos(lat2) · cos(Δlon)
 *   θ = atan2(y, x)    （弧度，正东=0，逆时针为正）
 *
 * 为什么要交换 x/y 参数？
 *   atan2 的数学定义是以正东为0、逆时针为正。
 *   地理方位角以正北为0、顺时针为正。
 *   这两者的关系是：bearing = 90° - math_angle
 *   在代码里，把 y（东分量）放第一参数、x（北分量）放第二参数，
 *   atan2(y, x) 正好等效于 90° - atan2(x, y)，即转换成地理角度。
 *
 * atan2 返回范围 [-π, +π]，加 360° 处理负数后归一化到 [0, 360)。
 */
float GPS_CalculateBearing_Practice(double lat1, double lon1,
                                    double lat2, double lon2)
{
    double lat1_r = lat1 * M_PI_PRACTICE / 180.0;
    double lon1_r = lon1 * M_PI_PRACTICE / 180.0;
    double lat2_r = lat2 * M_PI_PRACTICE / 180.0;
    double lon2_r = lon2 * M_PI_PRACTICE / 180.0;

    double d_lon = lon2_r - lon1_r;

    /* 球面方位角公式 */
    double y = sin(d_lon) * cos(lat2_r);
    double x = cos(lat1_r) * sin(lat2_r)
             - sin(lat1_r) * cos(lat2_r) * cos(d_lon);

    double bearing_rad = atan2(y, x);
    float  bearing_deg = (float)(bearing_rad * 180.0 / M_PI_PRACTICE);

    return NormalizeBearing(bearing_deg);
}

/**
 * @brief  计算两个方位角间的最短差值（考虑360°连续性）
 * @retval 范围 (-180, +180]，正=顺时针需要转，负=逆时针需要转
 *
 * 与 PID 中的 Angle_Diff 类似，但这里处理的是 [0,360) 范围的角度
 */
float Bearing_Difference_Practice(float bearing1, float bearing2)
{
    float diff = bearing1 - bearing2;
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

/* ====================================================================
 * Part 5：Navigation_Update —— 整合为 PID 提供目标偏航角
 * ==================================================================== */

/**
 * @brief  初始化导航状态
 */
void Nav_Init_Practice(NavState_t *nav)
{
    memset(nav, 0, sizeof(NavState_t));
    nav->arrival_threshold = 5.0f;  /* 5米到达阈值 */
}

/**
 * @brief  设置导航目标
 */
void Nav_SetTarget_Practice(NavState_t *nav, double target_lat, double target_lon)
{
    /* 简单有效性检查 */
    if (target_lat < -90.0 || target_lat > 90.0)  return;
    if (target_lon < -180.0 || target_lon > 180.0) return;
    if (target_lat == 0.0 && target_lon == 0.0)    return;

    nav->target.latitude  = target_lat;
    nav->target.longitude = target_lon;
    nav->target.valid     = true;
    nav->target_reached   = false;
}

/**
 * @brief  导航更新：根据当前 GPS 坐标计算目标偏航角
 * @param  nav          导航状态
 * @param  current_lat  当前纬度（十进制度）
 * @param  current_lon  当前经度（十进制度）
 * @param  current_yaw  当前 IMU 偏航角（-180~+180，用于到达时保持）
 * @retval 目标偏航角（度），供 PID yaw_pid.setpoint 使用
 *
 * 流程：
 *   1. 计算距离 → 若 ≤ arrival_threshold，停止导航，返回当前偏航
 *   2. 计算方位角（正北=0，顺时针）→ [0,360)
 *   3. 方位角就是期望的偏航角，传给 PID 控制器
 *
 * 坐标系统注意：
 *   IMU 偏航角（Yaw）范围 [-180°,+180°]，以正北=0，顺时针为正
 *   GPS 方位角（Bearing）范围 [0°,360°]，以正北=0，顺时针为正
 *   两者方向定义相同，但范围不同。
 *   直接用 Bearing_Difference 将方位角转换为 PID 误差即可。
 */
float Nav_Update_Practice(NavState_t *nav,
                          double current_lat, double current_lon,
                          float current_yaw)
{
    if (!nav->nav_active || !nav->target.valid) {
        return current_yaw;  /* 导航未激活，维持当前航向 */
    }

    /* 更新当前位置 */
    nav->current.latitude  = current_lat;
    nav->current.longitude = current_lon;
    nav->current.valid     = true;

    /* 步骤1：计算距目标距离 */
    nav->distance_to_target = GPS_CalculateDistance_Practice(
        current_lat, current_lon,
        nav->target.latitude, nav->target.longitude
    );

    /* 步骤2：到达判断（距离 ≤ 阈值）*/
    if (nav->distance_to_target <= nav->arrival_threshold) {
        nav->target_reached = true;
        nav->nav_active     = false;
        return current_yaw;  /* 到达，维持当前航向，停止 */
    }

    /* 步骤3：计算方位角（[0,360)） */
    nav->target_bearing = GPS_CalculateBearing_Practice(
        current_lat, current_lon,
        nav->target.latitude, nav->target.longitude
    );

    /* 返回目标方位角（PID 将以此为 setpoint）*/
    return nav->target_bearing;
}

/* ====================================================================
 * 测试演示
 * ==================================================================== */

/**
 * @brief  NMEA 解析测试
 *
 * 真实 GNRMC 语句例：
 *   $GNRMC,092750.000,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A*43
 *
 * 字段解释：
 *   UTC时间：09:27:50.000
 *   有效性：A（有效）
 *   纬度：5321.6802 N = 53°21.6802' N = 53.361337° N
 *   经度：0630.3372 W = 6°30.3372' W = 6.505620° W（负值）
 */
void GPS_ParseTest(void)
{
    const char *test_sentence =
        "$GNRMC,092750.000,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A*43";

    GPS_RawData_t  raw;
    GPS_Position_t pos;

    GPS_ParseGNRMC_Practice(test_sentence, &raw);
    GPS_ToPosition_Practice(&raw, &pos);

    printf("=== GNRMC 解析测试 ===\r\n");
    printf("UTC时间: %s\r\n",      raw.utc_time);
    printf("有效性:  %s\r\n",      raw.is_valid ? "有效(A)" : "无效(V)");
    printf("纬度原始: %s %s\r\n",  raw.latitude, raw.ns);
    printf("经度原始: %s %s\r\n",  raw.longitude, raw.ew);
    printf("纬度十进制: %.6f°\r\n", pos.latitude);
    printf("经度十进制: %.6f°\r\n", pos.longitude);
    printf("（期望：lat=53.361337, lon=-6.505620）\r\n\r\n");
}

/**
 * @brief  距离和方位角计算测试
 *
 * 测试用例：北京（39.9°N，116.4°E）→ 上海（31.2°N，121.5°E）
 *   实际距离约 1068 km
 *   方位角：从北京看上海大约朝东南方向（约 135°）
 */
void GPS_NavigationTest(void)
{
    double beijing_lat = 39.9042,  beijing_lon = 116.4074;
    double shanghai_lat = 31.2304, shanghai_lon = 121.4737;

    float dist    = GPS_CalculateDistance_Practice(beijing_lat, beijing_lon,
                                                   shanghai_lat, shanghai_lon);
    float bearing = GPS_CalculateBearing_Practice (beijing_lat, beijing_lon,
                                                   shanghai_lat, shanghai_lon);

    printf("=== 距离和方位角测试 ===\r\n");
    printf("北京 → 上海\r\n");
    printf("  距离: %.1f 米 (%.1f km)\r\n", dist, dist / 1000.0f);
    printf("  方位: %.1f° (期望约 135° 东南方向)\r\n\r\n", bearing);
}

/**
 * @brief  完整导航演示（模拟 GPS 数据驱动导航）
 *
 * 场景：船从某个位置出发，朝目标位置导航
 *       模拟 GPS 坐标逐步靠近目标
 */
void GPS_NavSimulation(void)
{
    NavState_t nav;
    Nav_Init_Practice(&nav);

    /* 目标坐标（模拟）*/
    double target_lat = 30.5000, target_lon = 114.3000;
    Nav_SetTarget_Practice(&nav, target_lat, target_lon);
    nav.nav_active = true;

    /* 当前坐标（从稍远处出发）*/
    double cur_lat = 30.4900, cur_lon = 114.2900;
    float  cur_yaw = 0.0f;  /* 当前朝正北 */

    printf("=== 导航仿真演示 ===\r\n");
    printf("目标: %.4f°N, %.4f°E\r\n", target_lat, target_lon);
    printf("出发: %.4f°N, %.4f°E\r\n\r\n", cur_lat, cur_lon);
    printf("%-5s %-12s %-10s %-10s %-10s\r\n",
           "步骤", "距离(m)", "方位角(°)", "目标Yaw(°)", "状态");

    for (int step = 0; step < 20; step++) {
        float target_yaw = Nav_Update_Practice(&nav, cur_lat, cur_lon, cur_yaw);

        printf("%-5d %-12.1f %-10.1f %-10.1f %s\r\n",
               step,
               nav.distance_to_target,
               nav.target_bearing,
               target_yaw,
               nav.target_reached ? "到达!" : "导航中");

        if (nav.target_reached) break;

        /* 模拟船沿目标方向移动（每步约10米）*/
        double bearing_rad = target_yaw * M_PI_PRACTICE / 180.0;
        cur_lat += cos(bearing_rad) * (10.0 / EARTH_RADIUS) * (180.0 / M_PI_PRACTICE);
        cur_lon += sin(bearing_rad) * (10.0 / EARTH_RADIUS) * (180.0 / M_PI_PRACTICE)
                 / cos(cur_lat * M_PI_PRACTICE / 180.0);
        cur_yaw = target_yaw;
    }
    printf("\r\n");
}

/* 主测试函数 */
void GPS_Nav_Test(void)
{
    GPS_ParseTest();
    GPS_NavigationTest();
    GPS_NavSimulation();
}

/* ====================================================================
 * 思考题：
 *
 * Q1：GNRMC 中纬度 "3914.1234" 转换为十进制度是多少？
 *     A：度部分 = 3914 / 100 = 39
 *        分部分 = (3914.1234 / 100.0 - 39) × 100 = 14.1234 分
 *        十进制 = 39 + 14.1234 / 60 = 39.235390°
 *
 * Q2：为什么 Haversine 公式要先转换弧度？
 *     A：C 数学库的 sin/cos/atan2 函数接受弧度参数。
 *        角度 × π/180 = 弧度（例：90° = π/2 ≈ 1.5708rad）。
 *        如果直接传度数，结果会完全错误。
 *
 * Q3：方位角和 IMU 偏航角的坐标系相同吗？
 *     A：方向定义相同（正北=0，顺时针为正），但范围不同：
 *        IMU 偏航角范围 [-180°, +180°]
 *        GPS 方位角范围 [0°, 360°)
 *        在 PID 误差计算时，用 Bearing_Difference 统一处理。
 *        例：方位角=350°，IMU偏航=-20°（等同于340°），差值=10°（应顺时针转10°）。
 *
 * Q4：到达阈值设为 5 米是否合理？
 *     A：取决于 GPS 精度。民用 GPS 精度约 3~10 米（HDOP良好时~3米）。
 *        阈值 5m 时，有可能 GPS 误差本身就让"到达"判断抖动。
 *        实际项目中考虑：
 *          - 阈值稍大（如 10m），确保真正停在目标附近而不是一直进/出判定范围
 *          - 结合速度：距离 < 10m 且速度很小时才算到达
 *          - 到达后保持几秒确认，避免短暂 GPS 误差导致误判
 * ==================================================================== */
