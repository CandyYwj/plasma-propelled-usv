/**
 * @file    state_machine_practice.c
 * @brief   模块八练习：主控状态机 + 上位机串口协议
 *
 * 学习目标：
 *   1. 理解三模式状态机（MANUAL/PID/GPS）及其切换逻辑
 *   2. 掌握基于字符串的命令解析（strncmp + atof/sscanf）
 *   3. 理解数据报告帧的格式构建（sprintf 拼接 + 帧尾标记）
 *   4. 掌握 200ms 测量周期与 20ms PID 控制周期的分频调度
 *   5. 理解为什么手动命令要立即停 PID/GPS（模式互斥）
 *
 * 对应项目文件：Users/main.c
 *
 * 验证方法：
 *   1. 发送 "begin" → 应收到周期性数据报告（TDS/TMP/distance/...）
 *   2. 发送 "advance" → 电机前进，同时报告停止（begin未发）
 *   3. 发送 "pid_on" → "target_yaw:45" → 观察电机差速转向 45°
 *   4. 发送 "nav_target:30.5,114.3" → "nav_start" → 观察航向追踪
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

/* ====================================================================
 * 控制模式枚举（与 main.c 一致）
 * ==================================================================== */

/**
 * 三种 PWM 控制模式：
 *
 *   MANUAL（手动）：
 *     响应 advance/left/right/stop 命令
 *     PWM 立即生效，无渐变
 *     PID 和 GPS 均停止
 *
 *   PID（姿态闭环）：
 *     响应 pid_on / target_yaw / pid_gains 命令
 *     每 20ms 运行一次 PID 计算
 *     PWM 有渐变保护（单次最大变化 30%）
 *
 *   GPS（自主导航）：
 *     响应 nav_target / nav_start / nav_stop 命令
 *     每 20ms 更新导航计算
 *     优先级高于 PID（GPS 激活时 PID 被覆盖）
 */
typedef enum {
    PWM_MODE_MANUAL = 0,
    PWM_MODE_PID    = 1,
    PWM_MODE_GPS    = 2
} PWM_Mode_t;

/* ====================================================================
 * 仿真用占位类型（实际项目由各模块头文件提供）
 * ==================================================================== */
typedef struct {
    float left_motor_pwm;
    float right_motor_pwm;
} MotorOutput_t;

typedef struct {
    float kp, ki, kd;
    float setpoint;
    float integral;
    float last_error;
    uint32_t last_time;
} PID_Simple_t;

typedef struct {
    float target_yaw;
    bool  yaw_enable;
    PID_Simple_t yaw_pid;
} AttCtrl_Simple_t;

typedef struct {
    double target_lat, target_lon;
    bool   target_valid;
    bool   nav_active;
    bool   target_reached;
    float  distance_to_target;
    float  target_bearing;
    float  arrival_threshold;
} NavState_Simple_t;

/* ====================================================================
 * 全局状态变量
 * ==================================================================== */

static PWM_Mode_t      g_pwm_mode          = PWM_MODE_MANUAL;
static uint8_t         g_measuring         = 0;    /* begin/end 标志 */
static uint8_t         g_pid_enable        = 0;
static uint8_t         g_nav_enable        = 0;
static AttCtrl_Simple_t g_att_ctrl         = {0};
static NavState_Simple_t g_nav_state       = {0};
static uint32_t        g_last_ctrl_time    = 0;    /* 上次控制计算时间 */

/* 简化的 PWM 状态（渐变保护用）*/
static float g_left_pwm  = 0.0f;
static float g_right_pwm = 0.0f;

/* ====================================================================
 * 工具函数
 * ==================================================================== */

static void PWM_Set(float left, float right)
{
    if (left  < 0.0f) left  = 0.0f;
    if (left  > 100.0f) left  = 100.0f;
    if (right < 0.0f) right = 0.0f;
    if (right > 100.0f) right = 100.0f;

    if (g_pwm_mode == PWM_MODE_MANUAL) {
        /* 手动模式：立即生效 */
        g_left_pwm  = left;
        g_right_pwm = right;
    } else {
        /* PID/GPS 模式：渐变限制（单次 ≤ 30%）*/
        float dl = left  - g_left_pwm;
        float dr = right - g_right_pwm;
        if (fabsf(dl) > 30.0f) left  = g_left_pwm  + (dl > 0 ? 30.0f : -30.0f);
        if (fabsf(dr) > 30.0f) right = g_right_pwm + (dr > 0 ? 30.0f : -30.0f);
        g_left_pwm  = left;
        g_right_pwm = right;
    }

    /* 实际项目：TIM1_PWM_SetDuty(PWM_CHANNEL_1, g_left_pwm); 等 */
    printf("[PWM] L=%.1f%%  R=%.1f%%\r\n", g_left_pwm, g_right_pwm);
}

/* ====================================================================
 * 命令解析（核心：根据命令字符串分发到对应处理函数）
 * ==================================================================== */

/**
 * @brief  解析并执行一条上位机命令
 * @param  cmd  命令字符串（不含 \r\n）
 * @param  len  长度
 *
 * 支持的命令表：
 *
 *  命令              | 类型   | 格式              | 功能
 * -------------------|--------|-------------------|---------------------------
 *  begin             | 固定   | "begin"           | 开始连续测量上报
 *  end               | 固定   | "end"             | 停止测量上报
 *  advance           | 手动   | "advance"         | 前进（左右各70%）
 *  left              | 手动   | "left"            | 左转（左70%，右0%）
 *  right             | 手动   | "right"           | 右转（左0%，右70%）
 *  stop              | 手动   | "stop"            | 停止（双路0%）
 *  pid_on            | PID    | "pid_on"          | 启用偏航角 PID 控制
 *  pid_off           | PID    | "pid_off"         | 关闭 PID，停止电机
 *  target_yaw:N      | PID    | "target_yaw:45.0" | 设置目标偏航角
 *  pid_gains:P:I:D   | PID    | "pid_gains:2:0.1:0.5" | 动态调整 PID 参数
 *  nav_target:lat,lon| GPS    | "nav_target:30.5,114.3" | 设置导航目标坐标
 *  nav_start         | GPS    | "nav_start"       | 开始 GPS 自主导航
 *  nav_stop          | GPS    | "nav_stop"        | 停止 GPS 导航，电机停
 *  nav_status        | GPS    | "nav_status"      | 查询当前导航状态
 *
 * 命令解析策略：
 *   1. 固定长度命令（begin/end/stop等）：用 strncmp 完整匹配
 *   2. 带参数命令（target_yaw:/pid_gains:/nav_target:）：
 *      用 strncmp 匹配前缀，再用 atof/sscanf 解析后面的参数
 *
 * 为什么不用 strcmp？
 *   串口可能带 '\r' 或 '\n'，用 strncmp + strlen(prefix) 只比较前N字符，
 *   不受末尾换行符影响。
 */
void Command_Parse_Practice(const char *cmd, uint16_t len)
{
    char resp[128];

    /* ---- 测量控制 ---- */
    if (len == strlen("begin") && strncmp(cmd, "begin", 5) == 0) {
        g_measuring = 1;
        printf("[CMD] 开始连续测量\r\n");
    }
    else if (len == strlen("end") && strncmp(cmd, "end", 3) == 0) {
        g_measuring = 0;
        printf("[CMD] 停止测量\r\n");
    }

    /* ---- 手动运动控制（模式互斥：立即清除 PID/GPS）---- */
    else if (len == strlen("advance") && strncmp(cmd, "advance", 7) == 0) {
        g_pwm_mode  = PWM_MODE_MANUAL;
        g_pid_enable = 0;
        g_nav_enable = 0;
        PWM_Set(70.0f, 70.0f);
    }
    else if (len == strlen("left") && strncmp(cmd, "left", 4) == 0) {
        g_pwm_mode  = PWM_MODE_MANUAL;
        g_pid_enable = 0;
        g_nav_enable = 0;
        PWM_Set(70.0f, 0.0f);
    }
    else if (len == strlen("right") && strncmp(cmd, "right", 5) == 0) {
        g_pwm_mode  = PWM_MODE_MANUAL;
        g_pid_enable = 0;
        g_nav_enable = 0;
        PWM_Set(0.0f, 70.0f);
    }
    else if (len == strlen("stop") && strncmp(cmd, "stop", 4) == 0) {
        g_pwm_mode  = PWM_MODE_MANUAL;
        g_pid_enable = 0;
        g_nav_enable = 0;
        PWM_Set(0.0f, 0.0f);
    }

    /* ---- PID 控制 ---- */
    else if (len == strlen("pid_on") && strncmp(cmd, "pid_on", 6) == 0) {
        g_pwm_mode   = PWM_MODE_PID;
        g_pid_enable = 1;
        g_nav_enable = 0;
        g_att_ctrl.yaw_enable = true;
        printf("[CMD] PID 控制启用，目标偏航=%.1f°\r\n", g_att_ctrl.target_yaw);
    }
    else if (len == strlen("pid_off") && strncmp(cmd, "pid_off", 7) == 0) {
        g_pwm_mode   = PWM_MODE_MANUAL;
        g_pid_enable = 0;
        g_att_ctrl.yaw_enable = false;
        PWM_Set(0.0f, 0.0f);
        printf("[CMD] PID 控制关闭\r\n");
    }
    else if (strncmp(cmd, "target_yaw:", strlen("target_yaw:")) == 0) {
        float target_yaw = atof(cmd + strlen("target_yaw:"));
        g_att_ctrl.target_yaw = target_yaw;
        /* 同时设置 PID setpoint */
        g_att_ctrl.yaw_pid.setpoint = target_yaw;
        snprintf(resp, sizeof(resp), "[CMD] 目标偏航角 = %.1f°\r\n", target_yaw);
        printf("%s", resp);
    }
    else if (strncmp(cmd, "pid_gains:", strlen("pid_gains:")) == 0) {
        float kp, ki, kd;
        if (sscanf(cmd + strlen("pid_gains:"), "%f:%f:%f", &kp, &ki, &kd) == 3) {
            g_att_ctrl.yaw_pid.kp = kp;
            g_att_ctrl.yaw_pid.ki = ki;
            g_att_ctrl.yaw_pid.kd = kd;
            snprintf(resp, sizeof(resp),
                     "[CMD] PID 参数 Kp=%.2f Ki=%.2f Kd=%.2f\r\n", kp, ki, kd);
            printf("%s", resp);
        } else {
            printf("[CMD] 格式错误，示例：pid_gains:2.0:0.1:0.5\r\n");
        }
    }

    /* ---- GPS 导航 ---- */
    else if (strncmp(cmd, "nav_target:", strlen("nav_target:")) == 0) {
        double lat, lon;
        if (sscanf(cmd + strlen("nav_target:"), "%lf,%lf", &lat, &lon) == 2) {
            g_nav_state.target_lat   = lat;
            g_nav_state.target_lon   = lon;
            g_nav_state.target_valid = true;
            g_nav_state.target_reached = false;
            snprintf(resp, sizeof(resp),
                     "[CMD] 导航目标 Lat=%.6f Lon=%.6f\r\n", lat, lon);
            printf("%s", resp);
        } else {
            printf("[CMD] 格式错误，示例：nav_target:30.5,114.3\r\n");
        }
    }
    else if (len == strlen("nav_start") && strncmp(cmd, "nav_start", 9) == 0) {
        if (g_nav_state.target_valid) {
            g_pwm_mode   = PWM_MODE_GPS;
            g_nav_enable = 1;
            g_pid_enable = 0;
            g_nav_state.nav_active = true;
            printf("[CMD] GPS 导航启动\r\n");
        } else {
            printf("[CMD] 错误：未设置导航目标（请先发 nav_target:lat,lon）\r\n");
        }
    }
    else if (len == strlen("nav_stop") && strncmp(cmd, "nav_stop", 8) == 0) {
        g_pwm_mode   = PWM_MODE_MANUAL;
        g_nav_enable = 0;
        g_nav_state.nav_active = false;
        PWM_Set(0.0f, 0.0f);
        printf("[CMD] GPS 导航停止\r\n");
    }
    else if (len == strlen("nav_status") && strncmp(cmd, "nav_status", 10) == 0) {
        snprintf(resp, sizeof(resp),
                 "Nav Status: Active=%d Reached=%d Dist=%.1fm Bear=%.1f°\r\n",
                 g_nav_state.nav_active,
                 g_nav_state.target_reached,
                 g_nav_state.distance_to_target,
                 g_nav_state.target_bearing);
        printf("%s", resp);
    }

    /* ---- 未知命令 ---- */
    else {
        printf("[CMD] 未知命令: %.*s\r\n", (int)len, cmd);
    }
}

/* ====================================================================
 * 数据报告帧构建
 * ==================================================================== */

/**
 * @brief  构建传感器数据报告字符串
 * @param  buf   输出缓冲区
 * @param  size  缓冲区大小
 *
 * 报告格式（每行一个键值对，最后以 "end\r\n" 结尾）：
 *
 *   TDS=125.3\r\n
 *   TMP=24.5\r\n
 *   distance=1234\r\n
 *   laser=1230\r\n
 *   IMU_ANGLE:1.2,0.3,87.5\r\n
 *   GPS_LAT=30.500123N\r\n
 *   GPS_LON=114.300456E\r\n
 *   PID_STATUS:ON,45.0\r\n   （仅 PID 模式时）
 *   NAV_STATUS:ON,150.0m,135.0°\r\n  （仅 GPS 模式时）
 *   end\r\n
 *
 * 设计要点：
 *   1. 用 sprintf(...+ strlen(buf)) 逐段追加，不会覆盖之前的内容
 *   2. 无效值（如 -999.0 或 0xFFFF）不追加，发送错误提示
 *   3. "end\r\n" 作为帧尾，上位机用它判断一帧完整
 *
 * 常见问题：
 *   报告字符串超过 buf 大小 → 缓冲区溢出（项目用 128 字节，实测约需 200+ 字节）
 *   解决：增大 buf 或分多次发送
 */
void Build_DataReport_Practice(char *buf, uint16_t size,
                                float tds, float temp,
                                uint16_t ultrasonic, uint16_t laser,
                                float yaw, float pitch, float roll)
{
    buf[0] = '\0';

    /* TDS */
    if (tds > -990.0f) {
        snprintf(buf + strlen(buf), size - strlen(buf),
                 "TDS=%.1f\r\n", tds);
    }

    /* 温度 */
    if (temp > -200.0f) {
        snprintf(buf + strlen(buf), size - strlen(buf),
                 "TMP=%.1f\r\n", temp);
    }

    /* 超声波距离 */
    if (ultrasonic != 0xFFFF) {
        snprintf(buf + strlen(buf), size - strlen(buf),
                 "distance=%u\r\n", ultrasonic);
    }

    /* 激光距离 */
    if (laser != 0xFFFF) {
        snprintf(buf + strlen(buf), size - strlen(buf),
                 "laser=%u\r\n", laser);
    }

    /* IMU 姿态角 */
    snprintf(buf + strlen(buf), size - strlen(buf),
             "IMU_ANGLE:%.3f,%.3f,%.3f\r\n", roll, pitch, yaw);

    /* PID 状态（仅 PID 模式）*/
    if (g_pid_enable) {
        snprintf(buf + strlen(buf), size - strlen(buf),
                 "PID_STATUS:ON,%.1f\r\n", g_att_ctrl.target_yaw);
    }

    /* GPS 导航状态（仅 GPS 模式）*/
    if (g_nav_enable) {
        snprintf(buf + strlen(buf), size - strlen(buf),
                 "NAV_STATUS:ON,%.1fm,%.1f°\r\n",
                 g_nav_state.distance_to_target,
                 g_nav_state.target_bearing);
    }

    /* 帧尾标记 */
    snprintf(buf + strlen(buf), size - strlen(buf), "end\r\n");
}

/* ====================================================================
 * 主循环结构（简化版，展示调度逻辑）
 * ==================================================================== */

/**
 * @brief  主循环调度演示
 *
 * 实际 main() 中的主循环有两个核心周期：
 *
 * ① 200ms 测量周期（由 is_measuring_continuously 控制）：
 *   - 读取所有传感器
 *   - 构建报告字符串
 *   - 通过 UART8 发送给上位机
 *   - HAL_Delay(200) 控制频率
 *
 * ② 20ms 控制周期（50Hz，由 HAL_GetTick() 差值控制）：
 *   - 读取 IMU 数据
 *   - 运行 PID 或 GPS 导航计算
 *   - 输出电机 PWM
 *
 * 两个周期互不干扰：
 *   测量周期慢（200ms），PID 周期快（20ms）
 *   PID 控制用 HAL_GetTick() 非阻塞判断，不被 HAL_Delay 影响
 */
void MainLoop_Skeleton(void)
{
    /* 模拟传感器数据 */
    float tds = 125.3f, temp = 24.5f;
    uint16_t ultrasonic = 1234, laser = 1230;
    float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;

    char report_buf[256];

    /* 模拟主循环运行 10 帧 */
    for (int tick = 0; tick < 200; tick++) {
        uint32_t now = tick * 10;  /* 模拟 HAL_GetTick()，每帧 10ms */

        /* ① 命令检查（模拟从 UART8 收到的命令）*/
        if (tick == 10) {
            Command_Parse_Practice("begin", 5);
        }
        if (tick == 50) {
            Command_Parse_Practice("advance", 7);
        }
        if (tick == 80) {
            Command_Parse_Practice("pid_on", 6);
        }
        if (tick == 90) {
            Command_Parse_Practice("target_yaw:90.0", 15);
        }
        if (tick == 150) {
            Command_Parse_Practice("pid_off", 7);
        }

        /* ② 200ms 测量上报周期 */
        if (g_measuring && (tick % 20 == 0)) {
            Build_DataReport_Practice(report_buf, sizeof(report_buf),
                                      tds, temp, ultrasonic, laser,
                                      yaw, pitch, roll);
            printf("--- 数据报告（t=%dms）---\r\n%s\r\n", (int)now, report_buf);
        }

        /* ③ 20ms PID/GPS 控制周期 */
        if ((g_pid_enable || g_nav_enable) && (now - g_last_ctrl_time >= 20)) {
            g_last_ctrl_time = now;

            if (g_pid_enable && g_pwm_mode == PWM_MODE_PID) {
                /* 实际：调用 Attitude_Controller_Update()，这里仅演示 */
                float yaw_error = g_att_ctrl.target_yaw - yaw;
                /* 简化 PID：P 项 */
                float turn = 2.0f * yaw_error;
                if (turn >  50.0f) turn =  50.0f;
                if (turn < -50.0f) turn = -50.0f;
                float base = 20.0f;
                PWM_Set(base + turn, base - turn);
                /* 模拟 IMU 响应 */
                yaw += turn * 0.1f;
            }
        }
    }
}

/* ====================================================================
 * 命令解析测试
 * ==================================================================== */
void Command_Test(void)
{
    printf("=== 命令解析测试 ===\r\n\r\n");

    const char *cmds[] = {
        "begin", "advance", "left", "right", "stop",
        "pid_on", "target_yaw:45.0", "pid_gains:3.0:0.2:0.8",
        "nav_target:30.5000,114.3000", "nav_start", "nav_status", "nav_stop",
        "end", "unknown_cmd"
    };

    for (int i = 0; i < (int)(sizeof(cmds)/sizeof(cmds[0])); i++) {
        printf("[发送] %s\r\n", cmds[i]);
        Command_Parse_Practice(cmds[i], (uint16_t)strlen(cmds[i]));
        printf("\r\n");
    }
}

/* 主测试入口 */
void StateMachine_Test(void)
{
    printf("\r\n=== 主控状态机测试 ===\r\n\r\n");
    Command_Test();

    printf("\r\n=== 主循环调度演示 ===\r\n\r\n");
    MainLoop_Skeleton();
}

/* ====================================================================
 * 思考题：
 *
 * Q1：为什么手动运动命令（advance/left/right/stop）要立即把 pid_enable
 *     和 nav_enable 清零？
 *     A：三种模式是互斥的，同时运行会产生冲突。
 *        例：PID 正在纠偏到 90°，此时发 "left" 命令，
 *        如果 PID 仍在运行，它会抵消手动操作，导致命令无效。
 *        清零后 PID 计算循环的 if(pid_enable) 判断跳过，
 *        手动命令就能完全控制电机。
 *
 * Q2：为什么 PID 控制用 HAL_GetTick() 非阻塞而不用 HAL_Delay()？
 *     A：HAL_Delay() 是阻塞的，20ms 内 CPU 无法做任何事。
 *        如果用 HAL_Delay(20)，那 200ms 测量周期和命令检查都会被延迟。
 *        用 HAL_GetTick() 差值判断是一种"协作式调度"：
 *        检查距上次执行是否过了 20ms，是则执行，否则跳过继续循环。
 *        CPU 始终在运行，各任务按时间片轮流执行。
 *
 * Q3：数据报告帧为什么以 "end\r\n" 结尾？
 *     A：传感器数量不固定（有些可能失败不输出），所以报告长度不确定。
 *        上位机无法靠字节数判断一帧是否完整。
 *        "end\r\n" 是帧尾标记，上位机收到它就知道这一帧数据已全部到达，
 *        可以开始解析。这是一种简单的协议分帧机制。
 *
 * Q4：GPS 模式和 PID 模式共存时，谁的优先级高？
 *     A：项目设计为 GPS 优先（见 main.c 的控制循环）：
 *        if (gps_navigation_enable) { ... }          ← 先处理 GPS
 *        else if (pid_control_enable) { ... }         ← GPS 未激活才用 PID
 *        启动 nav_start 时代码写了 pid_control_enable = 0，
 *        确保 GPS 模式下不会有 PID 干扰。
 *        实际应用中，GPS 导航需要 PID 来保持航向，
 *        所以正确做法是：GPS 计算目标航向 → 传给 PID 的 setpoint → PID 驱动电机。
 * ==================================================================== */
