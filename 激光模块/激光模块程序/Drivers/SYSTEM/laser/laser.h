/**
 * @file laser.h
 * @brief 激光模块通信协议头文件
 */

#ifndef __LASER_H
#define __LASER_H

#include "sys.h"

// 激光模块指令定义
#define LASER_START_MEASURE    0x01    // 开始测量
#define LASER_STOP_MEASURE     0x02    // 停止测量
#define LASER_QUERY_BAUDRATE   0x11    // 查询波特率

// 上位机指令字符串定义
#define CMD_BEGIN              "begin"  // 开始测量
#define CMD_END               "end"    // 结束测量
#define CMD_QUERY_BAUDRATE    "baud"   // 查询波特率

// 激光模块指令帧结构
typedef struct {
    uint8_t header;        // 帧头 0xA5
    uint8_t length;        // 数据长度 0x03
    uint8_t command;       // 命令字
    uint8_t data[4];       // 数据域
    uint16_t checksum;     // 校验和
} Laser_Frame_t;

// 上位机指令映射结构
typedef struct {
    const char *cmd_str;           // 上位机指令字符串
    const uint8_t *frame;          // 对应的激光模块指令帧
    uint8_t frame_len;             // 指令帧长度
} Laser_Cmd_Map_t;

// 函数声明
void Laser_Init(void);
void Laser_SendCommand(uint8_t command);
void Laser_ProcessCommand(uint8_t *data, uint16_t len);
void Laser_ProcessStringCommand(const char *cmd_str);

#endif 
