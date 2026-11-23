/**
 * @file laser.h
 * @brief 激光模块通信协议头文件
 */

#ifndef __LASER_H__
#define __LASER_H__

#include "sys.h"
#include "uart1.h"
#include "uart2.h"
#include "string.h"
#include "stdio.h"

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

// 外部变量声明
extern uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];
extern uint16_t uart1_rx_len;
extern uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE];
extern uint16_t uart2_rx_len;
extern uint8_t uart2_idle_flag;
extern uint16_t uart2_frame_len;
extern UART_HandleTypeDef uart1_handle;
extern UART_HandleTypeDef uart2_handle;

extern uint8_t laser_measurement_active; // 声明激光测量状态标志

// 函数声明
void Laser_Init(uint32_t baudrate);
void Laser_SendCommand(uint8_t command);
void Laser_ProcessCommand(uint8_t *data, uint16_t len);
void Laser_ProcessStringCommand(const char *cmd_str);
void Laser_ProcessResponse(uint8_t *data, uint16_t len);
uint16_t Laser_ParseDistance(uint8_t *data, uint16_t len);
void Laser_HandleMeasurement(void); // 声明新的处理函数

#endif 
