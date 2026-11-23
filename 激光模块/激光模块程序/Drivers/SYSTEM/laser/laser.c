/**
 * @file laser.c
 * @brief 激光模块通信协议实现文件
 */

#include "laser.h"
#include "uart1.h"
#include "string.h"
#include "stdio.h"

extern UART_HandleTypeDef uart1_handle;

// 激光模块指令帧
static const uint8_t LASER_FRAME_HEADER = 0xA5;
static const uint8_t LASER_FRAME_LENGTH = 0x03;

// 预定义的指令帧
static const uint8_t LASER_START_MEASURE_FRAME[] = {0xA5, 0x03, 0x20, 0x01, 0x00, 0x00, 0x00, 0x6E, 0x02};
static const uint8_t LASER_STOP_MEASURE_FRAME[] = {0xA5, 0x03, 0x20, 0x02, 0x00, 0x00, 0x00, 0x6E, 0x46};
static const uint8_t LASER_QUERY_BAUDRATE_FRAME[] = {0xA5, 0x03, 0x20, 0x11, 0x00, 0x00, 0x00, 0x6A, 0xC2};

// 上位机指令映射表
static const Laser_Cmd_Map_t cmd_map[] = {
    {CMD_BEGIN, LASER_START_MEASURE_FRAME, sizeof(LASER_START_MEASURE_FRAME)},
    {CMD_END, LASER_STOP_MEASURE_FRAME, sizeof(LASER_STOP_MEASURE_FRAME)},
    {CMD_QUERY_BAUDRATE, LASER_QUERY_BAUDRATE_FRAME, sizeof(LASER_QUERY_BAUDRATE_FRAME)},
    {NULL, NULL, 0}  // 结束标记
};

/**
 * @brief 初始化激光模块
 */
void Laser_Init(void)
{
    // 初始化UART1，波特率根据激光模块实际要求设置
    uart1_init(115200);
}

/**
 * @brief 发送指令到激光模块
 * @param command 要发送的命令
 */
void Laser_SendCommand(uint8_t command)
{
    const uint8_t *frame = NULL;
    uint8_t frame_len = 0;
    switch(command)
    {
        case LASER_START_MEASURE:
            frame = LASER_START_MEASURE_FRAME;
            frame_len = sizeof(LASER_START_MEASURE_FRAME);
            break;
        case LASER_STOP_MEASURE:
            frame = LASER_STOP_MEASURE_FRAME;
            frame_len = sizeof(LASER_STOP_MEASURE_FRAME);
            break;
        case LASER_QUERY_BAUDRATE:
            frame = LASER_QUERY_BAUDRATE_FRAME;
            frame_len = sizeof(LASER_QUERY_BAUDRATE_FRAME);
            break;
        default:
            return;
    }
    HAL_UART_Transmit(&uart1_handle, (uint8_t*)frame, frame_len, 1000);
}

/**
 * @brief 处理字符串命令
 * @param cmd_str 命令字符串
 */
void Laser_ProcessStringCommand(const char *cmd_str)
{
    if (cmd_str == NULL) return;
    for (const Laser_Cmd_Map_t *map = cmd_map; map->cmd_str != NULL; map++)
    {
        if (strcmp(cmd_str, map->cmd_str) == 0)
        {
            HAL_UART_Transmit(&uart1_handle, (uint8_t*)map->frame, map->frame_len, 1000);
            return;
        }
    }
}

/**
 * @brief 处理来自上位机的命令
 * @param data 接收到的数据
 * @param len 数据长度
 */
void Laser_ProcessCommand(uint8_t *data, uint16_t len)
{
    if(len < 3 || data[0] != LASER_FRAME_HEADER || data[1] != LASER_FRAME_LENGTH)
    {
        data[len] = '\0';  // 确保字符串结束
        Laser_ProcessStringCommand((char*)data);
        return;
    }
    if(data[2] == 0x20)
    {
        Laser_SendCommand(data[3]);
    }
} 
