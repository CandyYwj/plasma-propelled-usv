/**
 * @file laser.c
 * @brief 激光模块通信协议实现文件
 */

#include "laser.h"
#include "stdio.h"

extern UART_HandleTypeDef uart1_handle;
extern UART_HandleTypeDef uart2_handle;

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

uint8_t laser_measurement_active = 0; // 定义激光测量状态标志并初始化

/**
 * @brief 初始化激光模块
 * @param baudrate 串口波特率
 */
void Laser_Init(uint32_t baudrate)
{
    // 初始化UART1用于与上位机通信
    uart1_init(baudrate);
    // 初始化UART2用于与激光模块通信
    uart2_init(baudrate);
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
    // 通过UART2发送到激光模块
    HAL_UART_Transmit(&uart2_handle, (uint8_t*)frame, frame_len, 1000);
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
            HAL_UART_Transmit(&uart2_handle, (uint8_t*)map->frame, map->frame_len, 1000);
            return;
        }
    }
    
    // 未找到匹配的命令
    uart1_printf("Unknown command: %s\r\n", cmd_str);
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

/**
 * @brief 解析激光模块返回的数据帧
 * @param data 接收到的数据
 * @param len 数据长度
 * @return 解析出的距离值 (单位: mm)，如果解析失败返回0
 */
uint16_t Laser_ParseDistance(uint8_t *data, uint16_t len)
{
    int frame_start_idx = -1; // 用于存储找到的帧头位置索引，初始为-1表示未找到

    for (int i = 0; i < len; i++) {
        if (data[i] == 0xA5) {
            // 找到A5帧头，检查下一个字节是否是长度0x03
            if (i + 1 < len && data[i+1] == 0x03) {
                 frame_start_idx = i; // 找到帧头和长度，记录帧头索引
                 break; // 找到后立即退出循环
            }
        }
    }

    if (frame_start_idx == -1 || (len - frame_start_idx) < 15) {
        return 0; // 解析失败，返回0
    }

    if ((len - frame_start_idx) < 15) { // 确保从帧头开始的剩余长度至少为15
        return 0; // 数据太短，解析失败，返回0
    }

    // 组合高低字节得到距离值
    uint16_t distance = (data[frame_start_idx + 14] << 8) | data[frame_start_idx + 13];

    return distance; // 返回解析到的距离值
}

/**
 * @brief 处理来自激光模块的数据
 * @param data 接收到的数据
 * @param len 数据长度
 */
void Laser_ProcessResponse(uint8_t *data, uint16_t len)
{
    if(len == 0 || data == NULL) return;

    uint16_t distance = Laser_ParseDistance(data, len);
    
    if (distance > 0) {
        // 将解析出的距离通过UART1转发给上位机
        uart1_printf("Laser: %u mm\r\n", distance); // 保留成功的打印
    } else {
    }
}

/**
 * @brief 处理激光模块的测量流程
 * @note 检查UART2接收数据，解析距离，发送结束命令，控制单次测量
 */
void Laser_HandleMeasurement(void)
{
    // 如果正在等待激光模块数据且空闲中断发生
    if(laser_measurement_active && uart2_idle_flag)
    {
        // 处理接收到的激光模块数据，使用 uart2_frame_len 作为数据长度
        Laser_ProcessResponse(uart2_rx_buf, uart2_frame_len);
        
        // 处理完一帧后，发送结束测量指令
        Laser_ProcessStringCommand("end");
        // uart1_printf("Sent end command to laser module\r\n"); // 调试信息，根据需要开启
        
        uart2_rx_clear();
        uart2_idle_flag = 0; // 清除标志位
        uart2_frame_len = 0; // 清零帧长度
        
        // 处理完一帧并发送结束指令后，禁用UART2接收中断
        __HAL_UART_DISABLE_IT(&uart2_handle, UART_IT_RXNE);
        __HAL_UART_DISABLE_IT(&uart2_handle, UART_IT_IDLE);
        laser_measurement_active = 0; // 测量结束
    }
} 
