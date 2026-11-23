#include "laser.h"
#include "uart2.h"
#include <string.h>
#include "uart3.h" // Added for debug prints

// 启动测量指令 (发送一次后，模块周期性自动应答)
static const uint8_t laser_start_cmd[] = {0xA5, 0x03, 0x20, 0x01, 0x00, 0x00, 0x00, 0x6E, 0x02};

// 结束测量指令
static const uint8_t laser_stop_cmd[] = {0xA5, 0x03, 0x20, 0x02, 0x00, 0x00, 0x00, 0x6E, 0x46};

// 激光数据帧的固定头和数据区长度字节的位置和期望值
#define LASER_FRAME_HEADER      0xA5
#define LASER_FRAME_TYPE        0x03
#define LASER_DATA_LEN_POS      7    // 数据区长度字节在帧中的位置 (从0开始计数)
#define LASER_DISTANCE_L_POS    14   // 主目标距离低字节在帧中的位置
#define LASER_DISTANCE_H_POS    15   // 主目标距离高字节在帧中的位置
#define LASER_MIN_FRAME_LEN     (LASER_DISTANCE_H_POS + 1 + 2) // 最小帧长至少包含Header+Type+...DataLen+...Distance(L+H)+CRC(L+H)
                                                              // A5 03 20 01 00 00 00 0E FF FF FF FF FF FF 4B 03 5E 00 24 23 01 00 00 BB D8 (Total 25 bytes)
                                                              // Minimal check: Header(1) + Type(1) + Fixed(5) + DataLen(1) + DataArea + CRC(2). At least 1+1+5+1+0+2 = 10 bytes.
                                                              // Based on example: 25 bytes.
#define LASER_EXPECTED_FRAME_LEN 25 // 期望的完整数据帧长度

/**
 * @brief       激光模块初始化 (目前仅包含发送启动指令)
 * @param       无
 * @retval      无
 */
void laser_init(void)
{
    // UART2的初始化已在main.c中调用uart2_init完成
    // 这里可以添加激光模块特定的初始化，如果需要的话
}

/**
 * @brief       发送激光模块启动测量指令
 * @param       无
 * @retval      无
 */
void laser_send_start_cmd(void)
{
    uart3_send_string("[DEBUG]: Calling laser_send_start_cmd().\r\n"); // Added debug print
    uart2_send_data((uint8_t*)laser_start_cmd, sizeof(laser_start_cmd));
}

/**
 * @brief       发送激光模块结束测量指令
 * @param       无
 * @retval      无
 */
void laser_send_stop_cmd(void)
{
    uart3_send_string("[DEBUG]: Calling laser_send_stop_cmd().\r\n"); // 添加调试信息
    uart2_send_data((uint8_t*)laser_stop_cmd, sizeof(laser_stop_cmd));
}

/**
 * @brief       从UART2接收并解析激光数据帧，获取距离值
 * @param       无
 * @retval      uint16_t: 距离值(mm)，如果未收到完整有效帧或解析失败返回0xFFFF
 */
uint16_t laser_get_distance(void)
{
    uint16_t rx_len;
    uint16_t distance = 0xFFFF; // 默认返回错误值
    uint8_t* raw_data = NULL;

    // 检查UART2是否接收到新数据 (通过空闲中断标志)
    if (uart2_get_rx_complete_flag())
    {
        // 获取接收到的数据长度和缓冲区指针
        rx_len = uart2_get_rx_len();
        raw_data = uart2_get_rx_buf();

        // 清除接收完成标志，准备接收下一帧
        uart2_clear_rx_complete_flag();

        // --- 数据帧解析 --- //
        // 在接收到的数据中搜索并解析激光数据帧
        if (rx_len > 0 && raw_data != NULL) {
            // 遍历接收缓冲区，查找可能的帧头 A5 03 20 01 (4字节)
            for (int i = 0; i + 3 < rx_len; i++) { 
                // 检查帧头 A5 03 20 01
                if (raw_data[i] == 0xA5 && raw_data[i+1] == 0x03 && raw_data[i+2] == 0x20 && raw_data[i+3] == 0x01) {

                    // 潜在的一个完整帧的开始找到
                    // 根据您的描述，总帧长为 23 字节
                    uint16_t expected_frame_len = 23;

                    // 检查缓冲区是否有足够的字节构成完整的 23 字节帧
                    if (i + expected_frame_len <= rx_len) {
                        // 找到一个完整的帧，现在解析距离
                        // 根据您的描述，距离低字节在帧头偏移13，高字节在帧头偏移14
                        int distance_l_pos_in_frame = i + 13; // 距离低字节在缓冲区中的实际索引
                        int distance_h_pos_in_frame = i + 14; // 距离高字节在缓冲区中的实际索引

                        // 确保距离字节在当前帧范围内 (只需检查高字节位置，因为它在后)
                        if (distance_h_pos_in_frame < i + expected_frame_len) {
                            uint8_t distance_l = raw_data[distance_l_pos_in_frame];
                            uint8_t distance_h = raw_data[distance_h_pos_in_frame];

                            // 组合高字节和低字节 (高字节在后，低字节在前)
                            distance = (uint16_t)(distance_h << 8) | distance_l;

                            // TODO: 进行CRC16校验
                            // 如果校验失败，可以将distance设置为0xFFFF或记录错误

                            // 找到了并解析了一个帧，可以停止搜索或者继续查找下一个帧
                            // 为了简化，这里先返回第一个找到的距离

                            // Important: Since we processed data up to i + expected_frame_len, 
                            // we should adjust the loop index to continue searching after this frame.
                            i = i + expected_frame_len - 1; // Loop will increment i to i + expected_frame_len

                            return distance; // 返回解析到的距离值

                        }
                    }
                }
            }
             // 如果循环结束还没有找到完整的帧，说明当前接收到的数据不足以构成一个完整帧或者数据有误
             // 这些数据会在下次接收到新数据时一起处理 (因为我们没有清空除了已处理的部分)
             // 但是我们的uart2_clear_rx_complete_flag()和HAL_UART_Receive_DMA() 会重置DMA接收，覆盖旧数据
             // 这种处理连续帧的方式，需要一个累积缓冲区，不是直接在DMA缓冲区上操作
             // 考虑到现有的UART2驱动结构，基于空闲中断和DMA接收到固定缓冲区，
             // 最简单的处理方式是：当标志置位时，将整个接收到的数据视为一坨，在里面搜索帧头并解析第一帧有效的。
             // 如果一帧超过缓冲区大小，或者多帧连在一起，这种方式会有问题。
             // 但基于用户之前的代码结构，我们先尝试这种搜索帧头的方式。

             // Reset the flag and DMA after processing whatever data is in the buffer
             // This was already done at the beginning of the function.
        }
         // 如果 rx_len == 0 或 raw_data == NULL，说明没有收到数据，保持 distance 为默认错误值 0xFFFF
    }

    return distance; // 如果没有接收到完整有效的帧，返回默认错误值
}
