#include "ultrasonic.h"
#include "uart1.h"

// 测量指令
static const uint8_t measure_cmd[] = {0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0xD4, 0x36};

/**
 * @brief       发送超声波测量指令
 * @param       无
 * @retval      无
 */
void ultrasonic_send_measure_cmd(void)
{
    uart1_send_data((uint8_t*)measure_cmd, sizeof(measure_cmd));
}

/**
 * @brief       解析超声波返回数据
 * @param       data: 接收到的数据
 * @param       len: 数据长度
 * @retval      uint16_t: 距离值(mm)，如果解析失败返回0xFFFF
 */
uint16_t ultrasonic_parse_data(uint8_t *data, uint16_t len)
{
    // 检查数据长度是否为4字节
    if (len != 4)
    {
        return 0xFFFF;
    }
    
    // 提取中间两个字节作为距离值
    uint16_t distance = (data[1] << 8) | data[2];
    
    return distance;
}

/**
 * @brief       获取超声波测量值
 * @param       无
 * @retval      uint16_t: 距离值(mm)，如果测量失败返回0xFFFF
 */
uint16_t ultrasonic_get_distance(void)
{
    uint8_t rx_buf[4];
    uint16_t rx_len;
    
    // 发送测量指令
    ultrasonic_send_measure_cmd();
    
    // 等待接收数据
    if (uart1_receive_data(rx_buf, &rx_len) == 0)
    {
        // 解析数据
        return ultrasonic_parse_data(rx_buf, rx_len);
    }
    
    return 0xFFFF;
}


