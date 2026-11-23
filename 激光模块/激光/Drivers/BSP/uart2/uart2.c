#include "uart2.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"

// UART2接收缓冲区定义
uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE];  // 接收数据缓冲区
uint16_t uart2_rx_len = 0;                // 接收数据长度计数器

// UART2句柄定义
UART_HandleTypeDef uart2_handle = {0};

uint8_t uart2_idle_flag = 0; // 定义空闲标志位并初始化
uint16_t uart2_frame_len = 0; // 定义帧长度变量并初始化

/**
 * @brief 初始化UART2
 * @param baudrate 波特率
 * @note 配置UART2的通信参数，包括波特率、数据位、停止位等
 */
void uart2_init(uint32_t baudrate)
{
    uart2_handle.Instance = USART2;                    // 选择USART2
    uart2_handle.Init.BaudRate = baudrate;             // 设置波特率
    uart2_handle.Init.WordLength = UART_WORDLENGTH_8B; // 8位数据位
    uart2_handle.Init.StopBits = UART_STOPBITS_1;      // 1位停止位
    uart2_handle.Init.Parity = UART_PARITY_NONE;       // 无校验位
    uart2_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE; // 无硬件流控
    uart2_handle.Init.Mode = UART_MODE_TX_RX;          // 收发模式
    HAL_UART_Init(&uart2_handle);                      // 初始化UART
}

/**
 * @brief 清除UART2接收缓冲区
 * @note 将接收缓冲区和接收计数器清零
 */
void uart2_rx_clear(void)
{
    memset(uart2_rx_buf, 0, sizeof(uart2_rx_buf));    // 清空接收缓冲区
    uart2_rx_len = 0;                                 // 清零接收计数器
}

/**
 * @brief 发送字符串到UART2
 * @param str 要发送的字符串
 * @return 发送的字节数
 */
uint16_t uart2_send_string(const char *str)
{
    if(str == NULL) return 0;
    uint16_t len = strlen(str);
    HAL_UART_Transmit(&uart2_handle, (uint8_t*)str, len, 1000);
    return len;
}

/**
 * @brief 发送格式化字符串到UART2
 * @param format 格式化字符串
 * @param ... 可变参数
 * @return 发送的字节数
 */
uint16_t uart2_printf(const char *format, ...)
{
    char buf[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    
    if(len > 0)
    {
        HAL_UART_Transmit(&uart2_handle, (uint8_t*)buf, len, 1000);
    }
    return len;
}

/**
 * @brief 检查UART2是否接收到完整字符串
 * @return 1:接收到完整字符串 0:未接收到完整字符串
 */
uint8_t uart2_receive_string_complete(void)
{
    if(uart2_rx_len > 0 && uart2_rx_buf[uart2_rx_len-1] == '\n')
    {
        return 1;
    }
    return 0;
}

/**
 * @brief 获取UART2接收到的字符串
 * @param str 存储接收到的字符串的缓冲区
 * @param max_len 缓冲区最大长度
 * @return 接收到的字符串长度
 */
uint16_t uart2_get_string(char *str, uint16_t max_len)
{
    if(str == NULL || max_len == 0) return 0;
    
    uint16_t len = (uart2_rx_len < max_len) ? uart2_rx_len : max_len;
    memcpy(str, uart2_rx_buf, len);
    str[len] = '\0';
    uart2_rx_clear();
    return len;
}

/**
 * @brief USART2中断服务函数
 * @note 处理接收中断和空闲中断
 */
void USART2_IRQHandler(void)
{
    uint32_t isrflags   = READ_REG(uart2_handle.Instance->SR);
    uint32_t cr1its     = READ_REG(uart2_handle.Instance->CR1);

    // 接收中断
    if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
        uint8_t receive_data = READ_REG(uart2_handle.Instance->DR);
        if(uart2_rx_len < UART2_RX_BUF_SIZE)
        {
            uart2_rx_buf[uart2_rx_len++] = receive_data;
        } else {
            // 缓冲区溢出处理，清空缓冲区并放入当前数据
            uart2_rx_len = 0;
            uart2_rx_buf[uart2_rx_len++] = receive_data;
        }
    }

    // 空闲中断
    if(((isrflags & USART_SR_IDLE) != RESET) && ((cr1its & USART_CR1_IDLEIE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&uart2_handle);
        // 空闲中断发生，设置标志位并保存当前接收长度
        uart2_frame_len = uart2_rx_len;
        uart2_idle_flag = 1;
    }
}
