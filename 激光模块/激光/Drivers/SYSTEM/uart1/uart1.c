/**
 * @file uart1.c
 * @brief UART1串口通信实现文件
 */

#include "uart1.h"
#include "stdio.h"
#include "string.h"
#include "laser.h"
#include "stdarg.h"

// UART1接收缓冲区定义
uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];  // 接收数据缓冲区
uint16_t uart1_rx_len = 0;                // 接收数据长度计数器

// UART1句柄定义
UART_HandleTypeDef uart1_handle = {0};

/**
 * @brief 初始化UART1
 * @param baudrate 波特率
 * @note 配置UART1的通信参数，包括波特率、数据位、停止位等
 */
void uart1_init(uint32_t baudrate)
{
    uart1_handle.Instance = USART1;                    // 选择USART1
    uart1_handle.Init.BaudRate = baudrate;             // 设置波特率
    uart1_handle.Init.WordLength = UART_WORDLENGTH_8B; // 8位数据位
    uart1_handle.Init.StopBits = UART_STOPBITS_1;      // 1位停止位
    uart1_handle.Init.Parity = UART_PARITY_NONE;       // 无校验位
    uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE; // 无硬件流控
    uart1_handle.Init.Mode = UART_MODE_TX_RX;          // 收发模式
    HAL_UART_Init(&uart1_handle);                      // 初始化UART
}

/**
 * @brief UART MSP初始化回调函数
 * @param huart UART句柄指针
 * @note 配置UART的GPIO引脚、时钟和中断优先级
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();    // 使能USART1时钟
        __HAL_RCC_GPIOA_CLK_ENABLE();     // 使能GPIOA时钟
        GPIO_InitTypeDef gpio_initstruct;
        
        // 配置USART1 TX引脚(PA9)
        gpio_initstruct.Pin = GPIO_PIN_9;              // PA9引脚
        gpio_initstruct.Mode = GPIO_MODE_AF_PP;        // 复用推挽输出
        gpio_initstruct.Pull = GPIO_PULLUP;            // 上拉
        gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;  // 高速模式
        HAL_GPIO_Init(GPIOA, &gpio_initstruct);
        
        // 配置USART1 RX引脚(PA10)
        gpio_initstruct.Pin = GPIO_PIN_10;             // PA10引脚
        gpio_initstruct.Mode = GPIO_MODE_AF_INPUT;     // 复用输入
        HAL_GPIO_Init(GPIOA, &gpio_initstruct);
        
        // 配置USART1中断
        HAL_NVIC_EnableIRQ(USART1_IRQn);              // 使能USART1中断
        HAL_NVIC_SetPriority(USART1_IRQn, 2, 2);      // 设置中断优先级
        
        // 使能接收中断和空闲中断
        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);    // 使能接收中断
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);    // 使能空闲中断
    }
    else if(huart->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_ENABLE();    // 使能USART2时钟
        __HAL_RCC_GPIOA_CLK_ENABLE();     // 使能GPIOA时钟
        GPIO_InitTypeDef gpio_initstruct;
        
        // 配置USART2 TX引脚(PA2)
        gpio_initstruct.Pin = GPIO_PIN_2;              // PA2引脚
        gpio_initstruct.Mode = GPIO_MODE_AF_PP;        // 复用推挽输出
        gpio_initstruct.Pull = GPIO_PULLUP;            // 上拉
        gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;  // 高速模式
        HAL_GPIO_Init(GPIOA, &gpio_initstruct);
        
        // 配置USART2 RX引脚(PA3)
        gpio_initstruct.Pin = GPIO_PIN_3;             // PA3引脚
        gpio_initstruct.Mode = GPIO_MODE_AF_INPUT;     // 复用输入
        HAL_GPIO_Init(GPIOA, &gpio_initstruct);
        
        // 配置USART2中断
        HAL_NVIC_EnableIRQ(USART2_IRQn);              // 使能USART2中断
        HAL_NVIC_SetPriority(USART2_IRQn, 2, 2);      // 设置中断优先级
        
        // 使能接收中断和空闲中断
        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);    // 使能接收中断
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);    // 使能空闲中断
    }
}

/**
 * @brief 清除UART1接收缓冲区
 * @note 将接收缓冲区和接收计数器清零
 */
void uart1_rx_clear(void)
{
    memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));    // 清空接收缓冲区
    uart1_rx_len = 0;                                 // 清零接收计数器
}

/**
 * @brief USART1中断服务函数
 * @note 处理接收中断和空闲中断
 */
void USART1_IRQHandler(void)
{
    uint32_t isrflags   = READ_REG(uart1_handle.Instance->SR);
    uint32_t cr1its     = READ_REG(uart1_handle.Instance->CR1);

    // 接收中断
    if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
        uint8_t receive_data = READ_REG(uart1_handle.Instance->DR);
        if(uart1_rx_len < UART1_RX_BUF_SIZE)
        {
            uart1_rx_buf[uart1_rx_len++] = receive_data;
        } else {
            // 缓冲区溢出处理，可以清空缓冲区或者丢弃当前数据
            uart1_rx_len = 0; // 清空缓冲区重新开始
            uart1_rx_buf[uart1_rx_len++] = receive_data; // 放入当前数据
        }
    }

    // 空闲中断
    if(((isrflags & USART_SR_IDLE) != RESET) && ((cr1its & USART_CR1_IDLEIE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&uart1_handle);
        // 空闲中断发生，表示一帧接收完毕，数据已在 uart1_rx_buf 中
        // 主循环会检查 uart1_rx_len > 0 来处理数据
    }
}

/**
 * @brief 重定向printf到UART1
 * @param ch 要发送的字符
 * @param f 文件指针
 * @return 发送的字符
 * @note 用于支持printf函数通过UART1输出
 */
int fputc(int ch, FILE *f)
{
    while((USART1->SR & 0X40) == 0);  // 等待发送完成
    USART1->DR = (uint8_t)ch;         // 发送数据
    return ch;
}

/**
 * @brief 发送格式化字符串到UART1
 * @param format 格式化字符串
 * @param ... 可变参数
 * @return 发送的字节数
 */
uint16_t uart1_printf(const char *format, ...)
{
    char buf[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    
    if(len > 0)
    {
        HAL_UART_Transmit(&uart1_handle, (uint8_t*)buf, len, 1000);
    }
    return len;
}
