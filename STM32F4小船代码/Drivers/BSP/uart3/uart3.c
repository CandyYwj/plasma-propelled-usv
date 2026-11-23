#include "uart3.h"
#include "stm32f4xx_hal_uart.h" // 包含UART HAL库头文件
#include "stm32f4xx_hal_dma.h" // 包含DMA HAL库头文件
#include <string.h>  // 用于memset和strlen函数

UART_HandleTypeDef huart3;             /* UART3句柄 */
DMA_HandleTypeDef hdma_uart3_rx;       /* UART3 RX DMA句柄 */

uint8_t uart3_rx_buf[UART3_RX_BUF_SIZE]; /* UART3接收缓冲区 */
uint16_t uart3_rx_len = 0;               /* UART3接收字符长度 */
uint8_t uart3_rx_complete_flag = 0;    /* UART3接收完成标志 */

/**
 * @brief       串口3初始化函数
 * @param       baudrate: 波特率
 * @retval      无
 */
void uart3_init(uint32_t baudrate)
{
    /* UART3 初始化设置 */
    huart3.Instance = USART3;                                         /* USART3 */
    huart3.Init.BaudRate = baudrate;                                  /* 波特率 */
    huart3.Init.WordLength = UART_WORDLENGTH_8B;                      /* 字长为8位数据格式 */
    huart3.Init.StopBits = UART_STOPBITS_1;                           /* 一个停止位 */
    huart3.Init.Parity = UART_PARITY_NONE;                            /* 无奇偶校验位 */
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;                      /* 无硬件流控 */
    huart3.Init.Mode = UART_MODE_TX_RX;                               /* 收发模式 */
    HAL_UART_Init(&huart3);                                           /* HAL_UART_Init()会使能UART3 */

    /* 启动DMA接收 */
    HAL_UART_Receive_DMA(&huart3, uart3_rx_buf, UART3_RX_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                      /* 使能UART3总线空闲中断 */
}

/**
 * @brief       UART3接收缓冲区清除
 * @param       无
 * @retval      无
 */
void uart3_rx_clear(void)
{
    memset(uart3_rx_buf, 0, sizeof(uart3_rx_buf));                          /* 清空接收缓冲区 */
    uart3_rx_len = 0;                                                       /* 接收计数器清零 */
}

/**
 * @brief       获取UART3接收完成标志
 * @param       无
 * @retval      1: 接收完成, 0: 未完成
 */
uint8_t uart3_get_rx_complete_flag(void)
{
    return uart3_rx_complete_flag;
}

/**
 * @brief       清除UART3接收完成标志
 * @param       无
 * @retval      无
 */
void uart3_clear_rx_complete_flag(void)
{
    uart3_rx_complete_flag = 0;
}

/**
 * @brief       获取UART3接收到的数据长度
 * @param       无
 * @retval      接收到的数据长度
 */
uint16_t uart3_get_rx_len(void)
{
    return uart3_rx_len;
}

/**
 * @brief       获取UART3接收缓冲区指针
 * @param       无
 * @retval      接收缓冲区指针
 */
uint8_t* uart3_get_rx_buf(void)
{
    return uart3_rx_buf;
}

/**
 * @brief       通过UART3发送数据
 * @param       data: 数据缓冲区指针
 * @param       len: 数据长度
 * @retval      无
 */
void uart3_send_data(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart3, data, len, 100); // 使用HAL库阻塞式发送，超时时间100ms
}

/**
 * @brief       通过UART3发送字符串
 * @param       str: 字符串指针
 * @retval      无
 */
void uart3_send_string(char *str)
{
    if (str != NULL) {
        uart3_send_data((uint8_t*)str, strlen(str));
    }
}

/**
 * @brief       UART3中断服务函数
 * @note        在此使用空闲中断检测接收完成
 * @param       无
 * @retval      无
 */
void USART3_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)        /* 获取接收空闲中断标志位是否被置位 */
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);                           /* 清除UART总线空闲中断 */

        /* 计算接收到的数据长度 */
        HAL_UART_DMAStop(&huart3);
        uart3_rx_len = UART3_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

        /* 设置接收完成标志 */
        uart3_rx_complete_flag = 1;

        /* 添加调试信息：收到数据时立即回显 */
        // if(uart3_rx_len > 0) {
        //     uart3_send_data(uart3_rx_buf, uart3_rx_len);  // 回显接收到的数据
        //     uart3_send_string("\r\n");  // 添加换行
        // }

        /* 关键：重新启动DMA接收，准备接收下一帧数据 */
        HAL_UART_Receive_DMA(&huart3, uart3_rx_buf, UART3_RX_BUF_SIZE);
    }
}

/**
 * @brief       UART3 RX DMA中断服务函数
 * @param       无
 * @retval      无
 */
void DMA1_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart3_rx);
}

