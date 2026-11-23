// uart2.c
#include "uart2.h"
#include "stm32f4xx_hal_uart.h" // 包含UART HAL库头文件
#include "stm32f4xx_hal_dma.h" // 包含DMA HAL库头文件
#include <string.h>  // 添加string.h头文件，用于memset和strlen函数
#include "uart3.h" // 添加uart3.h，用于调试输出

UART_HandleTypeDef huart2;             /* UART2句柄 */
DMA_HandleTypeDef hdma_uart2_rx;       /* UART2 RX DMA句柄 */

uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE]; /* UART2接收缓冲区 */
uint16_t uart2_rx_len = 0;               /* UART2接收字符长度 */
uint8_t uart2_rx_complete_flag = 0;    /* UART2接收完成标志 */

/**
 * @brief       串口2初始化函数
 * @param       baudrate: 波特率
 * @retval      无
 */
void uart2_init(uint32_t baudrate)
{
    /* UART2 初始化设置 */
    huart2.Instance = USART2;                                         /* USART2 */
    huart2.Init.BaudRate = baudrate;                                  /* 波特率 */
    huart2.Init.WordLength = UART_WORDLENGTH_8B;                      /* 字长为8位数据格式 */
    huart2.Init.StopBits = UART_STOPBITS_1;                           /* 一个停止位 */
    huart2.Init.Parity = UART_PARITY_NONE;                            /* 无奇偶校验位 */
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;                      /* 无硬件流控 */
    huart2.Init.Mode = UART_MODE_TX_RX;                               /* 收发模式 */
    HAL_UART_Init(&huart2);                                           /* HAL_UART_Init()会使能UART2 */
    
    /* 启动DMA接收 */
    HAL_UART_Receive_DMA(&huart2, uart2_rx_buf, UART2_RX_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);                      /* 使能UART2总线空闲中断 */
}

/**
 * @brief       UART2接收缓冲区清除
 * @param       无
 * @retval      无
 */
void uart2_rx_clear(void)
{
    memset(uart2_rx_buf, 0, sizeof(uart2_rx_buf));                          /* 清空接收缓冲区 */
    uart2_rx_len = 0;                                                       /* 接收计数器清零 */
}

/**
 * @brief       获取UART2接收完成标志
 * @param       无
 * @retval      1: 接收完成, 0: 未完成
 */
uint8_t uart2_get_rx_complete_flag(void)
{
    return uart2_rx_complete_flag;
}

/**
 * @brief       清除UART2接收完成标志
 * @param       无
 * @retval      无
 */
void uart2_clear_rx_complete_flag(void)
{
    uart2_rx_complete_flag = 0;
}

/**
 * @brief       获取UART2接收到的数据长度
 * @param       无
 * @retval      接收到的数据长度
 */
uint16_t uart2_get_rx_len(void)
{
    return uart2_rx_len;
}

/**
 * @brief       获取UART2接收缓冲区指针
 * @param       无
 * @retval      接收缓冲区指针
 */
uint8_t* uart2_get_rx_buf(void)
{
    return uart2_rx_buf;
}

/**
 * @brief       通过UART2发送数据
 * @param       data: 数据缓冲区指针
 * @param       len: 数据长度
 * @retval      无
 */
void uart2_send_data(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart2, data, len, 100); // 使用HAL库阻塞式发送，超时时间100ms
}

/**
 * @brief       通过UART2发送字符串
 * @param       str: 字符串指针
 * @retval      无
 */
void uart2_send_string(char *str)
{
    if (str != NULL) {
        uart2_send_data((uint8_t*)str, strlen(str));
    }
}

/**
 * @brief       UART2中断服务函数
 * @note        在此使用空闲中断检测接收完成
 * @param       无
 * @retval      无
 */
void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)        /* 获取接收空闲中断标志位是否被置位 */
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);                           /* 清除UART总线空闲中断 */
        
        /* 计算接收到的数据长度 */
        HAL_UART_DMAStop(&huart2);
        uart2_rx_len = UART2_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
        
        /* 设置接收完成标志 */
        uart2_rx_complete_flag = 1;

        /* 关键：重新启动DMA接收，准备接收下一帧数据 */
        HAL_UART_Receive_DMA(&huart2, uart2_rx_buf, UART2_RX_BUF_SIZE);
    }
}

/**
 * @brief       UART2 RX DMA中断服务函数
 * @param       无
 * @retval      无
 */
void DMA1_Stream5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart2_rx);
}
