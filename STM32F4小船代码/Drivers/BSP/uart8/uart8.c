// uart8.c
#include "uart8.h"
#include "stm32f4xx_hal_uart.h" // 包含UART HAL库头文件
#include "stm32f4xx_hal_dma.h" // 包含DMA HAL库头文件
#include <string.h>  // 用于memset和strlen函数

UART_HandleTypeDef huart8;             /* UART8句柄 */
DMA_HandleTypeDef hdma_uart8_rx;       /* UART8 RX DMA句柄 */

uint8_t uart8_rx_buf[UART8_RX_BUF_SIZE]; /* UART8接收缓冲区 */
uint16_t uart8_rx_len = 0;               /* UART8接收字符长度 */
uint8_t uart8_rx_complete_flag = 0;    /* UART8接收完成标志 */

/**
 * @brief       串口8初始化函数
 * @param       baudrate: 波特率
 * @retval      无
 */
void uart8_init(uint32_t baudrate)
{
    /* 使能UART8和GPIOE时钟 */
    __HAL_RCC_UART8_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* UART8 初始化设置 */
    huart8.Instance = UART8;                                         /* UART8 */
    huart8.Init.BaudRate = baudrate;                                  /* 波特率 */
    huart8.Init.WordLength = UART_WORDLENGTH_8B;                      /* 字长为8位数据格式 */
    huart8.Init.StopBits = UART_STOPBITS_1;                           /* 一个停止位 */
    huart8.Init.Parity = UART_PARITY_NONE;                            /* 无奇偶校验位 */
    huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;                      /* 无硬件流控 */
    huart8.Init.Mode = UART_MODE_TX_RX;                               /* 收发模式 */
    
    /* 初始化UART8 */
    if (HAL_UART_Init(&huart8) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 启动DMA接收 */
    if (HAL_UART_Receive_DMA(&huart8, uart8_rx_buf, UART8_RX_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 使能UART8总线空闲中断 */
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
    
    /* 清空接收缓冲区 */
    uart8_rx_clear();
}

/**
 * @brief       UART8接收缓冲区清除
 * @param       无
 * @retval      无
 */
void uart8_rx_clear(void)
{
    memset(uart8_rx_buf, 0, sizeof(uart8_rx_buf));                          /* 清空接收缓冲区 */
    uart8_rx_len = 0;                                                       /* 接收计数器清零 */
}

/**
 * @brief       获取UART8接收完成标志
 * @param       无
 * @retval      1: 接收完成, 0: 未完成
 */
uint8_t uart8_get_rx_complete_flag(void)
{
    return uart8_rx_complete_flag;
}

/**
 * @brief       清除UART8接收完成标志
 * @param       无
 * @retval      无
 */
void uart8_clear_rx_complete_flag(void)
{
    uart8_rx_complete_flag = 0;
}

/**
 * @brief       获取UART8接收到的数据长度
 * @param       无
 * @retval      接收到的数据长度
 */
uint16_t uart8_get_rx_len(void)
{
    return uart8_rx_len;
}

/**
 * @brief       获取UART8接收缓冲区指针
 * @param       无
 * @retval      接收缓冲区指针
 */
uint8_t* uart8_get_rx_buf(void)
{
    return uart8_rx_buf;
}

/**
 * @brief       通过UART8发送数据
 * @param       data: 数据缓冲区指针
 * @param       len: 数据长度
 * @retval      无
 */
void uart8_send_data(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart8, data, len, 100); // 使用HAL库阻塞式发送，超时时间100ms
}

/**
 * @brief       通过UART8发送字符串
 * @param       str: 字符串指针
 * @retval      无
 */
void uart8_send_string(char *str)
{
    if (str != NULL) {
        uart8_send_data((uint8_t*)str, strlen(str));
    }
}

/**
 * @brief       UART8中断服务函数
 * @note        在此使用空闲中断检测接收完成
 * @param       无
 * @retval      无
 */
void UART8_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE) != RESET)        /* 获取接收空闲中断标志位是否被置位 */
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart8);                           /* 清除UART总线空闲中断 */
        
        /* 计算接收到的数据长度 */
        HAL_UART_DMAStop(&huart8);
        uart8_rx_len = UART8_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart8.hdmarx);
        
        /* 设置接收完成标志 */
        uart8_rx_complete_flag = 1;

        /* 关键：重新启动DMA接收，准备接收下一帧数据 */
        HAL_UART_Receive_DMA(&huart8, uart8_rx_buf, UART8_RX_BUF_SIZE);
    }
}

/**
 * @brief       UART8 RX DMA中断服务函数
 * @param       无
 * @retval      无
 */
void DMA1_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart8_rx);
}
