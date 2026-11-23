#ifndef __UART2_H
#define __UART2_H

#include "sys.h"

#define UART2_RX_BUF_SIZE 256  /* UART2接收缓冲区大小 */

/* 函数声明 */
void uart2_init(uint32_t baudrate);                    /* UART2初始化函数 */
void uart2_rx_clear(void);                            /* UART2接收缓冲区清除 */
uint8_t uart2_get_rx_complete_flag(void);             /* 获取UART2接收完成标志 */
void uart2_clear_rx_complete_flag(void);              /* 清除UART2接收完成标志 */
uint16_t uart2_get_rx_len(void);                      /* 获取UART2接收到的数据长度 */
uint8_t* uart2_get_rx_buf(void);                      /* 获取UART2接收缓冲区指针 */
void uart2_send_data(uint8_t *data, uint16_t len);    /* 通过UART2发送数据 */
void uart2_send_string(char *str);                    /* 通过UART2发送字符串 */

/* 外部变量声明 */
extern UART_HandleTypeDef huart2;                     /* UART2句柄 */
extern DMA_HandleTypeDef hdma_uart2_rx;               /* UART2 RX DMA句柄 */
extern uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE];       /* UART2接收缓冲区 */
extern uint16_t uart2_rx_len;                         /* UART2接收字符长度 */
extern uint8_t uart2_rx_complete_flag;                /* UART2接收完成标志 */

#endif
