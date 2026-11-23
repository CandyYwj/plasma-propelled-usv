#ifndef __UART1_H
#define __UART1_H

#include "sys.h"

// UART1接收缓冲区大小定义
#define UART1_RX_BUF_SIZE 256

// 函数声明
void uart1_init(uint32_t baudrate);
void uart1_rx_clear(void);
uint16_t uart1_printf(const char *format, ...);

// 外部变量声明
extern uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];
extern uint16_t uart1_rx_len;

#endif


