#ifndef __UART2_H
#define __UART2_H

#include "sys.h"

#define UART2_RX_BUF_SIZE  256
#define UART2_TX_BUF_SIZE  64

#define UART_EOK        0
#define UART_ERROR      1
#define UART_ETIMEOUT   2
#define UART_EINVAL     3

// 外部变量声明
extern uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE];
extern uint16_t uart2_rx_len;
extern uint8_t uart2_idle_flag;
extern uint16_t uart2_frame_len;
extern UART_HandleTypeDef uart2_handle;

// 函数声明
void uart2_init(uint32_t baudrate);
void uart2_receiv_test(void);
void uart2_rx_clear(void);
uint16_t uart2_send_string(const char *str);
uint16_t uart2_printf(const char *format, ...);
uint8_t uart2_receive_string_complete(void);
uint16_t uart2_get_string(char *str, uint16_t max_len);

#endif


