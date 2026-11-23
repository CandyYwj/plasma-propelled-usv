#ifndef __UART3_H
#define __UART3_H

#include "sys.h"

/* UART3 接收缓冲区大小定义 */
#define UART3_RX_BUF_SIZE 256

/* 函数声明 */
void uart3_init(uint32_t baudrate);
void uart3_rx_clear(void);
uint8_t uart3_get_rx_complete_flag(void);
void uart3_clear_rx_complete_flag(void);
uint16_t uart3_get_rx_len(void);
uint8_t* uart3_get_rx_buf(void);
void uart3_send_data(uint8_t *data, uint16_t len);
void uart3_send_string(char *str);

/* 外部变量声明 */
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_uart3_rx;
extern uint8_t uart3_rx_buf[UART3_RX_BUF_SIZE];
extern uint16_t uart3_rx_len;
extern uint8_t uart3_rx_complete_flag;

#endif
