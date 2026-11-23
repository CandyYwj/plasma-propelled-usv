#ifndef __UART8_H
#define __UART8_H

#include "sys.h"

/* UART8 接收缓冲区大小定义 */
#define UART8_RX_BUF_SIZE 256

/* 函数声明 */
void uart8_init(uint32_t baudrate);
void uart8_rx_clear(void);
uint8_t uart8_get_rx_complete_flag(void);
void uart8_clear_rx_complete_flag(void);
uint16_t uart8_get_rx_len(void);
uint8_t* uart8_get_rx_buf(void);
void uart8_send_data(uint8_t *data, uint16_t len);
void uart8_send_string(char *str);

/* 外部变量声明 */
extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern uint8_t uart8_rx_buf[UART8_RX_BUF_SIZE];
extern uint16_t uart8_rx_len;
extern uint8_t uart8_rx_complete_flag;

#endif
