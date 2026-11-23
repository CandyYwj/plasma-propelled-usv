#ifndef __USART_H__
#define __USART_H__

#include "stdio.h"
#include "string.h"
#include "sys.h"

/* ������� */
#define UART_EOK                     0      /* û�д��� */
#define UART_ERROR                   1      /* ͨ�ô��� */
#define UART_ETIMEOUT                2      /* ��ʱ���� */
#define UART_EINVAL                  3      /* �������� */

/* UART�շ������С */
#define UART1_RX_BUF_SIZE            128
#define UART1_TX_BUF_SIZE            128


void uart1_init(uint32_t bound);            /* ���ڳ�ʼ������ */

/* 获取接收完成标志 */
uint8_t uart1_get_rx_complete_flag(void);

/* 清除接收完成标志 */
void uart1_clear_rx_complete_flag(void);

/* 获取接收到的数据长度 */
uint16_t uart1_get_rx_len(void);

/* 获取接收缓冲区指针 */
uint8_t* uart1_get_rx_buf(void);

/* UART1接收缓冲区清除 */
void uart1_rx_clear(void);

/* 通过UART1发送数据 */
void uart1_send_data(uint8_t *data, uint16_t len);

/* 通过UART1接收数据 */
uint8_t uart1_receive_data(uint8_t *buffer, uint16_t *len);

#endif


