#ifndef __UART6_H__
#define __UART6_H__

#include "sys.h" // 包含必要的HAL库头文件
#include <stdbool.h> // 新增：为了使用bool类型

// 定义USART接收缓冲区的最大长度（根据预期的NMEA语句长度进行调整）
#define USART_REC_LEN 500

// 定义一个回调函数类型，用于在接收到完整数据帧时通知外部模块
// 修改回调函数类型，增加void* userData参数
typedef void (*UartRxCallback)(const char* data, uint16_t len, void* userData);

extern UART_HandleTypeDef huart6; // 声明USART6的UART句柄
extern uint8_t Uart6_Receive_buf[1];    // 通过中断接收单个字节的缓冲区

void MX_USART6_UART_Init(void); // USART6初始化函数
void CLR_Buf(void);             // 清空接收缓冲区函数（适配USART6）
uint8_t Hand(char *a);               // 检查接收缓冲区是否存在子字符串函数（适配）
// 修改注册函数，增加void* userData参数
void UART6_RegisterRxCallback(UartRxCallback callback, void* userData); // 注册接收回调函数

#endif /* __UART6_H__ */
