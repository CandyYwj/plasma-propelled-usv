#ifndef __DELAY_H__
#define __DELAY_H__

#include "stm32f4xx_hal.h" // 确保包含正确的HAL头文件

void delay_init(void);     // DWT 初始化函数 (如果使用DWT)
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);

#endif





























