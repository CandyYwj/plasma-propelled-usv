// ds18b20.h

#ifndef __DS18B20_H__
#define __DS18B20_H__

// 包含STM32F4的HAL库头文件，通常在你的工程中会有一个统一的包含点，比如 "main.h"
// 如果没有，确保这里包含了正确的HAL驱动头文件
#include "stm32f4xx_hal.h" // 对于F4系列

// 包含精确延时头文件，这个需要你为F429实现或提供
#include "delay.h"         // 假设 delay.h 提供了 delay_us() 和 delay_ms()

// DS18B20 数据引脚GPIO定义 (移植到STM32F429ZET6 PG9)
#define DS18B20_PORT          GPIOB
#define DS18B20_PIN           GPIO_PIN_0
#define DS18B20_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

// DS18B20 数据线操作宏
// 设置DQ引脚为输出状态 (x=1 为高电平, x=0 为低电平)
#define DS18B20_DQ_OUT(x)     do{ x ? \
                                HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET) : \
                                HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);\
                            }while(0)
// 获取DQ引脚的输入状态
#define DS18B20_DQ_IN         HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)

// DS18B20 ROM 指令
#define DS18B20_CMD_SEARCH_ROM      0xF0
#define DS18B20_CMD_READ_ROM        0x33
#define DS18B20_CMD_MATCH_ROM       0x55
#define DS18B20_CMD_SKIP_ROM        0xCC
#define DS18B20_CMD_ALARM_SEARCH    0xEC

// DS18B20 功能指令
#define DS18B20_CMD_CONVERT_T             0x44  // 启动温度转换
#define DS18B20_CMD_WRITE_SCRATCHPAD      0x4E  // 写暂存器
#define DS18B20_CMD_READ_SCRATCHPAD       0xBE  // 读暂存器
#define DS18B20_CMD_COPY_SCRATCHPAD       0x48  // 复制暂存器内容到EEPROM
#define DS18B20_CMD_RECALL_E2             0xB8  // 从EEPROM提取数据到暂存器
#define DS18B20_CMD_READ_POWER_SUPPLY     0xB4  // 读取供电模式

// 函数声明
void DS18B20_Init_GPIO(void); // GPIO初始化函数声明 (添加，保持良好习惯)
uint8_t DS18B20_Reset(void);
void DS18B20_Write_Byte(uint8_t data);
uint8_t DS18B20_Read_Byte(void);
float DS18B20_Read_Temperature(void);

// 定义一个错误温度值，便于识别
#define DS18B20_TEMP_ERROR -273.15f 

#endif /* __DS18B20_H__ */

