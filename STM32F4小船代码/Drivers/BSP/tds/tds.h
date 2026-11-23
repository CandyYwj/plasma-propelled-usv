// tds.h

#ifndef __TDS_H
#define __TDS_H

// 包含STM32F4的HAL库头文件和你的项目通用头文件（如果存在）
#include "stm32f4xx_hal.h"
// #include "main.h" // 或者你项目中包含HAL定义的头文件

// 如果需要用到 math.h 中的 pow 函数
#include <math.h>

// TDS传感器校准参数 (这些可以保持或根据实际情况调整)
#define TDS_VREF              3.3f  // ADC参考电压 (V) - 确保与实际VDDA电压一致
#define TDS_ADC_RESOLUTION    4095.0f // ADC分辨率 (12位)

// 校准系数，需要根据实际校准结果调整
extern float tds_k_value; // 将kValue声明为外部变量，方便在其他地方修改

// TDS传感器函数声明
HAL_StatusTypeDef TDS_Init(void); // 修改返回类型以适应HAL风格
float TDS_GetValue(float temperature);
uint16_t TDS_GetRawValue(void);
HAL_StatusTypeDef TDS_SetKValue(float k_val); // 新增：用于运行时设置k值
float TDS_GetKValue(void);                  // 新增：用于运行时获取k值

#endif /* __TDS_H */

