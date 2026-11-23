#ifndef __TIM_H
#define __TIM_H

#include "stm32f4xx_hal.h"
#include "sys.h"

// PWM通道定义
#define PWM_CHANNEL_1    1
#define PWM_CHANNEL_2    2

// PWM频率和分辨率定义
#define PWM_FREQUENCY    1000    // PWM频率 1kHz
#define PWM_RESOLUTION   1000    // PWM分辨率 1000级 (对应0-100%)

// PWM模式2说明：
// - 当CNT < CCR时，输出为有效电平
// - 当CNT >= CCR时，输出为无效电平
// - 占空比 = (Period - Compare) / Period
// - 所以Compare值越小，占空比越大

// 函数声明
void TIM1_PWM_Init(void);
void TIM1_PWM_SetDuty(uint8_t channel, float duty_percent);
void TIM1_PWM_SetDutyRaw(uint8_t channel, uint16_t duty_raw);
void TIM1_PWM_Start(void);
void TIM1_PWM_Stop(void);

#endif /* __TIM_H */
