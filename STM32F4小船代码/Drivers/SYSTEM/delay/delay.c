#include "delay.h"

#if 1 // 使用DWT外设进行精确延时 (推荐)

static uint8_t  fac_us=0; // us延时倍乘数

// 初始化延迟函数
// 当使用ucos的时候,此函数会初始化ucos的时钟节拍
// SYSTICK的时钟固定为HCLK时钟的1/8
// SYSCLK: 系统时钟频率
void delay_init(void)
{
    uint32_t SystemCoreClock = HAL_RCC_GetHCLKFreq(); // 获取HCLK频率

    // 使能DWT外设
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // DWT计数器清零
    DWT->CYCCNT = 0;
    // 使能DWT计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    fac_us = SystemCoreClock / 1000000; // 计算每微秒的CPU周期数
}

// 延时nus微秒
// nus: 要延时的微秒数
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    ticks = nus * fac_us; // 需要的节拍数
    told = DWT->CYCCNT;   // 刚进入时的计数器值
    while (1)
    {
        tnow = DWT->CYCCNT;
        if (tnow != told)
        {
            if (tnow > told) // 正常情况
            {
                tcnt += tnow - told;
            }
            else // 计数器溢出
            {
                tcnt += UINT32_MAX - told + tnow;
            }
            told = tnow;
            if (tcnt >= ticks) break; // 时间超过/等于要延迟的时间,则退出
        }
    }
}

// 延时nms毫秒
// nms: 要延时的毫秒数
void delay_ms(uint16_t nms)
{
    // 修正毫秒延时，避免无限递归，使用delay_us实现
    uint32_t i;
    for(i=0; i<nms; i++)
    {
        delay_us(1000); // 延时1毫秒
    }
}

// 延时ns秒
// ns: 要延时的秒数
void delay_s(uint32_t ns)
{
    while(ns--)
        delay_ms(1000); // 使用修正后的delay_ms
}

#else // 如果不使用DWT，可以使用简单循环延时，但精度较差，且受优化等级影响
// 这种方式非常不推荐用于1-Wire协议，但作为备选
void delay_us(uint32_t nus)
{
    uint32_t i;
    // 这个值需要根据F429的时钟频率和优化等级仔细调整
    // 假设180MHz主频，一个简单循环大约几个周期
    uint32_t ticks_per_us = HAL_RCC_GetHCLKFreq() / 1000000 / 4; // 估算值，需要调试
    for (i = 0; i < nus * ticks_per_us; i++)
    {
        __NOP();
    }
}

void delay_ms(uint16_t nms)
{
    HAL_Delay(nms);
}
#endif
