/**
  ******************************************************************************
  * @file    tds.c
  * @version V1.0
  * @note    TDS传感器驱动
  ******************************************************************************
  */

#include "sysinc.h"
#include "tds.h"

// TDS传感器校准参数
#define TDS_FACTOR 0.5  // TDS转换系数
#define TDS_OFFSET 0    // TDS偏移量

// ADC通道定义
#define ADC_CH0   0       // PA0

/**
 * @brief   NVIC初始化
 * @param   None
 * @retval  None
 */
static void NVIC_init(void)
{
    NVIC_SetPriorityGrouping(0);
    NVIC_SetPriority(SysTick_IRQn, 0x0F);
    NVIC_SetPriority(ADC1_2_IRQn, 0x0F);
}

/**
 * @brief   延时初始化
 * @param   None
 * @retval  None
 */
static void SYSTICK_DelayInit(void)
{
    SysTick->CTRL = 0;
    SysTick->LOAD = 0xFFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = 5;
}

/**
 * @brief   ADC初始化
 * @param   None
 * @retval  None
 */
static void ADC_Init(void)
{    
    // 配置PA0为模拟输入
    RCC->APB2ENR|=1<<2;    // 使能PORTA时钟
    GPIOA->CRL&=0XFFFFFFF0; // PA0设置为模拟输入
    
    // ADC1时钟使能
    RCC->APB2ENR|=1<<9;    // ADC1时钟使能
    RCC->APB2RSTR|=1<<9;   // ADC1复位
    RCC->APB2RSTR&=~(1<<9);// 复位结束
    
    // ADC时钟配置
    RCC->CFGR&=~(3<<14);   // 分频因子清零
    RCC->CFGR|=2<<14;      // SYSCLK/DIV2=12M ADC时钟设置为12M
    
    // ADC配置
    ADC1->CR1&=0XF0FFFF;   // 工作模式清零
    ADC1->CR1|= 0<<16;     // 独立工作模式
    ADC1->CR1|= ~(1<<8);   // 非扫描模式
    ADC1->CR2|= 1<<20;     // 使用外部触发
    ADC1->CR2&= ~(1<<1);   // 单次转换模式
    ADC1->CR2|= 7<<17;     // 外部触发模式
    ADC1->CR2&=~(1<<11);   // 右对齐
    ADC1->SMPR2&=~(7<<3);  // 通道0采样时间清空
    ADC1->SQR1&=~(0XF<<20);// 规则序列1 通道数
    ADC1->SQR1|=0<<20;     // 1个转换
    
    // 使能ADC
    ADC1->CR2|=1<<0;       // 开启ADC转换器
    ADC1->CR2|=1<<3;       // 使能复位校准
    while(ADC1->CR2&(1<<3));// 等待复位校准结束
    ADC1->CR2|=1<<2;       // 开始校准
    while(ADC1->CR2&(1<<2));// 等待校准结束
}

/**
 * @brief   获取ADC转换值
 * @param   None
 * @retval  u16: ADC值
 */
static u16 Get_Adc(void)
{
    ADC1->SQR3&=0XFFFFFFE0;// 规则序列1 通道ch
    ADC1->SQR3|=ADC_CH0;   // 设置通道0
    ADC1->CR2|=1<<22;      // 启动规则转换通道
    while(!(ADC1->SR&1<<1));// 等待转换结束
    return ADC1->DR;       // 返回adc值
}

/**
 * @brief   获取ADC平均值
 * @param   times: 采样次数
 * @retval  u16: ADC平均值
 */
static u16 Get_Adc_Average(u8 times)
{
    u32 temp_val=0;
    u8 t;
    for(t=0;t<times;t++)
    {
        temp_val+=Get_Adc();
    }
    return temp_val/times;
}

/**
 * @brief   TDS传感器初始化
 * @param   None
 * @retval  None
 */
void TDS_Init(void)
{
    NVIC_init();        // 初始化NVIC
    SYSTICK_DelayInit();// 初始化延时函数
    ADC_Init();         // 初始化ADC
}

/**
 * @brief   获取TDS值
 * @param   None
 * @retval  float: TDS值(ppm)
 */
float TDS_GetValue(void)
{
    u16 adc_value = Get_Adc_Average(10);  // 获取ADC原始值
    return (float)adc_value * TDS_FACTOR + TDS_OFFSET;  // 转换为TDS值
}

/**
 * @brief   获取ADC原始值
 * @param   None
 * @retval  u16: ADC原始值
 */
u16 TDS_GetRawValue(void)
{
    return Get_Adc_Average(10);
} 