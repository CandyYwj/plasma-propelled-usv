#include "tim.h"

// TIM1 PWM句柄
TIM_HandleTypeDef htim1;

/**
 * @brief  TIM1 PWM初始化
 * @param  None
 * @retval None
 */
void TIM1_PWM_Init(void)
{
    // 使能TIM1时钟
    __HAL_RCC_TIM1_CLK_ENABLE();
    
    // 使能GPIOE时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();
    
    // 配置PE9和PE11为TIM1_CH1和TIM1_CH2
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    // 配置TIM1基本参数
    // APB2时钟为84MHz，TIM1时钟为84MHz
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = (84000000 / PWM_FREQUENCY / PWM_RESOLUTION) - 1;  // 预分频值
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PWM_RESOLUTION - 1;  // 自动重装值
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        // 初始化错误处理
        Error_Handler();
    }
    
    // 配置PWM通道1
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM2;  // PWM模式2
    sConfigOC.Pulse = 1000;  // 初始占空比为20% (模式2下，Pulse=1000表示0%占空比)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    
    // 配置PWM通道2
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    
    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

/**
 * @brief  设置PWM占空比 (百分比方式) - PWM模式2
 * @param  channel: PWM通道 (PWM_CHANNEL_1 或 PWM_CHANNEL_2)
 * @param  duty_percent: 占空比百分比 (0.0-100.0)
 * @retval None
 */
void TIM1_PWM_SetDuty(uint8_t channel, float duty_percent)
{
    uint16_t duty_raw;
    
    // 限制占空比范围
    if (duty_percent < 0.0f)
    {
        duty_percent = 0.0f;
    }
    else if (duty_percent > 100.0f)
    {
        duty_percent = 100.0f;
    }
    
    // PWM模式2的占空比计算：占空比 = (Period - Compare) / Period
    // 所以 Compare = Period * (1 - duty_percent/100)
    duty_raw = (uint16_t)(PWM_RESOLUTION * (1.0f - duty_percent / 100.0f));
    
    TIM1_PWM_SetDutyRaw(channel, duty_raw);
}

/**
 * @brief  设置PWM占空比 (原始值方式) - PWM模式2
 * @param  channel: PWM通道 (PWM_CHANNEL_1 或 PWM_CHANNEL_2)
 * @param  duty_raw: 占空比原始值 (0-1000，PWM模式2下，值越小占空比越大)
 * @retval None
 */
void TIM1_PWM_SetDutyRaw(uint8_t channel, uint16_t duty_raw)
{
    // 限制占空比范围
    if (duty_raw > PWM_RESOLUTION)
    {
        duty_raw = PWM_RESOLUTION;
    }
    
    switch (channel)
    {
        case PWM_CHANNEL_1:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_raw);
            break;
            
        case PWM_CHANNEL_2:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_raw);
            break;
            
        default:
            break;
    }
}

/**
 * @brief  启动PWM输出
 * @param  None
 * @retval None
 */
void TIM1_PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

/**
 * @brief  停止PWM输出
 * @param  None
 * @retval None
 */
void TIM1_PWM_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}

/**
 * @brief  TIM PWM MSP初始化
 * @param  htim: TIM句柄
 * @retval None
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1)
    {
        // TIM1时钟已经在TIM1_PWM_Init中使能
        // GPIO配置已经在TIM1_PWM_Init中完成
    }
}

/**
 * @brief  TIM PWM MSP反初始化
 * @param  htim: TIM句柄
 * @retval None
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1)
    {
        // 停止PWM输出
        HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
        
        // 禁用TIM1时钟
        __HAL_RCC_TIM1_CLK_DISABLE();
        
        // 反初始化GPIO
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_9 | GPIO_PIN_11);
    }
}
