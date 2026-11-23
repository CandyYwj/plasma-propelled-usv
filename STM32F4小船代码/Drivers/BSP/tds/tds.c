// tds.c

#include "tds.h"

// 全局ADC句柄
ADC_HandleTypeDef hadc1; // F429有ADC1, ADC2, ADC3

// 校准系数定义，可以根据实际校准结果调整
// 默认值，可以通过 TDS_SetKValue() 修改
float tds_k_value = 0.692f; // 根据90ppm标准液测量结果初步调整

/**
 * @brief   ADC1初始化 (PA0作为模拟输入)
 * @param   None
 * @retval  HAL_StatusTypeDef: HAL状态
 */
static HAL_StatusTypeDef ADC1_GPIO_And_Clock_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* 使能ADC1和GPIOA的时钟 */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 配置PA0引脚为ADC模拟输入 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1基本配置 */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // ADC时钟 = PCLK2/4
                                                          // PCLK2通常是HCLK/2 = 168/2 = 84MHz
                                                          // ADC_CLK = 84MHz/4 = 21MHz (F429 ADC最大36MHz，通常推荐<=30MHz)
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;         // 12位分辨率
    hadc1.Init.ScanConvMode = DISABLE;                  // 关闭扫描模式 (单通道)
    hadc1.Init.ContinuousConvMode = DISABLE;            // 关闭连续转换 (单次转换)
    hadc1.Init.DiscontinuousConvMode = DISABLE;         // 关闭不连续模式
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; // 无外部触发 (软件触发)
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;   // 软件触发
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;         // 数据右对齐
    hadc1.Init.NbrOfConversion = 1;                     // 转换通道数目
    hadc1.Init.DMAContinuousRequests = DISABLE;         // 关闭DMA连续请求
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;      // 单次转换结束标志
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        return HAL_ERROR; // 或调用你的 Error_Handler()
    }

    /* 配置ADC1的通道0 (PA0) */
    sConfig.Channel = ADC_CHANNEL_0;                    // PA0对应ADC1的通道0
    sConfig.Rank = 1;                                   // 转换序列中的第一个 (也是唯一一个)
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;    // 采样时间，可以根据传感器特性调整
                                                        // 480周期 @ 21MHz ADC_CLK -> 480/21MHz approx 22.8us
                                                        // 总转换时间 = 采样时间 + 12周期 (12位)
                                                        // = 480 + 12 = 492 周期 -> 492/21MHz approx 23.4us
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return HAL_ERROR; // 或调用你的 Error_Handler()
    }
    
    // ADC校准 (F4系列通常不需要像F1那样显式调用校准，但可以执行一次自校准)
    // HAL库的HAL_ADC_Start()中可能会包含一些隐式的校准或准备步骤
    // 如果需要显式校准 (通常F40x/F41x有，F42x/F43x可能不同，查阅参考手册)
    // if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    // {
    //    return HAL_ERROR;
    // }
    // 注意: F429/F439的参考手册中提到ADC上电后有一个内部校准过程，
    // 通常不需要用户软件启动校准。HAL_ADC_Init() 和 HAL_ADC_ConfigChannel() 之后ADC即可使用。

    return HAL_OK;
}

/**
 * @brief   获取ADC转换值 (单次，轮询方式)
 * @param   None
 * @retval  uint16_t: ADC值，如果失败返回0xFFFF
 */
static uint16_t Get_Adc_Value(void)
{
    // 启动ADC转换
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0xFFFF; // 返回一个错误指示值
    }

    // 等待转换完成，设置超时时间 (例如10毫秒)
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1); // 出错时停止ADC
        return 0xFFFF; // 返回一个错误指示值
    }

    // 读取转换结果
    uint16_t adc_val = HAL_ADC_GetValue(&hadc1);

    // 停止ADC转换 (对于单次转换，每次转换后都需要Start/Stop，或者配置为连续转换)
    // 如果后续频繁读取，可以考虑不停止，或者配置为连续转换然后用DMA
    // 为了与原逻辑（每次Get_Adc都启动一次）保持一致，这里也停止。
    // HAL_ADC_Stop(&hadc1); // 单次转换模式下，EOC后会自动停止，这里可以不调用Stop

    return adc_val;
}

/**
 * @brief   获取ADC平均值
 * @param   times: 采样次数
 * @retval  uint16_t: ADC平均值，如果失败返回0xFFFF
 */
static uint16_t Get_Adc_Average(uint8_t times)
{
    uint32_t temp_val = 0;
    uint16_t single_adc;
    uint8_t valid_samples = 0;

    if (times == 0) return 0; // 防止除零

    for (uint8_t t = 0; t < times; t++)
    {
        single_adc = Get_Adc_Value();
        if (single_adc != 0xFFFF) // 检查单次读取是否成功
        {
            temp_val += single_adc;
            valid_samples++;
        }
        // 可以在两次采样之间加入微小延时，如果传感器响应慢的话
        // HAL_Delay(1); // 例如延时1ms
    }

    if (valid_samples == 0) return 0xFFFF; // 如果所有采样都失败

    return (uint16_t)(temp_val / valid_samples);
}

/**
 * @brief   TDS传感器初始化
 * @param   None
 * @retval  HAL_StatusTypeDef: HAL状态
 */
HAL_StatusTypeDef TDS_Init(void)
{
    // STM32F4的HAL_Init()通常会初始化SysTick。
    // NVIC配置也通常由HAL库的特定外设初始化函数处理（如ADC中断模式）。
    // 这里主要进行ADC的GPIO和时钟初始化。
    return ADC1_GPIO_And_Clock_Init();
}

/**
 * @brief   获取TDS值
 * @param   temperature: 当前水体温度 (摄氏度)
 * @retval  float: TDS值(ppm), 如果读取失败或计算错误，可能返回负值或特定错误码
 */
float TDS_GetValue(float temperature)
{
    uint16_t adc_value = Get_Adc_Average(10);  // 获取ADC原始值 (10次平均)
    if (adc_value == 0xFFFF)
    {
        return -1.0f; // ADC读取错误指示
    }

    float voltage = (float)adc_value / TDS_ADC_RESOLUTION * TDS_VREF; // 将ADC值转换为电压

    // 温度校正系数 T修正 = 1 + 0.02 * (T - 25)
    // 确保温度在合理范围内，避免除零或不合理修正
    if (temperature < -270.0f || temperature > 200.0f) { // 随便设一个比较宽泛但合理的范围
        temperature = 25.0f; // 如果温度值异常，则按25度计算
    }
    float temperature_coefficient = 1.0f + 0.02f * (temperature - 25.0f);
    if (fabs(temperature_coefficient) < 0.0001f) { // 防止除以接近零的数
        temperature_coefficient = 1.0f;
    }


    // 温度校正后的电压 V修正 = V测试 / T修正
    float corrected_voltage = voltage / temperature_coefficient;

    // 使用TDS标准曲线公式计算初步TDS值
    // TDS测试曲线 = A * Vc^3 - B * Vc^2 + C * Vc
    // 这里使用你提供的系数：
    // float tds_value_curve = 66.71f * powf(corrected_voltage, 3.0f)
    //                       - 127.93f * powf(corrected_voltage, 2.0f)
    //                       + 428.7f * corrected_voltage;

    // 注意：原始代码的TDS_FACTOR和TDS_OFFSET没有被使用。
    // 另一种常见的简化TDS计算（如果你的TDS模块输出与TDS值近似线性关系，或者手册给了线性转换因子）：
    // float ec_value = corrected_voltage * SOME_EC_FACTOR; // 转换为EC值 uS/cm
    // float tds_value_linear = ec_value * TDS_FACTOR; // TDS_FACTOR 通常为 0.5 (NaCl) 到 0.7 (442)

    // 我们还是用你原先的曲线公式，但要确保电压值在合理范围内，避免pow函数参数问题
    if (corrected_voltage < 0) corrected_voltage = 0; // 电压不能为负

    // 更安全的做法，有些TDS模块的输出电压范围有限，例如 0-2.3V 对应 0-1000ppm
    // 如果电压超出预期范围，曲线公式可能不适用。
    // 例如，如果电压超过2.5V，曲线可能不再准确
    // if (corrected_voltage > 2.5f) {
    //     // 可以返回一个超量程的错误码，或者根据模块特性处理
    // }

    float tds_value_curve;
    // 检查电压值，避免 pow 函数在负基数和非整数指数时产生复数 (虽然这里指数是整数)
    // 或者电压过大导致数值溢出
    if (corrected_voltage >= 0.0f) {
        tds_value_curve = 66.71f * powf(corrected_voltage, 3.0f)
                        - 127.93f * powf(corrected_voltage, 2.0f)
                        + 428.7f * corrected_voltage;
    } else {
        tds_value_curve = 0.0f; // 如果修正后电压为负，TDS视为0
    }


    // 应用校准系数
    float final_tds_value = tds_value_curve * tds_k_value;

    // 确保TDS值不为负 (校准或曲线拟合不当时可能出现)
    if (final_tds_value < 0) {
        final_tds_value = 0;
    }

    return final_tds_value;
}

/**
 * @brief   获取ADC原始值 (公开接口)
 * @param   None
 * @retval  uint16_t: ADC原始值
 */
uint16_t TDS_GetRawValue(void)
{
    return Get_Adc_Average(10); // 同样进行10次平均
}

/**
 * @brief   设置TDS校准系数kValue
 * @param   k_val: 新的k值
 * @retval  HAL_StatusTypeDef
 */
HAL_StatusTypeDef TDS_SetKValue(float k_val)
{
    if (k_val > 0 && k_val < 10.0) { // 假设k值在一个合理的小范围内
        tds_k_value = k_val;
        return HAL_OK;
    }
    return HAL_ERROR; // k值不合理
}

/**
 * @brief   获取当前TDS校准系数kValue
 * @param   None
 * @retval  float: 当前k值
 */
float TDS_GetKValue(void)
{
    return tds_k_value;
}

