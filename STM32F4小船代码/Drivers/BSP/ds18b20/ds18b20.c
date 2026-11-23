/**
  ******************************************************************************
  * @file    ds18b20.c
  * @brief   DS18B20 温度传感器驱动程序
  *          使用STM32 HAL库，并依赖精确的微秒和毫秒延时函数。
  ******************************************************************************
  */

#include "ds18b20.h" 

static void DS18B20_Set_Pin_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;         // 输出模式通常不需要内部上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // 对于1-Wire，低速足够
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
}

static void DS18B20_Set_Pin_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // 1-Wire总线需要上拉 (通常外部已有4.7K上拉电阻)
                                        // 如果外部上拉可靠，这里可以设为GPIO_NOPULL
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
}

/**
 * @brief       DS18B20 GPIO和时钟初始化
 * @note        应在系统初始化时调用一次。
 * @param       无
 * @retval      无
 */
void DS18B20_Init_GPIO(void)
{
    DS18B20_CLK_ENABLE(); // 使能GPIO端口时钟
    // 初始时可以将引脚设置为输出高电平或输入（依赖外部上拉）
    DS18B20_Set_Pin_Output();
    DS18B20_DQ_OUT(1); // 默认总线空闲时为高电平
}


/**
 * @brief       DS18B20 复位并检测从设备是否存在
 * @param       无
 * @retval      1: 检测到DS18B20设备
 * @retval      0: 未检测到DS18B20设备
 */
uint8_t DS18B20_Reset(void) // 函数名改为更通用的 Reset
{
    uint8_t presence_detected = 0;

    DS18B20_Set_Pin_Output();   // 设置为输出模式
    DS18B20_DQ_OUT(0);          // 主机拉低总线，产生复位脉冲
    delay_us(500);              // 拉低至少480us (这里使用500us)

    DS18B20_DQ_OUT(1);          // 主机释放总线（通过拉高或切换到输入依赖外部上拉）
    DS18B20_Set_Pin_Input();    // 快速切换到输入模式，准备接收存在脉冲

    delay_us(70);               // 等待DS18B20响应 (15-60us后DS18B20会拉低总线)
                                // 70us应该能覆盖到DS18B20拉低的时间点

    if (DS18B20_DQ_IN == GPIO_PIN_RESET) // 如果总线被DS18B20拉低，表示设备存在
    {
        presence_detected = 1;
    }

    delay_us(410);              // 等待存在脉冲结束 (总的复位周期约480us + 从机响应时间)
                                // (500us主机拉低 + 70us等待 + ~60-240us从机拉低 + 剩余时间)
                                // 此处410us确保从开始等待从机响应到整个时序窗口结束

    return presence_detected;
}

/**
 * @brief       向DS18B20写入一个字节
 * @param       data: 要写入的字节数据
 * @retval      无
 */
void DS18B20_Write_Byte(uint8_t data) // 函数名改为更通用的 Write_Byte
{
    uint8_t i;
    DS18B20_Set_Pin_Output(); // 确保为输出模式

    for (i = 0; i < 8; i++)
    {
        DS18B20_DQ_OUT(0);    // 主机拉低总线，启动写时隙
        delay_us(2);          // 短暂拉低 (1-15us)

        if (data & 0x01)      // 判断当前要发送的位 (最低位)
        {
            DS18B20_DQ_OUT(1); // 写'1': 释放总线 (外部上拉电阻将其拉高)
            delay_us(60);     // 写'1'时隙：总线在高电平持续，总时隙长度 > 60us
        }
        else
        {
            DS18B20_DQ_OUT(0); // 写'0': 继续保持总线低电平
            delay_us(60);     // 写'0'时隙：总线在低电平持续60-120us
            DS18B20_DQ_OUT(1); // 写'0'结束后，释放总线，为下一个时隙或总线空闲做准备
        }
        data >>= 1;           // 处理下一位
        delay_us(2);          // 时隙之间的恢复时间 (可选，但推荐)
    }
}

/**
 * @brief       从DS18B20读取一个字节
 * @param       无
 * @retval      读取到的字节数据
 */
uint8_t DS18B20_Read_Byte(void) // 函数名改为更通用的 Read_Byte
{
    uint8_t i;
    uint8_t data = 0;

    for (i = 0; i < 8; i++)
    {
        DS18B20_Set_Pin_Output(); // 主机需要先拉低总线来启动读时隙
        DS18B20_DQ_OUT(0);
        delay_us(2);              // 短暂拉低 (1-15us)

        DS18B20_Set_Pin_Input();  // 快速切换到输入模式
        // delay_us(5);           // 在15us窗口内采样，给从设备一点时间将数据放到总线上 (2us拉低+5us=7us)
                                  // 如果delay_us(2)足够精确且切换够快，这个额外延时可能不需要
                                  // 更精确的做法是在15us结束前采样

        data >>= 1;               // 右移，为新读入的位腾出MSB位置
        if (DS18B20_DQ_IN == GPIO_PIN_SET) // 读取总线状态
        {
            data |= 0x80;         // 如果是高电平('1'), 设置data的最高位
        }
        // else, data的最高位保持0 (因为data >>= 1后，MSB是0)

        delay_us(60);             // 读时隙总长度至少60us，并包含恢复时间
    }
    return data;
}

/**
 * @brief       读取DS18B20的温度值
 * @param       无
 * @retval      float: 读取到的温度值 (摄氏度)。
 * @retval      特定的错误值 (如 -273.15f 或 DS18B20_ERROR_VAL) 如果读取失败。
 */
float DS18B20_Read_Temperature(void) 
{
    uint8_t scratchpad_lsb, scratchpad_msb;
    int16_t raw_temperature;
    float temperature;

    if (!DS18B20_Reset()) // 1. 复位并检测设备
    {
        return -273.15f; // 返回一个明确的错误值 (绝对零度，实际中不可能达到)
                        
    }

    // 2. 发送ROM指令 (跳过ROM搜索，对总线上单个设备操作)
    DS18B20_Write_Byte(DS18B20_CMD_SKIP_ROM); // 0xCC

    // 3. 发送功能指令 (启动温度转换)
    DS18B20_Write_Byte(DS18B20_CMD_CONVERT_T); // 0x44

    // 4. 等待温度转换完成
    //    时间取决于DS18B20的精度设置：
    //    9-bit: 93.75ms
    //    10-bit: 187.5ms
    //    11-bit: 375ms
    //    12-bit (default): 750ms
    //    这里使用固定延时，确保足够长。对于寄生供电模式，不能用读状态方式等待。
    delay_ms(760); // 给予足够的时间 (略大于750ms以确保完成)

    if (!DS18B20_Reset()) // 5. 再次复位并检测设备，为读取做准备
    {
        return -273.15f; // 返回错误值
    }

    // 6. 发送ROM指令 (跳过ROM)
    DS18B20_Write_Byte(DS18B20_CMD_SKIP_ROM); // 0xCC

    // 7. 发送功能指令 (读取暂存器内容)
    DS18B20_Write_Byte(DS18B20_CMD_READ_SCRATCHPAD); // 0xBE

    // 8. 读取暂存器中的数据 (至少前两个字节是温度)
    scratchpad_lsb = DS18B20_Read_Byte(); // 温度低字节 (Byte 0)
    scratchpad_msb = DS18B20_Read_Byte(); // 温度高字节 (Byte 1)
    // 可以选择读取更多字节，如配置寄存器或CRC

    // 9. 计算温度 (处理正负温度)
    raw_temperature = (scratchpad_msb << 8) | scratchpad_lsb;

    // DS18B20 以补码形式存储负温度
    // 12位精度时，结果乘以0.0625 (1/16)
    // 对于DS18B20，如果最高位是1，它是负数。
    // 如果是DS18B20 (不是DS18S20)，其原始数据就是符号扩展的16位值。
    // 对于12位模式，未使用的位通常是符号位。
    // 例如，如果读取的值是 0xFFF0 (-1°C in 12-bit), raw_temperature会是 -16.
    // -16 * 0.0625 = -1.0
    // 如果读取的值是 0x0010 (1°C in 12-bit), raw_temperature会是 16.
    // 16 * 0.0625 = 1.0
    // 所以直接转换通常是正确的，只要raw_temperature是int16_t类型。
    temperature = (float)raw_temperature * 0.0625f;
    return temperature;
}
