#include "IOI2C.h"

// Microsecond delay function (very basic, calibrate or use DWT for accuracy)
// For a 168MHz or 180MHz STM32F4, this loop count needs adjustment.
// The original Delay(N) was N*8 loops. If F103 was 72MHz, Delay(5) = 40 CPU cycles.
// 40 cycles / 72MHz = ~0.55 microseconds.
// Let's aim for delays around 1-5 microseconds.
// A common I2C speed is 100kHz (10us period) or 400kHz (2.5us period).
// Min clock high/low time is ~4us for 100kHz, ~0.6us for 400kHz.
// We will create a rough delay_us function.
// For accurate timing, especially at higher I2C speeds, using the DWT cycle counter is recommended.
static void soft_delay_us(volatile uint32_t nus)
{
    // This factor is empirical and depends on CPU clock and compiler optimizations.
    // For STM32F429 @180MHz, a simple loop takes a few cycles.
    // To get 1us, you'd need roughly (180 / num_cycles_per_loop) iterations.
    // Let's assume roughly 10-20 cycles per loop with volatile.
    // So, for 1us, maybe 180/10 = 18 iterations.
    // This is a placeholder. Calibrate it!
    volatile uint32_t count = nus * (SystemCoreClock / 1000000 / 10); // Adjust the divisor (10)
    while (count--);
}


// Configure SDA pin as Output
static void SDA_Set_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = IIC_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;         // Or GPIO_PULLUP if you want to use internal pull-up
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // Max speed
    HAL_GPIO_Init(IIC_SDA_GPIO_Port, &GPIO_InitStruct);
}

// Configure SDA pin as Input
static void SDA_Set_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = IIC_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Enable pull-up for input as I2C lines are open-drain conceptually
    HAL_GPIO_Init(IIC_SDA_GPIO_Port, &GPIO_InitStruct);
}

void IIC_Init_HAL(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIOB clock
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure SCL pin (PB10)
    GPIO_InitStruct.Pin = IIC_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // SCL is always output from master
    GPIO_InitStruct.Pull = GPIO_PULLUP;         // Often SCL needs a pull-up
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC_SCL_GPIO_Port, &GPIO_InitStruct);

    // Configure SDA pin (PB11) - initially as output
    // Note: The original code configures SDA as PP output.
    // For true I2C, SDA should be OD (Open Drain) and requires external pull-ups.
    // Or, if using PP, ensure the slave device can handle it.
    // We are replicating original behavior.
    GPIO_InitStruct.Pin = IIC_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Pull-up for high state
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC_SDA_GPIO_Port, &GPIO_InitStruct);

    // Set initial state to idle (SCL=1, SDA=1)
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET);
}

void IIC_Start_HAL(void)
{
    SDA_Set_Output(); // Ensure SDA is output
    HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET);
    soft_delay_us(4); // Original: Delay(5)
    HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_RESET); // START: when CLK is high, DATA change from high to low
    soft_delay_us(4); // Original: Delay(5)
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET); // Clamp SCL low to allow next bit
}

void IIC_Stop_HAL(void)
{
    SDA_Set_Output(); // Ensure SDA is output
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_RESET); // STOP: when CLK is high DATA change from low to high
    soft_delay_us(4); // Original: Delay(5)
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET);
    soft_delay_us(4); // Original: Delay(5)
}

uint8_t IIC_Wait_Ack_HAL(void)
{
    uint8_t ucErrTime = 0;
    SDA_Set_Input();  // Set SDA to input mode to read ACK
    // HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET); // Not needed, input mode with pull-up
    soft_delay_us(2); // Original: Delay(5) - allow slave to respond
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET); // Clock high
    soft_delay_us(2); // Original: Delay(5) - ACK setup time

    while (HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port, IIC_SDA_Pin) == GPIO_PIN_SET)
    {
        ucErrTime++;
        if (ucErrTime > 250) // Timeout increased slightly, original was 50 * Delay(5)
        {
            IIC_Stop_HAL();
            return 1; // NACK or error
        }
        // soft_delay_us(1); // Small delay if needed, original had Delay(5) here
    }
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET); // Clock low
    return 0; // ACK received
}

void IIC_Ack_HAL(void)
{
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET);
    SDA_Set_Output();
    HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_RESET); // Send ACK (SDA low)
    soft_delay_us(2); // Original: Delay(5)
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET);
    soft_delay_us(2); // Original: Delay(5)
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET);
}

void IIC_NAck_HAL(void)
{
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET);
    SDA_Set_Output();
    HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET); // Send NACK (SDA high)
    soft_delay_us(2); // Original: Delay(5)
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET);
    soft_delay_us(2); // Original: Delay(5)
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET);
}

void IIC_Send_Byte_HAL(uint8_t txd)
{
    uint8_t t;
    SDA_Set_Output();
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET); // SCL low
    for (t = 0; t < 8; t++)
    {
        if ((txd & 0x80) >> 7)
            HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_RESET);
        txd <<= 1;
        soft_delay_us(2); // Original: Delay(2) - Data setup time
        HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET); // SCL high to clock data
        soft_delay_us(4); // Original: Delay(5) - SCL high pulse width
        HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET); // SCL low
        soft_delay_us(2); // Original: Delay(3) - SCL low pulse width / hold time
    }
}

uint8_t IIC_Read_Byte_HAL(uint8_t ack)
{
    unsigned char i;
    uint8_t receive = 0;
    SDA_Set_Input(); // Set SDA to input
    for (i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET); // SCL low
        soft_delay_us(2); // Original: Delay(5)
        HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET); // SCL high to clock data
        receive <<= 1;
        if (HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port, IIC_SDA_Pin) == GPIO_PIN_SET)
            receive++;
        soft_delay_us(2); // Original: Delay(5)
    }
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET); // Ensure SCL is low before ACK/NACK

    if (ack)
        IIC_Ack_HAL();  // Send ACK
    else
        IIC_NAck_HAL(); // Send NACK
    return receive;
}


int32_t IIC_ReadBytes_HAL(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint32_t length)
{
    uint32_t count = 0;

    IIC_Start_HAL();
    IIC_Send_Byte_HAL(dev_addr); // Send device address (write mode)
    if (IIC_Wait_Ack_HAL() == 1) { IIC_Stop_HAL(); return 0; }

    IIC_Send_Byte_HAL(reg_addr); // Send register address
    if (IIC_Wait_Ack_HAL() == 1) { IIC_Stop_HAL(); return 0; }

    IIC_Start_HAL(); // Repeated start
    IIC_Send_Byte_HAL(dev_addr + 1); // Send device address (read mode)
    if (IIC_Wait_Ack_HAL() == 1) { IIC_Stop_HAL(); return 0; }

    for (count = 0; count < length; count++)
    {
        if (count != length - 1)
            data[count] = IIC_Read_Byte_HAL(1); // ACK for all but last byte
        else
            data[count] = IIC_Read_Byte_HAL(0); // NACK for the last byte
    }
    IIC_Stop_HAL();
    return 1; // Success
}

int32_t IIC_WriteBytes_HAL(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint32_t length)
{
    uint32_t count = 0;
    IIC_Start_HAL();
    IIC_Send_Byte_HAL(dev_addr); // Send device address (write mode)
    if (IIC_Wait_Ack_HAL() == 1) { IIC_Stop_HAL(); return 0; }

    IIC_Send_Byte_HAL(reg_addr); // Send register address
    if (IIC_Wait_Ack_HAL() == 1) { IIC_Stop_HAL(); return 0; }

    for (count = 0; count < length; count++)
    {
        IIC_Send_Byte_HAL(data[count]);
        if (IIC_Wait_Ack_HAL() == 1) { IIC_Stop_HAL(); return 0; }
    }
    IIC_Stop_HAL();
    return 1; // Success
}
