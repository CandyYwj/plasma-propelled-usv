#ifndef __IOI2C_H__
#define __IOI2C_H__

#include "stm32f4xx_hal.h" // Main HAL include for F4 series
#include <stdint.h>        // For standard integer types like uint8_t

// Define GPIO Pins for I2C (Using GPIOB, Pin 10 for SCL, Pin 11 for SDA as in original)
// Make sure these pins are not used by other peripherals in your F429 project.
#define IIC_SCL_GPIO_Port   GPIOB
#define IIC_SCL_Pin         GPIO_PIN_6  

#define IIC_SDA_GPIO_Port   GPIOB
#define IIC_SDA_Pin         GPIO_PIN_7

// Function Prototypes
void IIC_Init_HAL(void);
void IIC_Start_HAL(void);
void IIC_Stop_HAL(void);
void IIC_Send_Byte_HAL(uint8_t txd);
uint8_t IIC_Read_Byte_HAL(uint8_t ack); // Changed 'unsigned char' to 'uint8_t'
uint8_t IIC_Wait_Ack_HAL(void);
void IIC_Ack_HAL(void);
void IIC_NAck_HAL(void);

// Higher-level read/write functions (names changed slightly to avoid conflict if old files are around)
int32_t IIC_ReadBytes_HAL(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint32_t length);
int32_t IIC_WriteBytes_HAL(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint32_t length);

#endif // __IOI2C_HAL_H
