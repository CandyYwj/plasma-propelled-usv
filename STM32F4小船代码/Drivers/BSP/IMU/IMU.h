#ifndef __IMU_H__
#define __IMU_H__

#include "stm32f4xx_hal.h" // Main HAL include
#include "wit_c_sdk.h"     // Include your WitMotion SDK header
#include <stdint.h>

// IMU数据结构体 (assuming this is compatible or defined by wit_c_sdk)
typedef struct {
    float acc[3];    // 加速度数据 (g)
    float gyro[3];   // 陀螺仪数据 (dps)
    float angle[3];  // 角度数据 (deg)
    int16_t mag[3];  // 磁力计数据
} IMU_Data_t_HAL; // Renamed slightly to avoid conflicts if old struct exists

// Function声明
void IMU_Init_HAL(void);
void IMU_ReadData_HAL(IMU_Data_t_HAL *imu_data);
void IMU_PrintData_HAL(IMU_Data_t_HAL *imu_data); // Assuming printf is retargeted

// Callback function declaration (name unchanged, as it's likely expected by the SDK)
void IMU_DataCallback(uint32_t uiReg, uint32_t uiRegNum); // Ensure this matches SDK expectation

// These defines are from your original IMU.c, assuming they are still needed
// and not part of wit_c_sdk.h
#define ACC_UPDATE   0x01
#define GYRO_UPDATE  0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE   0x08
#define READ_UPDATE  0x80

// These register defines like AX, GX, Roll, etc. are assumed to be in wit_c_sdk.h
// If not, you need to define them. Example:
// extern volatile int16_t sReg[]; // Declaration for sReg if it's global in wit_c_sdk
// #define AX    0x34 // Example, replace with actual values from SDK
// #define AY    0x35
// #define AZ    0x36
// #define GX    0x37
// #define GY    0x38
// #define GZ    0x39
// #define Roll  0x3d
// #define Pitch 0x3e
// #define Yaw   0x3f
// #define HX    0x3a // Example for magnetometer
// #define HY    0x3b
// #define HZ    0x3c

#endif // __IMU_HAL_H

