#include "IMU.h"         // 你的 IMU 头文件 (例如 imu_hal.h)
#include "IOI2C.h"       // 你的 I2C HAL 库头文件 (例如 ioi2c_hal.h)
#include "wit_c_sdk.h"   // Wit SDK 主头文件，它应该负责包含 AHRSREG.h (或 REG.h)
                         // 如果 wit_c_sdk.h 没有包含 AHRSREG.h，你可能需要在这里显式包含：
                         // #include "AHRSREG.h" // 或者你的寄存器定义文件名
#include "uart3.h"       // 添加UART3头文件
#include <stdio.h>       // 为了使用sprintf

// sReg 数组和所有寄存器地址宏 (AX, AY, AZ, GX, GY, GZ, HX, HY, HZ, Roll, Pitch, Yaw 等)
// 都应该通过包含 wit_c_sdk.h (及其依赖的 AHRSREG.h/REG.h) 来获得。
// 不要在这里重新定义它们或使用 extern 声明 sReg (除非 wit_c_sdk.h 中没有 extern sReg)。

// 假设你的 IMU.h 中定义了这些：
// #define ACC_UPDATE   0x01
// #define GYRO_UPDATE  0x02
// #define ANGLE_UPDATE 0x04
// #define MAG_UPDATE   0x08
// #define READ_UPDATE  0x80
// 以及 IMU_Data_t_HAL 结构体

static volatile char s_cDataUpdate_HAL = 0;
static char imu_str[128]; // 用于存储格式化后的字符串

// Millisecond delay function for Wit SDK
static void DelayMs_HAL(uint16_t usMs)
{
    HAL_Delay(usMs);
}

void IMU_Init_HAL(void)
{
    IIC_Init_HAL();
    WitInit(WIT_PROTOCOL_I2C, 0x50);
    WitI2cFuncRegister(IIC_WriteBytes_HAL, IIC_ReadBytes_HAL);
    WitRegisterCallBack(IMU_DataCallback);
    WitDelayMsRegister(DelayMs_HAL);
}

// 回调函数，由 Wit SDK 在数据更新后调用
// uiReg: 本次读取的起始寄存器地址
// uiRegNum: 本次读取的寄存器数量
void IMU_DataCallback(uint32_t uiReg, uint32_t uiRegNum)
{
    uint32_t endReg = uiReg + uiRegNum - 1;
    
    // 检查加速度数据是否在更新的范围内
    if ((uiReg <= AX && endReg >= AX) || 
        (uiReg <= AY && endReg >= AY) || 
        (uiReg <= AZ && endReg >= AZ) || 
        (uiReg >= AX && endReg <= AZ))
    {
        s_cDataUpdate_HAL |= ACC_UPDATE;
    }

    // 检查陀螺仪数据是否在更新的范围内
    if ((uiReg <= GX && endReg >= GX) || 
        (uiReg <= GY && endReg >= GY) || 
        (uiReg <= GZ && endReg >= GZ))
    {
        s_cDataUpdate_HAL |= GYRO_UPDATE;
    }

    // 检查磁力计数据是否在更新的范围内
    if ((uiReg <= HX && endReg >= HX) || 
        (uiReg <= HY && endReg >= HY) || 
        (uiReg <= HZ && endReg >= HZ))
    {
        s_cDataUpdate_HAL |= MAG_UPDATE;
    }

    // 检查角度数据是否在更新的范围内
    if ((uiReg <= Roll && endReg >= Roll) || 
        (uiReg <= Pitch && endReg >= Pitch) || 
        (uiReg <= Yaw && endReg >= Yaw))
    {
        s_cDataUpdate_HAL |= ANGLE_UPDATE;
    }

    if (uiRegNum > 0 && s_cDataUpdate_HAL == 0)
    {
        s_cDataUpdate_HAL |= READ_UPDATE;
    }
}

void IMU_ReadData_HAL(IMU_Data_t_HAL *imu_data)
{
    WitReadReg(AX, 12);
    HAL_Delay(5);
    
    if (s_cDataUpdate_HAL)
    {
        if (s_cDataUpdate_HAL & ACC_UPDATE)
        {
            imu_data->acc[0] = sReg[AX] / 32768.0f * 16.0f;
            imu_data->acc[1] = sReg[AY] / 32768.0f * 16.0f;
            imu_data->acc[2] = sReg[AZ] / 32768.0f * 16.0f;
        }
        
        if (s_cDataUpdate_HAL & GYRO_UPDATE)
        {
            imu_data->gyro[0] = sReg[GX] / 32768.0f * 2000.0f;
            imu_data->gyro[1] = sReg[GY] / 32768.0f * 2000.0f;
            imu_data->gyro[2] = sReg[GZ] / 32768.0f * 2000.0f;
        }
        
        if (s_cDataUpdate_HAL & ANGLE_UPDATE)
        {
            imu_data->angle[0] = sReg[Roll] / 32768.0f * 180.0f;
            imu_data->angle[1] = sReg[Pitch] / 32768.0f * 180.0f;
            imu_data->angle[2] = sReg[Yaw] / 32768.0f * 180.0f;
        }
        
        if (s_cDataUpdate_HAL & MAG_UPDATE)
        {
            imu_data->mag[0] = sReg[HX];
            imu_data->mag[1] = sReg[HY];
            imu_data->mag[2] = sReg[HZ];
        }

        s_cDataUpdate_HAL = 0;
    }
}

void IMU_PrintData_HAL(IMU_Data_t_HAL *imu_data)
{
    // 使用sprintf格式化数据到imu_str
    sprintf(imu_str, "Acc: %.3f %.3f %.3f g\r\n",
            imu_data->acc[0], imu_data->acc[1], imu_data->acc[2]);
    uart3_send_string(imu_str);
    
    sprintf(imu_str, "Gyro: %.3f %.3f %.3f dps\r\n",
            imu_data->gyro[0], imu_data->gyro[1], imu_data->gyro[2]);
    uart3_send_string(imu_str);
    
    sprintf(imu_str, "Angle: %.3f %.3f %.3f deg\r\n",
            imu_data->angle[0], imu_data->angle[1], imu_data->angle[2]);
    uart3_send_string(imu_str);
    
    sprintf(imu_str, "Mag: %d %d %d\r\n",
            imu_data->mag[0], imu_data->mag[1], imu_data->mag[2]);
    uart3_send_string(imu_str);
}
