// 包含GPS模块相关的头文件
#include "gps.h"
#include <string.h> // 用于 memset, memcpy, strstr
#include <stdio.h>  // 用于 sprintf
#include <stdlib.h> // 用于 atof
#include <stdbool.h> // 用于 bool 类型

// 定义全局的Save_Data结构体
_SaveData Save_Data; 

// 错误日志函数
void ErrorLog(UART_HandleTypeDef* huart, int num)
{
    char error_msg[20]; // 错误消息缓冲区
    // 格式化错误消息
    sprintf(error_msg, "ERROR%d\r\n", num);
    // 尝试通过传入的UART句柄发送错误消息，使用超时，避免如果串口本身有问题导致死循环
    HAL_UART_Transmit(huart, (uint8_t*)error_msg, strlen(error_msg), 100); // 发送错误消息，100ms超时

	while (1)
	{
        // 在这里可以添加其他错误指示，例如让CPU进入低功耗模式或者进行复位
	}
}

// 解析GPS数据缓冲区
void ParseGpsBuffer(const char* data, uint16_t len, UART_HandleTypeDef* huart)
{
	char *subString; // 子字符串指针
	char *subStringNext; // 下一个子字符串指针
	char i = 0; // 循环计数器

    // 确保Save_Data.GPS_Buffer有足够的空间，并复制传入的数据
    memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);
    memcpy(Save_Data.GPS_Buffer, data, (len < GPS_Buffer_Length) ? len : GPS_Buffer_Length - 1); // 确保不超过缓冲区长度
    Save_Data.GPS_Buffer[len] = '\0'; // 确保字符串以空字符结尾

    Save_Data.isGetData = true; // 设置数据接收标志，表示有新数据待解析

	if (Save_Data.isGetData) // 如果接收到完整的数据帧
	{
		Save_Data.isGetData = false; // 清除数据接收标志
        // 在此模块中不直接打印原始数据，交由PrintGpsBuffer处理
		// printf("**************\r\n");
		// printf("%s",Save_Data.GPS_Buffer);
		for (i = 0 ; i <= 6 ; i++) // 遍历GNRMC语句的字段
		{
			if (i == 0) // 跳过语句类型字段
			{
				if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL)
					ErrorLog(huart, 1);	// 未找到第一个逗号，数据格式错误
			}
			else
			{
				subString++; // 移动到逗号后一个位置
				if ((subStringNext = strstr(subString, ",")) != NULL) // 查找下一个逗号
				{
					char usefullBuffer[2]; // 用于存储有效性标识
					switch(i)
					{
						case 1:memcpy(Save_Data.UTCTime, subString, subStringNext - subString);break;	// 提取UTC时间
						case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break;	// 提取有效性标识
						case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);break;	// 提取纬度信息
						case 4:memcpy(Save_Data.N_S, subString, subStringNext - subString);break;	// 提取N/S标识
						case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);break;	// 提取经度信息
						case 6:memcpy(Save_Data.E_W, subString, subStringNext - subString);break;	// 提取E/W标识

						default:break;
					}
					subString = subStringNext; // 更新子字符串指针到下一个字段的起始位置
					Save_Data.isParseData = true; // 设置数据已解析标志
					if(usefullBuffer[0] == 'A') // 判断GPS数据是否有效
						Save_Data.isUsefull = true; // 数据有效
					else if(usefullBuffer[0] == 'V')
						Save_Data.isUsefull = false; // 数据无效
				}
				else
				{
					ErrorLog(huart, 2);	// 未找到下一个逗号，数据格式错误
				}
			}
		}
	}
}

// GPS原始坐标转换为度格式
double Convert_to_degrees(char* data)
{
	double temp_data = atof(data); // 将字符串转换为浮点数
	int degree = (int)(temp_data / 100); // 提取度部分
	double f_degree = (temp_data / 100.0 - degree)*100/60.0; // 计算分的小数部分并转换为度
	double result = degree + f_degree; // 度加上转换后的分得到最终的度格式
	return result;
}

// 打印解析后的GPS数据
void PrintGpsBuffer(UART_HandleTypeDef* huart) // 接受UART句柄作为参数
{
	double f_latitude = 0.0; // 浮点格式的纬度
	double f_longitude = 0.0; // 浮点格式的经度
	char output_buf[100]; // 用于格式化输出字符串的缓冲区

	if (Save_Data.isParseData) // 如果数据已解析
	{
		Save_Data.isParseData = false; // 清除数据解析标志

        // 格式化并发送UTC时间
        sprintf(output_buf, "Save_Data.UTCTime = %s\r\n", Save_Data.UTCTime);
        HAL_UART_Transmit(huart, (uint8_t*)output_buf, strlen(output_buf), HAL_MAX_DELAY);

		if(Save_Data.isUsefull) // 如果GPS数据有效
		{
			Save_Data.isUsefull = false; // 清除数据有效标志

            // 格式化并发送纬度
			f_latitude = Convert_to_degrees(Save_Data.latitude); // 转换为度格式
            sprintf(output_buf, "Save_Data.latitude = %lf%s\r\n", f_latitude, Save_Data.N_S);
            HAL_UART_Transmit(huart, (uint8_t*)output_buf, strlen(output_buf), HAL_MAX_DELAY);

            // 格式化并发送南北标识
            sprintf(output_buf, "Save_Data.N_S = %s\r\n", Save_Data.N_S);
            HAL_UART_Transmit(huart, (uint8_t*)output_buf, strlen(output_buf), HAL_MAX_DELAY);

            // 格式化并发送经度
			f_longitude = Convert_to_degrees(Save_Data.longitude); // 转换为度格式
            sprintf(output_buf, "Save_Data.longitude = %lf%s\r\n", f_longitude, Save_Data.E_W);
            HAL_UART_Transmit(huart, (uint8_t*)output_buf, strlen(output_buf), HAL_MAX_DELAY);

            // 格式化并发送东西标识
            sprintf(output_buf, "Save_Data.E_W = %s\r\n", Save_Data.E_W);
            HAL_UART_Transmit(huart, (uint8_t*)output_buf, strlen(output_buf), HAL_MAX_DELAY);
		}
		else // 如果GPS数据无效
		{
            // 发送"GPS DATA is not usefull!"消息
            char not_useful_msg[] = "GPS DATA is not usefull!\r\n";
            HAL_UART_Transmit(huart, (uint8_t*)not_useful_msg, strlen(not_useful_msg), HAL_MAX_DELAY);
		}

        // 发送分隔线
        char separator[] = "**************\r\n";
        HAL_UART_Transmit(huart, (uint8_t*)separator, strlen(separator), HAL_MAX_DELAY);

        // 发送原始GPS数据（GNRMC语句），以匹配之前的输出格式
        HAL_UART_Transmit(huart, (uint8_t*)Save_Data.GPS_Buffer, strlen(Save_Data.GPS_Buffer), HAL_MAX_DELAY);
	}
}

// 清空Save_Data结构体
void clrStruct()
{
    Save_Data.isGetData = false;
    Save_Data.isParseData = false;
    Save_Data.isUsefull = false;
    memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length); // 清空GPS原始数据缓冲区
    memset(Save_Data.UTCTime, 0, UTCTime_Length); // 清空UTC时间
    memset(Save_Data.latitude, 0, latitude_Length); // 清空纬度
    memset(Save_Data.N_S, 0, N_S_Length); // 清空南北标识
    memset(Save_Data.longitude, 0, longitude_Length); // 清空经度
    memset(Save_Data.E_W, 0, E_W_Length); // 清空东西标识
}

void GPS_RxCpltCallback(const char* data, uint16_t len, void* userData)
{
    // Cast userData to UART_HandleTypeDef*
    UART_HandleTypeDef* huart = (UART_HandleTypeDef*)userData;
    // Call the GPS module's parse function with the received data and UART handle
    ParseGpsBuffer(data, len, huart);
}

// 通过UART3打印GPS数据到上位机
void PrintGpsBufferToUART3(UART_HandleTypeDef* huart3)
{
    double f_latitude = 0.0; // 浮点格式的纬度
    double f_longitude = 0.0; // 浮点格式的经度
    char output_buf[100]; // 用于格式化输出字符串的缓冲区

    if (Save_Data.isParseData) // 如果数据已解析
    {
        Save_Data.isParseData = false; // 清除数据解析标志

        // 格式化并发送UTC时间
        sprintf(output_buf, "GPS_TIME=%s\r\n", Save_Data.UTCTime);
        HAL_UART_Transmit(huart3, (uint8_t*)output_buf, strlen(output_buf), 100);

        if(Save_Data.isUsefull) // 如果GPS数据有效
        {
            Save_Data.isUsefull = false; // 清除数据有效标志

            // 格式化并发送纬度
            f_latitude = Convert_to_degrees(Save_Data.latitude); // 转换为度格式
            sprintf(output_buf, "GPS_LAT=%.6f%s\r\n", f_latitude, Save_Data.N_S);
            HAL_UART_Transmit(huart3, (uint8_t*)output_buf, strlen(output_buf), 100);

            // 格式化并发送经度
            f_longitude = Convert_to_degrees(Save_Data.longitude); // 转换为度格式
            sprintf(output_buf, "GPS_LON=%.6f%s\r\n", f_longitude, Save_Data.E_W);
            HAL_UART_Transmit(huart3, (uint8_t*)output_buf, strlen(output_buf), 100);
        }
        else // 如果GPS数据无效
        {
            // 发送GPS数据无效消息
            char not_useful_msg[] = "GPS_DATA=INVALID\r\n";
            HAL_UART_Transmit(huart3, (uint8_t*)not_useful_msg, strlen(not_useful_msg), 100);
        }
    }
}

