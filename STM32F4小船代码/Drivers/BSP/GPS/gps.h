#ifndef __GPS_H__
#define __GPS_H__

// 包含必要的HAL库头文件
#include <stdbool.h> // 为了使用bool类型
#include "sys.h" // 由于ErrorLog需要HAL库的UART句柄，暂时保留main.h

// 定义GPS数据解析相关的宏
#define GPS_Buffer_Length 100 // GPS原始数据缓冲区长度
#define UTCTime_Length 15     // UTC时间字符串长度
#define latitude_Length 15    // 纬度字符串长度
#define longitude_Length 16   // 经度字符串长度
#define N_S_Length 2          // 南北半球标识字符串长度
#define E_W_Length 2          // 东西半球标识字符串长度

// 保存解析后的GPS数据的结构体
typedef struct {
    char GPS_Buffer[GPS_Buffer_Length]; // GPS原始数据缓冲区
    char UTCTime[UTCTime_Length];       // UTC时间字符串
    char latitude[latitude_Length];     // 纬度字符串
    char N_S[N_S_Length];               // 南北半球标识字符串
    char longitude[longitude_Length];   // 经度字符串
    char E_W[E_W_Length];               // 东西半球标识字符串
    bool isGetData;   // 标志：是否接收到完整的GPS数据帧
    bool isParseData; // 标志：是否已解析GPS数据帧
    bool isUsefull;   // 标志：GPS数据是否有效（'A'状态）
} _SaveData;

extern _SaveData Save_Data; // 声明全局的Save_Data结构体

// GPS解析相关函数原型
void ParseGpsBuffer(const char* data, uint16_t len, UART_HandleTypeDef* huart); // 解析GPS数据缓冲区函数，接收原始数据、长度和UART句柄
void PrintGpsBuffer(UART_HandleTypeDef* huart); // 打印GPS数据缓冲区函数，通过指定UART发送
double Convert_to_degrees(char* data);     // GPS原始坐标转换为度格式函数
void clrStruct(void);                      // 清空Save_Data结构体函数
void ErrorLog(UART_HandleTypeDef* huart, int num); // 错误日志函数，接受UART句柄
void GPS_RxCpltCallback(const char* data, uint16_t len, void* userData);
void PrintGpsBufferToUART3(UART_HandleTypeDef* huart3); // 通过UART3打印GPS数据到上位机

#endif /* __GPS_H__ */
