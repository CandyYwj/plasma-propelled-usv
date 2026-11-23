#ifndef __FILTERING_H__
#define __FILTERING_H__

#include <stdint.h> // For standard integer types like uint16_t, int16_t etc.

// 定义滤波器窗口大小 (必须是奇数，便于取中位数)
// 你可以根据实际需求调整这个值，例如 3, 5, 7, 9, 11 ...
#define MEDIAN_FILTER_SIZE 5

// 滤波器结构体
typedef struct {
    // 根据你的数据类型来定义 window_buffer 的类型
    // 如果是ADC采样值 (0-4095)，用 uint16_t
    // 如果是带符号的传感器数据，用 int16_t 或 int32_t
    // 这里以 uint16_t 为例
    uint16_t window_buffer[MEDIAN_FILTER_SIZE]; // 滑动窗口缓冲区
    uint8_t count;          // 当前缓冲区中有效数据的数量 (用于初始填充阶段)
    uint8_t current_index;  // 指向缓冲区中下一个要被替换的位置 (循环使用)
} MedianFilter_TypeDef;

// 函数声明
void MedianFilter_Init(MedianFilter_TypeDef *filter);
uint16_t MedianFilter_Update(MedianFilter_TypeDef *filter, uint16_t new_value);

#endif // MEDIAN_FILTER_H


