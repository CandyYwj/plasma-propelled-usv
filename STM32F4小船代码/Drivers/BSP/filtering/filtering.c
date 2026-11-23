#include "filtering.h"
#include <stdlib.h> // For qsort (或者你可以自己实现一个简单的排序算法)
#include <string.h> // For memcpy (可选，如果需要复制数组)

// qsort 的比较函数 (升序排序)
// 注意：这个函数是为 qsort 服务的，它的参数是 void 指针
// 需要根据 window_buffer 的实际数据类型进行转换
static int compare_uint16(const void *a, const void *b) {
    uint16_t val1 = *(const uint16_t *)a;
    uint16_t val2 = *(const uint16_t *)b;

    if (val1 < val2) return -1;
    if (val1 > val2) return 1;
    return 0;
}

// 如果不想用 stdlib.h 中的 qsort，可以自己实现一个简单的排序算法
// 例如：插入排序 (对于小窗口，效率尚可)
static void simple_sort_uint16(uint16_t arr[], uint8_t n) {
    uint8_t i, j;
    uint16_t key;
    for (i = 1; i < n; i++) {
        key = arr[i];
        j = i - 1;
        // 将大于 key 的元素向后移动
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j = j - 1;
        }
        arr[j + 1] = key;
    }
}


/**
 * @brief 初始化中位值滤波器结构体
 * @param filter: 指向滤波器结构体的指针
 * @retval None
 */
void MedianFilter_Init(MedianFilter_TypeDef *filter) {
    if (filter == NULL) {
        return;
    }
    filter->count = 0;
    filter->current_index = 0;
    // 可以选择性地将缓冲区清零，但非必需，因为会被新数据覆盖
    // for (uint8_t i = 0; i < MEDIAN_FILTER_SIZE; i++) {
    //     filter->window_buffer[i] = 0;
    // }
}

/**
 * @brief 更新滤波器，输入新值并返回中位数值
 * @param filter: 指向滤波器结构体的指针
 * @param new_value: 新的采样值
 * @retval uint16_t: 滤波后的中位数值
 */
uint16_t MedianFilter_Update(MedianFilter_TypeDef *filter, uint16_t new_value) {
    if (filter == NULL) {
        // 可以返回一个错误码或默认值
        return 0; // 或者 new_value, 或者上次的值等，取决于错误处理策略
    }

    // 将新值存入滑动窗口缓冲区
    filter->window_buffer[filter->current_index] = new_value;

    // 更新下一个写入位置的索引 (循环)
    filter->current_index = (filter->current_index + 1) % MEDIAN_FILTER_SIZE;

    // 更新有效数据计数，直到缓冲区填满
    if (filter->count < MEDIAN_FILTER_SIZE) {
        filter->count++;
    }

    // 临时缓冲区，用于排序，避免修改原始 window_buffer 的顺序（如果不想修改的话）
    // 或者直接在 window_buffer 上排序（如果允许修改）
    uint16_t sorted_buffer[MEDIAN_FILTER_SIZE];

    // 如果缓冲区未填满，只对有效数据进行排序和取中位数
    // 这种情况下，中位数可能不是那么“中”，但也是一种处理方式
    // 另一种方式是，在缓冲区未填满前，直接返回原始值或简单平均值
    if (filter->count < MEDIAN_FILTER_SIZE) {
        // 选项1: 对当前有效数据取中位数 (可能不是最佳，因为数据量少)
        memcpy(sorted_buffer, filter->window_buffer, filter->count * sizeof(uint16_t));
        // qsort(sorted_buffer, filter->count, sizeof(uint16_t), compare_uint16);
        simple_sort_uint16(sorted_buffer, filter->count); // 使用简单排序
        return sorted_buffer[filter->count / 2]; // 取中间值 (对于偶数个，取偏左或偏右一个)

        // 选项2: 在填满前返回原始值 (更简单，避免早期不稳定的中位数)
        // return new_value;
    } else {
        // 缓冲区已满，对整个窗口的数据进行排序
        // 复制到临时缓冲区进行排序，以保持 window_buffer 的原始插入顺序（可选）
        memcpy(sorted_buffer, filter->window_buffer, MEDIAN_FILTER_SIZE * sizeof(uint16_t));

        // 使用C标准库的qsort进行排序
        // qsort(sorted_buffer, MEDIAN_FILTER_SIZE, sizeof(uint16_t), compare_uint16);

        // 或者使用我们自己实现的简单排序 (对于小的MEDIAN_FILTER_SIZE，如5,7,9，性能尚可)
        simple_sort_uint16(sorted_buffer, MEDIAN_FILTER_SIZE);

        // 返回排序后数组的中间值
        // 由于 MEDIAN_FILTER_SIZE 是奇数，中间元素的索引是 (MEDIAN_FILTER_SIZE / 2)
        return sorted_buffer[MEDIAN_FILTER_SIZE / 2];
    }
}

