/**
 * @file    median_filter_practice.c
 * @brief   模块四练习：中值滤波器（环形缓冲+插入排序）
 *
 * 学习目标：
 *   1. 理解中值滤波为何能消除偶发尖刺噪声
 *   2. 掌握环形缓冲区的实现（覆盖最老数据）
 *   3. 理解冷启动阶段的处理（数据不足窗口大小时）
 *   4. 对比中值滤波与均值滤波的适用场景
 *
 * 对应项目文件：Drivers/BSP/filtering/filtering.c
 *
 * 验证方法：
 *   喂入序列 [120, 118, 850, 119, 121]（850是噪声），
 *   中值滤波输出应为 120，而不是 850
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* ====================================================================
 * 参数定义
 * ==================================================================== */
#define MEDIAN_FILTER_SIZE  5    /* 窗口大小（奇数更好，中位置不歧义）*/

/* ====================================================================
 * 数据结构
 * ==================================================================== */
typedef struct {
    uint16_t window_buffer[MEDIAN_FILTER_SIZE]; /* 环形数据缓冲 */
    uint8_t  current_index;    /* 下次写入位置（环形） */
    uint8_t  count;            /* 已有有效数据个数（冷启动阶段 < SIZE）*/
} MedianFilter_TypeDef;

/* ====================================================================
 * 插入排序（不用 stdlib.h 的 qsort，嵌入式更安全）
 *
 * 原理：从第2个元素开始，向前找到正确位置插入
 * 时间：O(n²) — n=5时完全够用（只有5个元素，约10次比较）
 * ==================================================================== */
static void simple_sort_uint16(uint16_t arr[], uint8_t n)
{
    for (uint8_t i = 1; i < n; i++) {
        uint16_t key = arr[i];
        int8_t j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

/* ====================================================================
 * Step1：初始化
 * ==================================================================== */
void MedianFilter_Init_Practice(MedianFilter_TypeDef *f)
{
    f->count         = 0;
    f->current_index = 0;
    /* 缓冲区内容无需初始化，count=0会正确处理冷启动 */
}

/* ====================================================================
 * Step2：更新滤波器，返回中值
 * ==================================================================== */

/**
 * @brief  加入新样本，返回当前窗口的中值
 * @param  f          滤波器实例
 * @param  new_value  最新采样值
 * @retval 当前窗口的中位数
 *
 * 工作过程：
 *   1. 将新值写入环形缓冲区（覆盖最老数据）
 *   2. 更新写指针（循环）
 *   3. 复制缓冲区副本并排序（不改变原始顺序！）
 *   4. 取排好序的中间值
 *
 * 冷启动（count < SIZE）：
 *   窗口还没满，只对已有数据排序取中值
 *   优点：不用等满5个才开始输出，适合启动时快速响应
 */
uint16_t MedianFilter_Update_Practice(MedianFilter_TypeDef *f, uint16_t new_value)
{
    /* 步骤1：写入环形缓冲（覆盖最老数据）*/
    f->window_buffer[f->current_index] = new_value;
    f->current_index = (f->current_index + 1) % MEDIAN_FILTER_SIZE;  /* 循环递增 */

    /* 步骤2：更新有效数据计数（满了就不再增加）*/
    if (f->count < MEDIAN_FILTER_SIZE) f->count++;

    /* 步骤3：复制到临时缓冲区后排序（避免破坏原始环形顺序）*/
    uint16_t sorted[MEDIAN_FILTER_SIZE];
    memcpy(sorted, f->window_buffer, f->count * sizeof(uint16_t));
    simple_sort_uint16(sorted, f->count);

    /* 步骤4：返回中间值 */
    return sorted[f->count / 2];
    /* count=1: sorted[0]（唯一的值）
     * count=3: sorted[1]（第2个，即中位）
     * count=5: sorted[2]（第3个，即中位）*/
}

/* ====================================================================
 * Step3：对比测试
 * ==================================================================== */

/**
 * @brief  均值滤波（对比用）
 *
 * 均值滤波缺点：1个极端值拉偏整体均值
 * 例：[118, 119, 120, 121, 850] → 均值=(118+119+120+121+850)/5=265（失真！）
 *     中值=120（正确！）
 */
static uint16_t SimpleAverage(uint16_t *arr, uint8_t n)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) sum += arr[i];
    return (uint16_t)(sum / n);
}

/**
 * @brief  验证测试：喂入含噪声序列，对比中值vs均值
 *
 * 预期结果：
 *   中值滤波：逐渐稳定在 119~120 附近，不受 850 影响
 *   均值滤波：出现 850 时均值会被拉高到 ~230
 */
void MedianFilter_Test(void)
{
    MedianFilter_TypeDef filter;
    MedianFilter_Init_Practice(&filter);

    /* 模拟含噪声的传感器序列 */
    uint16_t raw_data[] = {120, 118, 850, 119, 121, 122, 117, 900, 120, 119};
    uint8_t  N = sizeof(raw_data) / sizeof(raw_data[0]);

    printf("=== 中值滤波 vs 均值滤波 对比 ===\r\n");
    printf("%-5s %-8s %-10s %-10s\r\n", "帧号", "原始值", "中值滤波", "说明");

    for (uint8_t i = 0; i < N; i++) {
        uint16_t filtered = MedianFilter_Update_Practice(&filter, raw_data[i]);
        const char *note = (raw_data[i] > 500) ? "<- 噪声帧" : "";
        printf("%-5d %-8d %-10d %s\r\n", i+1, raw_data[i], filtered, note);
    }

    printf("\r\n=== 窗口=[120,118,850,119,121] 均值vs中值 ===\r\n");
    uint16_t window[] = {120, 118, 850, 119, 121};
    printf("均值 = %d (被850拉偏)\r\n", SimpleAverage(window, 5));
    /* 排序后: [118,119,120,121,850], 中值=window[2]=120 */
    uint16_t sorted_copy[] = {120, 118, 850, 119, 121};
    simple_sort_uint16(sorted_copy, 5);
    printf("中值 = %d (正确)\r\n", sorted_copy[5/2]);
}

/* ====================================================================
 * 在 main 里的典型使用方式
 * ====================================================================

    MedianFilter_TypeDef ultrasonic_filter;
    MedianFilter_TypeDef laser_filter;

    MedianFilter_Init_Practice(&ultrasonic_filter);
    MedianFilter_Init_Practice(&laser_filter);

    // 在测量循环里：
    uint16_t raw_u = ultrasonic_get_distance();  // 原始超声波距离
    uint16_t raw_l = laser_get_distance();       // 原始激光距离

    uint16_t filtered_u = MedianFilter_Update_Practice(&ultrasonic_filter, raw_u);
    uint16_t filtered_l = MedianFilter_Update_Practice(&laser_filter, raw_l);

    // 用 filtered_u 和 filtered_l 上报，不用 raw

 * ==================================================================== */

/* ====================================================================
 * 思考题：
 *
 * Q1：为什么窗口大小选5而不是3或10？
 *     A：窗口越大，抗噪声越强，但响应越慢（真实距离变化时需要更多帧才更新）。
 *        窗口=3：可以去掉单帧噪声，响应快（3帧）
 *        窗口=5：可以去掉2帧连续噪声，响应适中（5帧）← 本项目选择
 *        窗口=10：抗噪强，但真实跳变要10帧才能体现
 *        对于运动中的无人船，5帧（约1秒）的距离稳定性比较合适。
 *
 * Q2：为什么排序时要"复制到sorted[]后排序"而不是直接对window_buffer排序？
 *     A：window_buffer 是环形缓冲区，其中数据顺序很重要（最老在current_index处）。
 *        如果直接排序，原始的时间顺序被破坏，下次更新时会写到错误位置，
 *        导致滤波器失效。因此必须复制后在副本上排序。
 *
 * Q3：中值滤波能处理"连续多帧"噪声吗？
 *     A：如果噪声持续帧数 > 窗口大小/2，中值就会被噪声值"占领"。
 *        例：窗口=5，连续3帧都是850 → sorted=[118,119,850,850,850]，中值=850（失败！）
 *        对策：加大窗口，或使用卡尔曼滤波等更复杂的方法。
 *
 * Q4：这个滤波器线程安全吗（如果有中断？）
 *     A：不是！MedianFilter_Update 修改多个变量，不是原子操作。
 *        如果传感器数据在中断里更新，主循环里读取，会有竞争条件。
 *        解决：在更新滤波器时禁用中断，或使用RTOS互斥锁保护。
 * ==================================================================== */
