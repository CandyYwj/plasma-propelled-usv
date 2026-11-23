/**
 * @file    uart_dma_practice.c
 * @brief   模块一练习：UART DMA + 空闲中断 接收不定长数据
 *
 * 学习目标：
 *   1. 理解 UART 空闲中断的触发时机（总线安静后触发）
 *   2. 掌握"DMA停止 → 计算长度 → 重启DMA"的三步模式
 *   3. 理解 DMA 计数器与实际接收字节的关系
 *
 * 对应项目文件：Drivers/BSP/uart8/uart8.c
 *
 * 硬件（以 UART8 为例）：
 *   PE0 → RX，PE1 → TX
 *   DMA1_Stream6_Channel5 → UART8 RX
 *
 * 验证方法：
 *   用串口助手发 "hello"（不加回车），
 *   触发空闲中断后打印出接收到的字节数和内容
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

/* ====================================================================
 * 缓冲区和状态变量
 * ==================================================================== */
#define MY_UART_RX_BUF_SIZE  256

static UART_HandleTypeDef  my_huart;
static DMA_HandleTypeDef   my_hdma_rx;
static uint8_t  rx_buf[MY_UART_RX_BUF_SIZE];
static uint16_t rx_len = 0;
static uint8_t  rx_complete = 0;   /* 接收完成标志 */

/* ====================================================================
 * 初始化（手动配置，不依赖 CubeMX）
 * ==================================================================== */

/**
 * @brief  初始化 UART（以 UART8 PE0/PE1 为例）
 *
 * 三步模式：
 *   ① HAL_UART_Init() + HAL_UART_Receive_DMA() → 启动DMA接收
 *   ② 使能 UART_IT_IDLE 空闲中断
 *   ③ 中断里：停DMA → 算长度 → 设标志 → 重启DMA
 */
void MyUART_Init(uint32_t baudrate)
{
    /* 使能时钟 */
    __HAL_RCC_UART8_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* GPIO 配置：PE0=RX，PE1=TX，AF8（UART8）*/
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &gpio);

    /* DMA 配置：DMA1_Stream6_Channel5 */
    my_hdma_rx.Instance                 = DMA1_Stream6;
    my_hdma_rx.Init.Channel             = DMA_CHANNEL_5;
    my_hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;  /* 外设→内存 */
    my_hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;      /* 外设地址不递增 */
    my_hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;        /* 内存地址递增 */
    my_hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    my_hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    my_hdma_rx.Init.Mode                = DMA_NORMAL;             /* 普通模式（非循环）*/
    my_hdma_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&my_hdma_rx);

    /* 关联 DMA 句柄到 UART */
    __HAL_LINKDMA(&my_huart, hdmarx, my_hdma_rx);

    /* UART 配置 */
    my_huart.Instance          = UART8;
    my_huart.Init.BaudRate     = baudrate;
    my_huart.Init.WordLength   = UART_WORDLENGTH_8B;
    my_huart.Init.StopBits     = UART_STOPBITS_1;
    my_huart.Init.Parity       = UART_PARITY_NONE;
    my_huart.Init.Mode         = UART_MODE_TX_RX;
    my_huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    HAL_UART_Init(&my_huart);

    /* 启动 DMA 接收（非阻塞，DMA自动搬运）*/
    HAL_UART_Receive_DMA(&my_huart, rx_buf, MY_UART_RX_BUF_SIZE);

    /* 使能 IDLE 空闲中断 */
    __HAL_UART_ENABLE_IT(&my_huart, UART_IT_IDLE);

    /* 配置 NVIC 中断优先级 */
    HAL_NVIC_SetPriority(UART8_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(UART8_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

/* ====================================================================
 * 中断服务函数（放到 stm32f4xx_it.c 里）
 * ==================================================================== */

/**
 * @brief  UART8 中断服务函数
 *
 * 重点理解这三步：
 *   ① 停止 DMA（固定已接收的字节数，防止继续被DMA覆盖）
 *   ② 用"缓冲区大小 - DMA计数器剩余"算出实际接收字节数
 *   ③ 重启 DMA（为下一帧做好准备）
 */
void UART8_IRQHandler(void)
{
    /* HAL先处理UART相关中断 */
    HAL_UART_IRQHandler(&my_huart);

    /* 检测空闲中断标志 */
    if (__HAL_UART_GET_FLAG(&my_huart, UART_FLAG_IDLE) != RESET)
    {
        /* 步骤1：清除空闲中断标志（必须，否则持续触发）*/
        __HAL_UART_CLEAR_IDLEFLAG(&my_huart);

        /* 步骤2：停止 DMA，"冻结"当前接收位置 */
        HAL_UART_DMAStop(&my_huart);

        /* 步骤3：计算实际接收字节数
         * DMA 计数器初始 = MY_UART_RX_BUF_SIZE（还剩多少空间可填）
         * DMA 已搬运 = 初始值 - 当前计数器值 = 实际收到的字节数 */
        rx_len = MY_UART_RX_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(my_huart.hdmarx);

        /* 步骤4：设置完成标志，通知主循环处理 */
        rx_complete = 1;

        /* 步骤5：重启 DMA 接收，准备下一帧
         *        注意：rx_buf 会从头开始被覆盖，主循环需要在下一帧来前处理完 */
        HAL_UART_Receive_DMA(&my_huart, rx_buf, MY_UART_RX_BUF_SIZE);
    }
}

void DMA1_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&my_hdma_rx);
}

/* ====================================================================
 * 发送函数
 * ==================================================================== */
void MyUART_Send(const char *str)
{
    HAL_UART_Transmit(&my_huart, (uint8_t *)str, strlen(str), 100);
}

/* ====================================================================
 * 主循环处理（放到 main 的 while(1) 里）
 * ==================================================================== */
void MyUART_Handler(void)
{
    if (!rx_complete) return;
    rx_complete = 0;

    /* 确保字符串结尾（方便打印和字符串操作）*/
    rx_buf[rx_len] = '\0';

    /* 打印收到的内容 */
    char resp[64];
    sprintf(resp, "[RX %d bytes]: %s\r\n", rx_len, (char *)rx_buf);
    MyUART_Send(resp);

    /* 清空缓冲区（可选，下次DMA会覆盖）*/
    memset(rx_buf, 0, rx_len);
    rx_len = 0;
}

/* ====================================================================
 * 思考题：
 *
 * Q1：为什么要先 HAL_UART_DMAStop() 再读 DMA_GET_COUNTER？
 *     A：如果 DMA 还在运行，计数器随时在变化，读到的值不确定。
 *        停止后计数器固定，读到的才是"此刻已收了多少字节"。
 *
 * Q2：重启 DMA 后 rx_buf 会被覆盖吗？
 *     A：会！DMA 重启后从 rx_buf[0] 开始写。所以主循环必须在
 *        下一帧到来前完成对 rx_buf 的处理（拷贝或解析）。
 *        如果担心来不及，可以用"双缓冲"：准备两个 buf，交替使用。
 *
 * Q3：UART6（GPS）为什么不用 DMA + 空闲中断，而是逐字节中断？
 *     A：GPS 数据流是连续的（一帧接一帧），没有明显的帧间空闲时间，
 *        空闲中断不能可靠判帧。而且 GPS 需要检测"$GNRMC"帧头，
 *        逐字节检测更适合这种场景。
 *
 * Q4：UART8 用 DMA1_Stream6_Channel5，为什么不能用 Channel4？
 *     A：STM32F4 的 DMA 通道映射由硬件固定，每个外设只能接固定的
 *        通道（参考数据手册 DMA1 request mapping 表格）。
 *        UART8_RX 映射到 DMA1_Stream6_Channel5，这是硬件规定，无法更改。
 * ==================================================================== */
