#include "uart1.h"
#include "uart2.h"
#include "uart6.h"
#include "uart3.h"
#include "uart8.h"

UART_HandleTypeDef uart1_handle;                                            /* UART1句柄 */
DMA_HandleTypeDef uart1_dma_handle;                                        /* UART1 DMA句柄 */

uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];                                    /* UART1接收缓冲区 */
uint16_t uart1_rx_len = 0;                                                  /* UART1接收字符长度 */
uint8_t uart1_rx_complete_flag = 0;                                         /* UART1接收完成标志 */

/**
 * @brief       串口1初始化函数
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @retval      无
 */
void uart1_init(uint32_t baudrate)
{
    /* UART1 初始化设置 */
    uart1_handle.Instance = USART1;                                         /* USART1 */
    uart1_handle.Init.BaudRate = baudrate;                                  /* 波特率 */
    uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;                      /* 字长为8位数据格式 */
    uart1_handle.Init.StopBits = UART_STOPBITS_1;                           /* 一个停止位 */
    uart1_handle.Init.Parity = UART_PARITY_NONE;                            /* 无奇偶校验位 */
    uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;                      /* 无硬件流控 */
    uart1_handle.Init.Mode = UART_MODE_TX_RX;                               /* 收发模式 */
    HAL_UART_Init(&uart1_handle);                                           /* HAL_UART_Init()会使能UART1 */
    
    /* 启动DMA接收 */
    HAL_UART_Receive_DMA(&uart1_handle, uart1_rx_buf, UART1_RX_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&uart1_handle, UART_IT_IDLE);                      /* 使能UART1总线空闲中断 */
}

/**
 * @brief       UART底层初始化函数
 * @param       huart: UART句柄类型指针
 * @note        此函数会被HAL_UART_Init()调用
 *              完成时钟使能，引脚配置，中断配置
 * @retval      无
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_Initure;

    if (huart->Instance == USART1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();                                       /* 使能串口TX脚时钟 */
        __HAL_RCC_USART1_CLK_ENABLE();                                      /* 使能串口时钟 */
        __HAL_RCC_DMA2_CLK_ENABLE();                                        /* 使能DMA2时钟 */

        GPIO_Initure.Pin = GPIO_PIN_9;                                  /* 串口发送引脚号 */
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;                            /* 复用推挽输出 */
        GPIO_Initure.Pull = GPIO_PULLUP;                                /* 上拉 */
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_Initure.Alternate = GPIO_AF7_USART1;                       /* 复用为USART1 */
        HAL_GPIO_Init(GPIOA, &GPIO_Initure);
                
        GPIO_Initure.Pin = GPIO_PIN_10;                                 /* 串口RX脚 模式设置 */
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;    
        HAL_GPIO_Init(GPIOA, &GPIO_Initure);                            /* 串口RX脚 必须设置成输入模式 */
        
        /* DMA配置 for UART1 RX */
        uart1_dma_handle.Instance = DMA2_Stream2;                           /* 使用DMA2 Stream2 */
        uart1_dma_handle.Init.Channel = DMA_CHANNEL_4;                      /* 使用通道4 */
        uart1_dma_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;             /* 外设到内存 */
        uart1_dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;                 /* 外设地址不自增 */
        uart1_dma_handle.Init.MemInc = DMA_MINC_ENABLE;                     /* 内存地址自增 */
        uart1_dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;    /* 外设数据宽度为8位 */
        uart1_dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;       /* 内存数据宽度为8位 */
        uart1_dma_handle.Init.Mode = DMA_CIRCULAR;                          /* 循环模式 */
        uart1_dma_handle.Init.Priority = DMA_PRIORITY_HIGH;                 /* 高优先级 */
        uart1_dma_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;              /* 禁用FIFO */
        uart1_dma_handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;      /* FIFO阈值 */
        uart1_dma_handle.Init.MemBurst = DMA_MBURST_SINGLE;                 /* 内存突发传输 */
        uart1_dma_handle.Init.PeriphBurst = DMA_PBURST_SINGLE;              /* 外设突发传输 */
        
        HAL_DMA_Init(&uart1_dma_handle);                                    /* 初始化DMA */
        
        /* 将UART1的DMA句柄与UART1句柄关联 */
        __HAL_LINKDMA(huart, hdmarx, uart1_dma_handle);
        
        /* 配置DMA中断 for UART1 RX */
        HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 3, 3);                      /* 设置DMA中断优先级 */
        HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);                              /* 使能DMA中断 */
        
        HAL_NVIC_EnableIRQ(USART1_IRQn);                                    /* 使能USART1中断通道 */
        HAL_NVIC_SetPriority(USART1_IRQn, 3, 3);                            /* 组2，最低优先级:抢占优先级3，子优先级3 */
    }
    else if (huart->Instance == USART2)
    {
        GPIO_InitTypeDef gpio_init_struct_uart2;
        
        /* 使能GPIO端口时钟 (UART2在GPIOA，引脚PA2/TX, PA3/RX) */
        __HAL_RCC_GPIOA_CLK_ENABLE(); 
        __HAL_RCC_USART2_CLK_ENABLE();                                      /* 使能UART2时钟 */
        
        /* 使能DMA时钟 (UART2 RX 使用 DMA1 Stream5 Channel 4) */
        __HAL_RCC_DMA1_CLK_ENABLE();                                       

        /* UART2引脚配置 */
        gpio_init_struct_uart2.Pin = GPIO_PIN_2;                                  /* UART2 TX引脚 (PA2) */
        gpio_init_struct_uart2.Mode = GPIO_MODE_AF_PP;                            /* 复用推挽输出 */
        gpio_init_struct_uart2.Pull = GPIO_PULLUP;                                /* 上拉 */
        gpio_init_struct_uart2.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct_uart2.Alternate = GPIO_AF7_USART2;                       /* 复用为UART2 */
        HAL_GPIO_Init(GPIOA, &gpio_init_struct_uart2);
                
        gpio_init_struct_uart2.Pin = GPIO_PIN_3;                                 /* UART2 RX引脚 (PA3) */
        gpio_init_struct_uart2.Mode = GPIO_MODE_AF_PP;    
        gpio_init_struct_uart2.Pull = GPIO_PULLUP;                                /* 上拉 */
        gpio_init_struct_uart2.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct_uart2.Alternate = GPIO_AF7_USART2;                       /* 复用为UART2 */
        HAL_GPIO_Init(GPIOA, &gpio_init_struct_uart2);
        
        /* DMA配置 for UART2 RX */
        hdma_uart2_rx.Instance = DMA1_Stream5;                           /* 使用DMA1 Stream5 */
        hdma_uart2_rx.Init.Channel = DMA_CHANNEL_4;                      /* 使用通道4 */
        hdma_uart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;             /* 外设到内存 */
        hdma_uart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;                 /* 外设地址不自增 */
        hdma_uart2_rx.Init.MemInc = DMA_MINC_ENABLE;                     /* 内存地址自增 */
        hdma_uart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;    /* 外设数据宽度为8位 */
        hdma_uart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;       /* 内存数据宽度为8位 */
        hdma_uart2_rx.Init.Mode = DMA_CIRCULAR;                          /* 循环模式 */
        hdma_uart2_rx.Init.Priority = DMA_PRIORITY_HIGH;                 /* 高优先级 */
        hdma_uart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;              /* 禁用FIFO */
        hdma_uart2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;      /* FIFO阈值 */
        hdma_uart2_rx.Init.MemBurst = DMA_MBURST_SINGLE;                 /* 内存突发传输 */
        hdma_uart2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;              /* 外设突发传输 */
        
        HAL_DMA_Init(&hdma_uart2_rx);                                    /* 初始化DMA */
        
        /* 将UART2的DMA句柄与UART2句柄关联 */
        __HAL_LINKDMA(huart, hdmarx, hdma_uart2_rx);
        
        /* 配置DMA中断 for UART2 RX */
        HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);                      /* 设置DMA中断优先级 */
        HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);                              /* 使能DMA中断 */
        
        /* 配置UART2中断 (主要用于空闲中断) */
        HAL_NVIC_SetPriority(USART2_IRQn, 3, 1);                            /* 设置UART中断优先级 */
        HAL_NVIC_EnableIRQ(USART2_IRQn);                                    /* 使能UART2中断通道 */
    }
    else if(huart->Instance==USART6)
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN USART6_MspInit 0 */

	/* USER CODE END USART6_MspInit 0 */
	/* Peripheral clock enable */
	__HAL_RCC_USART6_CLK_ENABLE();

	__HAL_RCC_GPIOC_CLK_ENABLE();
	/**USART6 GPIO Configuration
	PC6     ------> USART6_TX
	PC7     ------> USART6_RX
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART6; // AF8 for USART6 on PC6/PC7
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* USART6 interrupt Init */
	HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART6_IRQn);
	/* USER CODE BEGIN USART6_MspInit 1 */

	/* USER CODE END USART6_MspInit 1 */
	}
    else if (huart->Instance == USART3)
    {
        GPIO_InitTypeDef gpio_init_struct_uart3;

        /* 使能GPIO端口时钟 (USART3在GPIOD，引脚PD8/TX, PD9/RX) */
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();                                      /* 使能USART3时钟 */

        /* 使能DMA时钟 (USART3 RX 使用 DMA1 Stream1 Channel 4) */
        __HAL_RCC_DMA1_CLK_ENABLE();

        /* USART3引脚配置 */
        gpio_init_struct_uart3.Pin = GPIO_PIN_8;                                  /* USART3 TX引脚 (PD8) */
        gpio_init_struct_uart3.Mode = GPIO_MODE_AF_PP;                            /* 复用推挽输出 */
        gpio_init_struct_uart3.Pull = GPIO_PULLUP;                                /* 上拉 */
        gpio_init_struct_uart3.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct_uart3.Alternate = GPIO_AF7_USART3;                       /* 复用为USART3 */
        HAL_GPIO_Init(GPIOD, &gpio_init_struct_uart3);

        gpio_init_struct_uart3.Pin = GPIO_PIN_9;                                 /* USART3 RX引脚 (PD9) */
        gpio_init_struct_uart3.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct_uart3.Pull = GPIO_PULLUP;                                /* 上拉 */
        gpio_init_struct_uart3.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct_uart3.Alternate = GPIO_AF7_USART3;                       /* 复用为USART3 */
        HAL_GPIO_Init(GPIOD, &gpio_init_struct_uart3);

        /* DMA配置 for USART3 RX */
        // 注意：hdma_uart3_rx句柄已经在uart3.c中全局定义
        hdma_uart3_rx.Instance = DMA1_Stream1;                           /* 使用DMA1 Stream1 */
        hdma_uart3_rx.Init.Channel = DMA_CHANNEL_4;                      /* 使用通道4 */
        hdma_uart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;             /* 外设到内存 */
        hdma_uart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;                 /* 外设地址不自增 */
        hdma_uart3_rx.Init.MemInc = DMA_MINC_ENABLE;                     /* 内存地址自增 */
        hdma_uart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;    /* 外设数据宽度为8位 */
        hdma_uart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;       /* 内存数据宽度为8位 */
        hdma_uart3_rx.Init.Mode = DMA_CIRCULAR;                          /* 循环模式 */
        hdma_uart3_rx.Init.Priority = DMA_PRIORITY_HIGH;                 /* 高优先级 */
        hdma_uart3_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;              /* 启用FIFO */
        hdma_uart3_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL; /* FIFO阈值设为半满 */
        hdma_uart3_rx.Init.MemBurst = DMA_MBURST_SINGLE;                 /* 内存突发传输 */
        hdma_uart3_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;              /* 外设突发传输 */

        HAL_DMA_Init(&hdma_uart3_rx);                                    /* 初始化DMA */

        /* 将USART3的DMA句柄与USART3句柄关联 */
        __HAL_LINKDMA(huart, hdmarx, hdma_uart3_rx);

        /* 配置DMA中断 for USART3 RX */
        HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);                      /* 设置DMA中断优先级为最高 */
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);                              /* 使能DMA中断 */

        /* 配置USART3中断 (主要用于空闲中断) */
        HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);                            /* 设置UART中断优先级为最高 */
        HAL_NVIC_EnableIRQ(USART3_IRQn);                                    /* 使能USART3中断通道 */
    }
    else if (huart->Instance == UART8)
    {
        GPIO_InitTypeDef gpio_init_struct_uart8;

        /* 使能GPIO端口时钟 (UART8在GPIOE，引脚PE1/TX, PE0/RX) */
        __HAL_RCC_GPIOE_CLK_ENABLE();
        __HAL_RCC_UART8_CLK_ENABLE();                                     /* 使能UART8时钟 */

        /* 使能DMA时钟 (UART8 RX 使用 DMA1 Stream6 Channel 5) */
        __HAL_RCC_DMA1_CLK_ENABLE();

        /* UART8引脚配置 */
        gpio_init_struct_uart8.Pin = GPIO_PIN_1;                                  /* UART8 TX引脚 (PE1) */
        gpio_init_struct_uart8.Mode = GPIO_MODE_AF_PP;                            /* 复用推挽输出 */
        gpio_init_struct_uart8.Pull = GPIO_PULLUP;                                /* 上拉 */
        gpio_init_struct_uart8.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct_uart8.Alternate = GPIO_AF8_UART8;                       /* 复用为UART8 */
        HAL_GPIO_Init(GPIOE, &gpio_init_struct_uart8);

        gpio_init_struct_uart8.Pin = GPIO_PIN_0;                                 /* UART8 RX引脚 (PE0) */
        gpio_init_struct_uart8.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct_uart8.Pull = GPIO_PULLUP;                                /* 上拉 */
        gpio_init_struct_uart8.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct_uart8.Alternate = GPIO_AF8_UART8;                       /* 复用为UART8 */
        HAL_GPIO_Init(GPIOE, &gpio_init_struct_uart8);

        /* DMA配置 for UART8 RX */
        hdma_uart8_rx.Instance = DMA1_Stream6;                           /* 使用DMA1 Stream6 */
        hdma_uart8_rx.Init.Channel = DMA_CHANNEL_5;                      /* 使用通道5 */
        hdma_uart8_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;             /* 外设到内存 */
        hdma_uart8_rx.Init.PeriphInc = DMA_PINC_DISABLE;                 /* 外设地址不自增 */
        hdma_uart8_rx.Init.MemInc = DMA_MINC_ENABLE;                     /* 内存地址自增 */
        hdma_uart8_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;    /* 外设数据宽度为8位 */
        hdma_uart8_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;       /* 内存数据宽度为8位 */
        hdma_uart8_rx.Init.Mode = DMA_CIRCULAR;                          /* 循环模式 */
        hdma_uart8_rx.Init.Priority = DMA_PRIORITY_HIGH;                 /* 高优先级 */
        hdma_uart8_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;              /* 禁用FIFO */
        hdma_uart8_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;      /* FIFO阈值 */
        hdma_uart8_rx.Init.MemBurst = DMA_MBURST_SINGLE;                 /* 内存突发传输 */
        hdma_uart8_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;              /* 外设突发传输 */

        if (HAL_DMA_Init(&hdma_uart8_rx) != HAL_OK)
        {
            Error_Handler();
        }

        /* 将UART8的DMA句柄与UART8句柄关联 */
        __HAL_LINKDMA(huart, hdmarx, hdma_uart8_rx);

        /* 配置DMA中断 for UART8 RX */
        HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);                      /* 设置DMA中断优先级为最高 */
        HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);                              /* 使能DMA中断 */

        /* 配置UART8中断 (主要用于空闲中断) */
        HAL_NVIC_SetPriority(UART8_IRQn, 0, 0);                            /* 设置UART中断优先级为最高 */
        HAL_NVIC_EnableIRQ(UART8_IRQn);                                    /* 使能UART8中断通道 */
        
        /* 发送调试信息 */
        uart8_send_string("UART8 MSP Init completed\r\n");
    }
}

/**
 * @brief       UART1接收缓冲区清除
 * @param       无
 * @retval      无
 */
void uart1_rx_clear(void)
{
    memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));                          /* 清空接收缓冲区 */
    uart1_rx_len = 0;                                                       /* 接收计数器清零 */
}

/**
 * @brief       通过UART1发送数据
 * @param       data: 数据缓冲区指针
 * @param       len: 数据长度
 * @retval      无
 */
void uart1_send_data(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&uart1_handle, data, len, 1000); // 使用HAL库发送数据，超时时间1000ms
}

/**
 * @brief       通过UART1接收数据
 * @param       buffer: 接收数据存放的缓冲区
 * @param       len: 指向存储接收到数据长度的变量
 * @retval      接收状态，0表示成功获取到数据，其他值表示无数据或错误
 */
uint8_t uart1_receive_data(uint8_t *buffer, uint16_t *len)
{
    if (uart1_get_rx_complete_flag())
    {
        *len = uart1_get_rx_len();
        memcpy(buffer, uart1_get_rx_buf(), *len); // 将接收缓冲区的数据拷贝到用户提供的缓冲区
        uart1_clear_rx_complete_flag();
        uart1_rx_clear(); // 清空内部接收缓冲区和长度计数
        HAL_UART_Receive_DMA(&uart1_handle, uart1_rx_buf, UART1_RX_BUF_SIZE); // 重新启动DMA接收
        return 0; // 成功获取到数据
    }
    return 1; // 无数据接收完成
}

/**
 * @brief       串口1中断服务函数
 * @note        在此使用接收中断及空闲中断，实现不定长数据收发
 * @param       无
 * @retval      无
 */
void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&uart1_handle, UART_FLAG_IDLE) != RESET)        /* 获取接收空闲中断标志位是否被置位 */
    {
        __HAL_UART_CLEAR_IDLEFLAG(&uart1_handle);                           /* 清除UART总线空闲中断 */
        
        /* 计算接收到的数据长度 */
        uart1_rx_len = UART1_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(uart1_handle.hdmarx);
        
        /* 设置接收完成标志 */
        uart1_rx_complete_flag = 1;
        
        /* 重新启动DMA接收 (这里只停止DMA，等待uart1_receive_data调用时重新启动) */
        HAL_UART_DMAStop(&uart1_handle);
    }
}

/* DMA中断服务函数 */
void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&uart1_dma_handle);
}

/* 获取接收完成标志 */
uint8_t uart1_get_rx_complete_flag(void)
{
    return uart1_rx_complete_flag;
}

/* 清除接收完成标志 */
void uart1_clear_rx_complete_flag(void)
{
    uart1_rx_complete_flag = 0;
}

/* 获取接收到的数据长度 */
uint16_t uart1_get_rx_len(void)
{
    return uart1_rx_len;
}

/* 获取接收缓冲区指针 */
uint8_t* uart1_get_rx_buf(void)
{
    return uart1_rx_buf;
}


