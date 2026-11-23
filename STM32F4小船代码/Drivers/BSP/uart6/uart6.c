#include "uart6.h"
#include <string.h> // For memset, memcpy, strstr
#include <stdio.h>  // For printf (if needed, but we'll use HAL_UART_Transmit for output)
#include <stdlib.h> // For atof
#include <stdbool.h> // 新增：为了使用bool类型

// 明确定义USART接收缓冲区的最大长度，确保其在uart6.c中可用
#define USART_REC_LEN 500 

// Define global variables for USART6
UART_HandleTypeDef huart6;
uint8_t Uart6_Receive_buf[1]; // 用于通过中断接收单个字节的缓冲区
char USART6_RX_BUF[USART_REC_LEN]; // 用于存储接收到的GPS数据帧的缓冲区
uint16_t point1 = 0; // 接收缓冲区指针

// 声明一个静态变量来存储注册的回调函数
static UartRxCallback rxCallback = NULL;
// 声明一个静态变量来存储用户数据指针
static void* rxUserData = NULL;

// Function prototypes (also declared in uart6.h)
void MX_USART6_UART_Init(void);
void CLR_Buf(void);
uint8_t Hand(char *a);
void UART6_RegisterRxCallback(UartRxCallback callback, void* userData);

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600; // 波特率设置为9600
  huart6.Init.WordLength = UART_WORDLENGTH_8B; // 8位数据长度
  huart6.Init.StopBits = UART_STOPBITS_1; // 1位停止位
  huart6.Init.Parity = UART_PARITY_NONE; // 无奇偶校验
  huart6.Init.Mode = UART_MODE_TX_RX; // 收发模式
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE; // 无硬件流控制
  huart6.Init.OverSampling = UART_OVERSAMPLING_16; // 16倍过采样
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler(); // 初始化失败，调用错误处理函数
  }
  /* USER CODE BEGIN USART6_Init 2 */
  // 启动中断接收第一个字节
  HAL_UART_Receive_IT(&huart6, Uart6_Receive_buf, sizeof(Uart6_Receive_buf));
  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}


// UART Receive Complete Callback (Weakly defined in HAL, implemented here for USART6)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart)
{
    uint8_t Res; // 接收到的数据字节
    if(huart->Instance == USART6)
    {
        Res = Uart6_Receive_buf[0];

        // Check if the received character is the start of a new sentence '$'
        if(Res == '$')
        {
            point1 = 0; // 重置缓冲区指针
        }

        // Store the received character in the buffer
        USART6_RX_BUF[point1++] = Res;

        // Check if the buffer might contain a GNRMC sentence and is ending with '\n'
        // Assuming the critical part of the identifier is within the first few bytes
        if(USART6_RX_BUF[0] == '$' &&
           ( (USART6_RX_BUF[1] == 'G' && USART6_RX_BUF[2] == 'N' && USART6_RX_BUF[3] == 'R' && USART6_RX_BUF[4] == 'M' && USART6_RX_BUF[5] == 'C') ||
             (USART6_RX_BUF[1] == 'G' && USART6_RX_BUF[2] == 'P' && USART6_RX_BUF[3] == 'R' && USART6_RX_BUF[4] == 'M' && USART6_RX_BUF[5] == 'C') ) &&
           Res == '\n')
        {
            // Complete GNRMC sentence received
            if (rxCallback != NULL) {
                rxCallback(USART6_RX_BUF, point1, rxUserData); // 通过回调函数传递接收到的数据和用户数据指针
            }
            point1 = 0; // Reset buffer pointer
            memset(USART6_RX_BUF, 0, USART_REC_LEN); // Clear receive buffer
        }

        // Prevent buffer overflow
        if(point1 >= USART_REC_LEN) point1 = USART_REC_LEN - 1; // Keep the last character to detect newline if it was missed earlier

        // Restart the interrupt reception for the next byte
        HAL_UART_Receive_IT(&huart6, Uart6_Receive_buf, sizeof(Uart6_Receive_buf));
    }
    // If there are other UARTs with enabled interrupts, their callbacks would be handled here or in their own implementations
}


// Clear the receive buffer
void CLR_Buf(void)
{
    memset(USART6_RX_BUF, 0, USART_REC_LEN);
    point1 = 0;
}

// Check if a substring exists in the receive buffer
uint8_t Hand(char *a)
{
    if(strstr(USART6_RX_BUF, a) != NULL)
        return 1;
    else
        return 0;
}

// 实现回调函数注册函数
void UART6_RegisterRxCallback(UartRxCallback callback, void* userData)
{
    rxCallback = callback;
    rxUserData = userData;
}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
//void HAL_UART_MspInit(UART_HandleTypeDef* huart)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(huart->Instance==USART6)
//  {
//  /* USER CODE BEGIN USART6_MspInit 0 */

//  /* USER CODE END USART6_MspInit 0 */
//    /* Peripheral clock enable */
//    __HAL_RCC_USART6_CLK_ENABLE();

//    __HAL_RCC_GPIOC_CLK_ENABLE();
//    /**USART6 GPIO Configuration
//    PC6     ------> USART6_TX
//    PC7     ------> USART6_RX
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF8_USART6; // AF8 for USART6 on PC6/PC7
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//    /* USART6 interrupt Init */
//    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(USART6_IRQn);
//  /* USER CODE BEGIN USART6_MspInit 1 */

//  /* USER CODE END USART6_MspInit 1 */
//  }
//}

/**
* @brief UART MSP De-Initialization
* This function frees the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}
