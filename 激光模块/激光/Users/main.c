#include "sys.h"
#include "delay.h"
#include "led.h"
#include "uart1.h"
#include "uart2.h"
#include "laser.h"
#include "stdio.h"
#include "string.h" 

extern UART_HandleTypeDef uart2_handle; // 需要在这里声明uart2_handle

int main(void)
{
    HAL_Init();                         
    stm32_clock_init(RCC_PLL_MUL9);    
    led_init();                        
    Laser_Init(115200);
    
    // 初始化完成提示
    uart1_printf("System initialized\r\n");
    uart1_printf("Please send commands to control laser module\r\n");
    
    // 初始禁用UART2接收中断
    __HAL_UART_DISABLE_IT(&uart2_handle, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&uart2_handle, UART_IT_IDLE);
    
    while(1)
    { 
        // 检查串口1是否接收到上位机命令
        if(uart1_rx_len > 0)
        {
            // 处理上位机命令
            Laser_ProcessCommand(uart1_rx_buf, uart1_rx_len);
            
            // 如果接收到begin命令
            if (strstr((char*)uart1_rx_buf, "begin") != NULL) {
                uart1_printf("Received command: begin\r\n");
                
                // 清空UART2接收缓冲区和相关标志
                uart2_rx_clear(); 
                uart2_idle_flag = 0;
                uart2_frame_len = 0;
                
                // 设置标志，表示正在等待激光模块数据
                laser_measurement_active = 1;
                
                // 使能UART2接收中断
                __HAL_UART_ENABLE_IT(&uart2_handle, UART_IT_RXNE);
                __HAL_UART_ENABLE_IT(&uart2_handle, UART_IT_IDLE);
                
                // 发送开始测量指令到激光模块
                Laser_SendCommand(LASER_START_MEASURE);
                uart1_printf("Sent begin command to laser module\r\n");
            }
            // TODO: 添加其他上位机命令的处理
            // 例如：else if (strstr((char*)uart1_rx_buf, "status") != NULL) { ... }
            
            uart1_rx_clear(); // 清空UART1接收缓冲区
        }
        
        // 调用封装的激光模块处理函数
        Laser_HandleMeasurement();

        // TODO: 在这里添加其他传感器的数据处理函数调用
        // 例如: Ultrasonic_HandleMeasurement();
        // 例如: GPS_HandleData();
        
        delay_ms(10);  // 短暂延时
    }
}

