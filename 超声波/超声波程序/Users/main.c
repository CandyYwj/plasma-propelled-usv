#include "sys.h"
#include "delay.h"
#include "led.h"
#include "uart1.h"

int main(void)
{
    HAL_Init();                         /* 初始化HAL库 */
    stm32_clock_init(RCC_PLL_MUL9);     /* 设置时钟, 72Mhz */
    led_init();                         /* 初始化LED灯 */
    uart1_init(115200);

    while(1)
    { 

    }
}

