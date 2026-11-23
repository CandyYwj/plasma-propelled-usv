#include "sysinc.h"
#include "ds18b20.h"


void timer2()
{	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		  PCout(13)= !PCout(13);
			printf("TMP =  %d", DS18B20_Get_Temp() );
	  	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}

int main(void){
	NVIC_init();
	SYSTICK_DelayInit();	    	 //延时函数初始化	  	  	
	 
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口
	
	UART_QuickInit(HW_UART1, 9600, 2, 2, ENABLE);
	
	while( DS18B20_Init() ){
		printf("未检测到温度传感器" );
		SYSTICK_DelayMs(900);
	}
	
	printf("成功检测到温度传感器" );	

	TIM_ms_Init(TIM2, 1000, TIM_IT_Update,2, ENABLE);	
 	TIMER_CallbackInstall(HW_TIMER2, timer2);
	
	while(1);
}
 

