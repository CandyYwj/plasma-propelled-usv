#include "sysinc.h"
#include "tds.h"

int main(void){
	float tds_value = 0;
	u16 adc_value = 0;
	
	UART_QuickInit(HW_UART1, 9600, 2, 2, ENABLE);  //初始化串口1，波特率9600
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);//初始化LED指示灯

	TDS_Init();  // 初始化TDS传感器
	
	while(1){  
		adc_value = TDS_GetRawValue();  // 获取ADC原始值
		tds_value = TDS_GetValue();     // 获取TDS值
		
		printf("ADC原始值: %d, TDS值: %.2f ppm\r\n", adc_value, tds_value);
		SYSTICK_DelayMs(1000);  // 每秒更新一次
	}
}
 

