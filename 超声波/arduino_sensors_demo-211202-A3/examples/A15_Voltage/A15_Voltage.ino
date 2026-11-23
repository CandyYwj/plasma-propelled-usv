/***********************************************************************
 * Copyright(C) 2021 深圳市电应普科技
 * 文件  : A15_Voltage.ino
 * 描述 :  不停采集传感器模拟电压输出引脚的电压，每1000个电压AD数据为一组进行求和平均后转换成电压和距离值打印到 PC 串口
 *          A0 连接传感器模拟电压输出引脚，传感器需和arduino uno共地。模拟电压采集范围0V~5V，对应150MM~2000MM。
 * 版本与日期  作者  说明
 * 211112-A0   YYL  创建了该文件
 * 211126-A0   YYL  增加注释
 ***********************************************************************/
/* 包含头文件 -------------------------------------------------------------------*/
#include <Arduino.h>

/* 宏定义 -----------------------------------------------------------------------*/
#define SOFT_VERSION "A15_Voltage-211126-A0"  //软件版本

#define COM_TX_PIN A1   // 开发板模拟串口 TX 引脚
#define COM_RX_PIN A0   // 开发板模拟串口 RX 引脚

/* 类型定义 ---------------------------------------------------------------------*/
/* 扩展变量 ---------------------------------------------------------------------*/
/* 函数声明 ---------------------------------------------------------------------*/
void digital_toggle( uint8_t pin );
/* 私有变量 ---------------------------------------------------------------------*/
/* 函数实现 ---------------------------------------------------------------------*/
void digital_toggle( uint8_t pin )
{
  digitalRead( pin ) ? digitalWrite( pin, 0 ) : digitalWrite( pin, 1 );
}

void setup()
{
  // put your setup code here, to run once:
  pinMode( LED_BUILTIN, OUTPUT );//初始化LED
  Serial.begin( 9600 );//初始化主串口 波特率9600
  Serial.println( SOFT_VERSION );//主串口输出软件版本
  pinMode( COM_RX_PIN, INPUT );//将A0引脚初始化成输入
}

void loop()
{
  static uint32_t trigger_cnt = 0;
  static uint32_t out_vol = 0;
  static uint16_t read_cnt = 0;
  static uint32_t adc_val = 0;
  static uint16_t analog_A1 = 150;//A1最小量程
  static uint16_t analog_A2 = 2000;//A2最大量程

  read_cnt++;
  adc_val += analogRead( COM_RX_PIN );//读取A0电压值并求和
  if (read_cnt >= 1000)//连续采集A0电压值1000次求和取平均值
  {
    read_cnt = 0;
    out_vol = adc_val / 1000;
    adc_val = 0;
  }
  if ( millis() - trigger_cnt > 1000 )//以1000ms的周期发送电压值和距离值到PC串口
  {
    char     out_txt[ 16 ] = { 0 };
    uint16_t out_data = out_vol * 5000 / 1023;//以5000mv作为ADC基准，求A0引脚电压值
    uint16_t distance = (out_data * ((uint32_t)(analog_A2 - analog_A1))) / 5000 + analog_A1;//根据电压值求A15测量的距离值
    sprintf( out_txt, "vol: %dmv  ", out_data );
    Serial.print( out_txt );//输出电压值到PC串口
    sprintf( out_txt, "distance: %dmm \r\n", distance );
    Serial.print( out_txt );//输出距离值到PC串口
    trigger_cnt = millis();
  }
  static uint32_t led_cnt = millis();
  if ( millis() - led_cnt > 100 )//LED以100ms周期闪烁
  {
    digital_toggle( LED_BUILTIN );
    led_cnt = millis();
  }
}
// main.cpp
