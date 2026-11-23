
/***********************************************************************
 * Copyright(C) 2021 深圳市电应普科技
 * 文件	:	pwm.ino
 * 描述	:   读取pwm传感器数据，将读取到的 PWM 占空比换算成距离,打印到 PC 串口
 *          A1 连接传感器 RX 引脚,A0 连接传感器 TX 引脚
 *          修改 TRIGGER_MODE 此宏定义切换触发方式,默认低电平触发
 *
 *          工作过程(参考 A02 传感器)
 *               当触发输入引线“RX” 接收到一个有下降沿的触发脉冲，下降沿会触发模组工作一次，
 *               输出引线“TX”将输出一次TTL电平的PWM高电平脉宽信号，模组的触发周期必须大于70ms,
 *               如果模组没有检测到物体，输出引线“TX”将输出约35ms的固定脉宽。
 *           计算方式
 *               公式：S=T*V/2（S为距离值，T为PWM高电平脉宽时间，V为声音在空气中的传播速度）。
 *               在常温下得声速V为348M/S，可简化公式得S= T/57.5 （此时距离S单位为厘米，时间T单位为微秒）。
 *               举例：当输出引线“TX”的PWM高电平脉宽时间T3为10000us时，
 *               得S= T/57.5=10000/57.5≈173.9(cm)，表示当前测量的距离值为173.9厘米。
 * 版本与日期  作者  说明
 * 211102-A0   CH   创建了该文件
 * 211123-A0   CH   PWM触发脉冲由500us减小为100us
 * 211130-A0   CH   处理显示值输出不正确问题
 ***********************************************************************/
/* 包含头文件 -------------------------------------------------------------------*/
#include <Arduino.h>

/* 宏定义 -----------------------------------------------------------------------*/
#define SOFT_VERSION "PWM-211130-A0"  // 软件版本

#define COM_TX_PIN A1
#define COM_RX_PIN A0

#define TRIGGER_LOW  0
#define TRIGGER_HIGH 1
#define TRIGGER_MODE TRIGGER_LOW  // 触发模式, 0 低触发, 1 高触发

/* 类型定义 ---------------------------------------------------------------------*/
/* 私有变量 ---------------------------------------------------------------------*/
/* 扩展变量 ---------------------------------------------------------------------*/
/* 函数声明 ---------------------------------------------------------------------*/
void digital_toggle( uint8_t pin );
void pwm_trigger();
void pwm_stop_trigger();
/* 函数实现 ---------------------------------------------------------------------*/

void pwm_trigger()
{
#if TRIGGER_MODE == TRIGGER_LOW
    digitalWrite( COM_TX_PIN, LOW );  // 触发输出
#else
    digitalWrite( COM_TX_PIN, HIGH );  // 触发不输出
#endif
}

void pwm_stop_trigger()
{
#if TRIGGER_MODE == TRIGGER_LOW
    digitalWrite( COM_TX_PIN, HIGH );  // 停止触发
#else
    digitalWrite( COM_TX_PIN, LOW );  // 停止触发
#endif
}

void digital_toggle( uint8_t pin )
{
    digitalRead( pin ) ? digitalWrite( pin, 0 ) : digitalWrite( pin, 1 );
}

void setup()
{
    // put your setup code here, to run once:
    pinMode( LED_BUILTIN, OUTPUT );
    Serial.begin( 9600 );
    Serial.println( SOFT_VERSION );
    pinMode( COM_TX_PIN, OUTPUT );  // 触发测距引脚
    pwm_stop_trigger();             // 默认不触发
    pinMode( COM_RX_PIN, INPUT );
}

void loop()
{
    static uint32_t trigger_cnt = millis();
    if ( millis() - trigger_cnt > 500 )  // 间隔 500 ms 触发输出 1 次
    {
        pwm_trigger();  // 触发输出
        uint32_t trigger_stop_cnt = micros();
        while ( digitalRead( COM_RX_PIN ) == LOW )  //{} 等待低电平结束
        {
            if ( micros() - trigger_stop_cnt > 100 )
            {
                pwm_stop_trigger();
                break;
            }
        }
        while ( digitalRead( COM_RX_PIN ) == LOW ) {}  // 等待脉宽开始输出
        uint32_t pluse_time = micros();
        while ( digitalRead( COM_RX_PIN ) == HIGH ) {}  // 等待脉宽结束
        pluse_time = micros() - pluse_time;             // 计算脉宽时间
        // Serial.print( pluse_time );
        uint16_t distance_val  = pluse_time / 57.5 * 10;
        char     out_txt[ 16 ] = { 0 };
        sprintf( out_txt, "%d mm \r\n", distance_val );
        Serial.print( out_txt );

        trigger_cnt = millis();
    }

    static uint32_t led_cnt = millis();
    if ( millis() - led_cnt > 100 )
    {
        digital_toggle( LED_BUILTIN );
        led_cnt = millis();
    }
}
// main.cpp