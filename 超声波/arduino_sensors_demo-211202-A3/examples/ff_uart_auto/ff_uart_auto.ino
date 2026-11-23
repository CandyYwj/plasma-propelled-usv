/***********************************************************************
 * Copyright(C) 2021 深圳市电应普科技
 * 文件	:	ff_uart_auto.ino
 * 描述	:   监听传感器串口数据(ff 协议),转换成距离测距信息打印到 PC 串口
 *          此例程仅在 arduino uno 开发板上测试通过,包含 SoftwareSerial 第 3 方库,请自行添加
 *          A1 连接传感器 RX 引脚,A0 连接传感器 TX 引脚
 * 版本与日期  作者  说明
 * 211102-A0  CH  创建了该文件
 * 211123-A0  CH  FF 协议增加包头判断
 ***********************************************************************/
/* 包含头文件 -------------------------------------------------------------------*/
#include <Arduino.h>
#include <SoftwareSerial.h>

/* 宏定义 -----------------------------------------------------------------------*/
#define SOFT_VERSION "UART_AUTO-211123-A0"  //软件版本

#define COM_TX_PIN A1  // 开发板模拟串口 TX 引脚
#define COM_RX_PIN A0  // 开发板模拟串口 RX 引脚

/* 类型定义 ---------------------------------------------------------------------*/
/* 私有变量 ---------------------------------------------------------------------*/
SoftwareSerial softSerial( COM_RX_PIN, COM_TX_PIN );  // 软件模拟串口
/* 扩展变量 ---------------------------------------------------------------------*/
/* 函数声明 ---------------------------------------------------------------------*/
/* 函数实现 ---------------------------------------------------------------------*/
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
    softSerial.begin( 9600 );
}

void loop()
{
    if ( softSerial.available() )
    {
        static uint8_t  recv_buf[ 10 ] = { 0 };
        static uint16_t recv_buf_cnt   = 0;
        uint16_t        len            = softSerial.readBytes( ( uint8_t* )&recv_buf, 4 );
        if ( len == 4 && recv_buf[0] == 0xff)
        {
            uint8_t sum = recv_buf[ 0 ] + recv_buf[ 1 ] + recv_buf[ 2 ];
            if ( sum == recv_buf[ 3 ] )
            {
                uint16_t distance      = recv_buf[ 1 ] << 8 | recv_buf[ 2 ];
                char     out_txt[ 16 ] = { 0 };
                sprintf( out_txt, "%d mm \r\n", distance );
                Serial.print( out_txt );
                // Serial.write( recv_buf[ 1 ] ), Serial.write( recv_buf[ 2 ] );
            }
        }
    }

    static uint32_t led_cnt = millis();
    if ( millis() - led_cnt > 100 )
    {
        digital_toggle( LED_BUILTIN );
        led_cnt = millis();
    }
}
// main.cpp