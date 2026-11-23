/***********************************************************************
 * Copyright(C) 2021 深圳市电应普科技
 * 文件  : H03TRT.ino
 * 描述 :   定时触发传感器输出,解析传感器输出数据(H01 协议数据)打印到 PC 串口
 *          arduino unon 开发板发送0X55 0XAA 0X01 0X01 0X01指令触发传感器工作，传感器回复 0x55 0xAA 0x01 0x01 data_h data_l sum  
 *          校验码sum = 0x55 + 0xAA + 0x01 + 0x01 + data_h + data_l，详细协议说明请阅读H01模组相关规格书
 *          成功接收到传感器回复的距离数据后，将距离值输出到主串口
 *          此例程仅在 arduino unon 开发板上测试通过,包含SoftwareSerial 一个第 3 方库,请自行添加
 *          A1 连接传感器 RX 引脚,A0 连接传感器 TX 引脚
 * 版本与日期  作者  说明
 * 211112-A0   YYL  创建了该文件
 * 211130-A0   YYL  对代码进行了优化
 ***********************************************************************/
/* 包含头文件 -------------------------------------------------------------------*/
#include <Arduino.h>
#include <SoftwareSerial.h>

/* 宏定义 -----------------------------------------------------------------------*/
#define SOFT_VERSION "H01-211130-A0"  //软件版本

#define COM_TX_PIN A1   // 开发板模拟串口 TX 引脚
#define COM_RX_PIN A0   // 开发板模拟串口 RX 引脚

/* 类型定义 ---------------------------------------------------------------------*/
/* 扩展变量 ---------------------------------------------------------------------*/
/* 函数声明 ---------------------------------------------------------------------*/
void digital_toggle( uint8_t pin );
/* 私有变量 ---------------------------------------------------------------------*/
SoftwareSerial softSerial( COM_RX_PIN, COM_TX_PIN );  // 软件模拟串口
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
    softSerial.begin( 9600 );//初始化连接传感器的模拟串口 波特率9600
}

void loop()
{
    static uint32_t trigger_cnt = 0;
    static uint8_t  recv_buf[ 10 ] = { 0 };
    static uint16_t recv_buf_cnt   = 0;
    static uint16_t len = 0;  
    uint16_t temp_len = 0;  

    if ( millis() - trigger_cnt > 500 )//以500ms的周期发送触发传感器工作指令
    {
        uint8_t i;
        uint8_t tripcmd_buf[5] = {0x55,0xAA,0x01,0x01};//帧头,帧头,传感器地址,特征码 ,校验和
        tripcmd_buf[4] = tripcmd_buf[0] + tripcmd_buf[1] + tripcmd_buf[2] + tripcmd_buf[3];//计算校验和
        softSerial.write( tripcmd_buf,5);//发送指令到传感器       
        trigger_cnt = millis();      
    }
   temp_len = softSerial.available();//获取串口缓冲区接到了多少个数据
    if(temp_len>=7)//当模拟串口接收到数据
    {
       len = softSerial.readBytes(recv_buf,temp_len);//读取数据到缓冲区
    }
    if ( len >= 7 )//当模拟串口接收到的数据>=3个字节
    {
        len = 0;
        uint8_t sum = recv_buf[ 0 ] + recv_buf[ 1 ] + recv_buf[ 2 ] + recv_buf[ 3 ] + recv_buf[ 4 ] + recv_buf[ 5 ];//计算校验码
        if ( (0x55 == recv_buf[ 0 ])&&(0xAA == recv_buf[ 1 ])&&(sum == recv_buf[ 6 ]) )//进行校验
        {
            uint16_t distance   = recv_buf[ 4 ] << 8 | recv_buf[ 5 ];//获取距离值
            char     out_txt[ 32 ] = { 0 };           
            sprintf( out_txt,"distance : %d mm \r\n", distance );          
            Serial.print( out_txt );//输出距离值到主串口
        }
    }

    static uint32_t led_cnt = millis();
    if ( millis() - led_cnt > 100 )//LED以100ms周期闪烁
    {
        digital_toggle( LED_BUILTIN );
        led_cnt = millis();
    }
}
// main.cpp
