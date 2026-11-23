/***********************************************************************
 * Copyright(C) 2021 深圳市电应普科技
 * 文件	:	u02.ino
 * 描述	:   读取传感器数据（支持自动输出协议/Modbus/电容式油杆兼容输出协议/模拟电压输出,通过修改 MODULE_VER 适配），将测距信息打印到 PC 串口
 *          A1 连接传感器 RX 引脚,A0 连接传感器 TX 引脚,VOLT_VER版本,传感器的 S 脚连接开发板的 A0 引脚
 *          此例程仅在 arduino uno 开发板上测试通过,包含 SoftwareSerial 第 3 方库,请自行添加
 *          切换协议时，请手动重启传感器
 *
 * 版本与日期  作者  说明
 * 211104-A0    CH   创建了该文件,添加 自动输出协议,modbus 协议,电容式油杆兼容输出协议
 * 211130-A0    CH   修复 AUTO_OUT_VER 版本测距项格式显示错误
 * 211130-A1    CH   修复 AUTO_OUT_VER 版本显示缓冲区越界问题
 * 211130-A1    CH   优化 AUTO_OUT_VER 版本解析代码
 ***********************************************************************/
/* 包含头文件 -------------------------------------------------------------------*/
#include <Arduino.h>
#include <SoftwareSerial.h>

/* 宏定义 -----------------------------------------------------------------------*/
#define SOFT_VERSION "U02-211202-A0"  //软件版本

#define AUTO_OUT_VER 1  // 自动输出协议
#define MB_VER       2  // modbus 协议
#define JR_OUT_VER   3  // 电容式油杆兼容输出协议
#define VOLT_VER     4  // 模拟电压输出协议
#define MODULE_VER   AUTO_OUT_VER

#define COM_TX_PIN A1  // 开发板模拟串口 TX 引脚
#define COM_RX_PIN A0  // 开发板0模拟串口 RX 引脚

/* 类型定义 ---------------------------------------------------------------------*/
/* 私有变量 ---------------------------------------------------------------------*/
SoftwareSerial softSerial( COM_RX_PIN, COM_TX_PIN );  // 软件模拟串口

/* Table of CRC values for high-order byte */
#if defined( ARDUINO ) && defined( __AVR__ )
static PROGMEM const uint8_t table_crc_hi[] = {
#else
static const uint8_t table_crc_hi[] = {
#endif
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
#if defined( ARDUINO ) && defined( __AVR__ )
#include <avr/pgmspace.h>
static PROGMEM const uint8_t table_crc_lo[] = {
#else
static const uint8_t table_crc_lo[] = {
#endif
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
/* 扩展变量 ---------------------------------------------------------------------*/
/* 函数声明 ---------------------------------------------------------------------*/
uint16_t crc16( uint8_t* buffer, uint16_t buffer_length );
/* 函数实现 ---------------------------------------------------------------------*/
void digital_toggle( uint8_t pin )
{
    digitalRead( pin ) ? digitalWrite( pin, 0 ) : digitalWrite( pin, 1 );
}

uint16_t crc16( uint8_t* buffer, uint16_t buffer_length )
{
    uint8_t      crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t      crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i;             /* will index into CRC lookup */

    /* pass through message buffer */
    while ( buffer_length-- )
    {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
#if defined( ARDUINO ) && defined( __AVR__ )
        crc_hi = crc_lo ^ pgm_read_byte_near( table_crc_hi + i );
        crc_lo = pgm_read_byte_near( table_crc_lo + i );
#else
        crc_hi = crc_lo ^ table_crc_hi[ i ];
        crc_lo = table_crc_lo[ i ];
#endif
    }

    return ( crc_hi << 8 | crc_lo );
}
uint8_t add_sum( uint8_t* _buf, uint16_t _len )
{
    uint8_t ret = 0;
    for ( int i = 0; i < _len; i++ ) ret += _buf[ i ];
    return ret;
}

void setup()
{
    // put your setup code here, to run once:
    pinMode( LED_BUILTIN, OUTPUT );
    Serial.begin( 9600 );
    Serial.println( SOFT_VERSION );
    softSerial.begin( 9600 );

#if MODULE_VER == VOLT_VER
    pinMode( COM_RX_PIN, INPUT );
#endif
}

void loop()
{
#if MODULE_VER == JR_OUT_VER
    /* 电容式油杆兼容输出模式
     * 由终端或电脑发往液位仪的命令
     * ASCII      ：  $!RY0151
     * 十六进制	   ：  24 21 52 59 30 31 35 31 0D 0A
     * 液位仪回复：
     * ASCII      ：  *CFV0100FA32B6
     * 00FA32为当前AD值，00为不足6个字节补0代替，FA32为当前油位AD值ASCII表示的十六进制，即表示当前油位值为0xFA32。
     * 注：此AD值为油位0%~100%变化而对应的000000~~00FFFF的AD值
     * 十六进制： 2A 43 46 56 30 31 30 30 46 41 33 32 42 36 0D 0A
     * 注：此AD值为油位0~999.9mm变化而对应的000000~~00FFFF的AD值
     */
    static uint32_t send_timecnt = 0;
    if ( millis() - send_timecnt > 500 )
    {
        softSerial.print( "$!RY0151" );
        uint8_t  response_buf[ 16 ] = { 0 };
        uint16_t len                = softSerial.readBytes( response_buf, 16 );

        // Serial.println( ( char* )response_buf );

        if ( response_buf[ 0 ] == '*' && response_buf[ 14 ] == 0x0d && response_buf[ 15 ] == 0x0a )
        {
            char tmp_buf[ 7 ] = { 0 };
            memcpy( tmp_buf, &response_buf[ 6 ], 6 );
            uint32_t distance_adc = 0;  // 此AD值为油位0~999.9mm变化而对应的000000~~00FFFF的AD值
            sscanf( ( char* )tmp_buf, "%x", &distance_adc );
            // printf( "%x", distance_adc );
            // uint16_t distance = distance_adc / 0xffff * 1000;
            uint16_t distance = distance_adc * 1000 / 0xffff;

            char out_txt[ 16 ] = { 0 };
            sprintf( out_txt, "液位:%dmm \r\n", distance );
            Serial.print( out_txt );
        }

        send_timecnt = millis();
    }
#elif MODULE_VER == MB_VER
    /*  modbus 通讯举例:
        读取
        例1： 测距,单位：0.1mm
        主机：01 03 01 00 00 01 85 F6
        从机：01 03 02 FF FF B9 F4
        说明：
    */
    static uint32_t send_timecnt = 0;
    if ( millis() - send_timecnt > 500 )
    {
        uint8_t send_cmd_buf[] = { 0x01, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85, 0xF6 };  //读取探头1,2,3,4测量值
        softSerial.write( send_cmd_buf, sizeof( send_cmd_buf ) );                     // 发送 modbus 指令给传感器

        uint8_t  read_reg_num       = send_cmd_buf[ 5 ];
        uint8_t  response_buf[ 16 ] = { 0 };
        uint16_t len                = softSerial.readBytes( response_buf, 7 );
        // for ( int i = 0; i < len; i++ )  //调试输出
        // {
        //     uint8_t tmp[ 8 ] = { 0 };
        //     sprintf( tmp, "%02x ", response_buf[ i ] );
        //     Serial.print( ( char* )tmp );
        // }
        // Serial.println();

        uint16_t calc_checksum = crc16( ( uint8_t* )&response_buf, len - 2 );
        uint16_t recv_checksum = response_buf[ len - 2 ] << 8 | response_buf[ len - 1 ];
        if ( calc_checksum == recv_checksum )
        {
            uint16_t distance      = response_buf[ 3 ] << 8 | response_buf[ 4 ];
            uint8_t  out_txt[ 32 ] = { 0 };

            if ( distance == 0xffff ) { distance = 0; }
            sprintf( out_txt, "距离:%d.%d mm \r\n", distance / 10, distance % 10 );
            Serial.print( ( char* )out_txt );
        }
        send_timecnt = millis();
    }
#elif MODULE_VER == AUTO_OUT_VER
    /*  *XD ,小时 , ID号 , 液位 , 实时值 , 信号质量 , 温度 ，检验码#
     *  *XD,0000,01,FFFF,0000,0030,2469,1433#
     */
    if ( softSerial.available() )
    {
        static uint8_t  recv_buf[ 48 ] = { 0 };
        static uint16_t recv_buf_cnt   = 0;
        uint16_t        len            = softSerial.readBytes( ( uint8_t* )&recv_buf, 37 );

        // Serial.println( ( char* )recv_buf );

        uint16_t hour, id, liquid_level, real_val, signal_quality, temperature, checksum;
        hour = id = liquid_level = real_val = signal_quality = temperature = checksum = 0;
        if ( ( recv_buf[ 0 ] == '*' ) && ( recv_buf[ 36 ] == '#' ) )
        // if (  recv_buf[ 0 ] == '*'  &&  recv_buf[ 36 ] == '#'  )
        {
            // Serial.println( ( char* )recv_buf );
            uint16_t calc_checksum = 0;
            for ( int i = 4; i <= 31; i++ ) calc_checksum += recv_buf[ i ];
            sscanf( recv_buf, "*XD,%d,%d,%d,%d,%d,%d,%d#", &hour, &id, &liquid_level, &real_val, &signal_quality, &temperature, &checksum );

            if ( liquid_level == 0xffff ) liquid_level = 0;  // 0xffff 代表测不到
            if ( real_val == 0xffff ) real_val = 0;

            // if ( checksum == calc_checksum )
            {
                static char out_txt[ 200 ] = { 0 };
                sprintf( out_txt, "小时:%d ID号:%d 油位值:%d.%dmm 实时值:%d.%dmm 信号质量:%d 温度:%d.%d℃ \r\n", hour, id, liquid_level / 10, liquid_level % 10, real_val / 10, real_val % 10,
                         signal_quality, temperature / 10, temperature % 10 );
                Serial.print( out_txt );
            }
        }
    }

#elif MODULE_VER == VOLT_VER
    /*
        通过读取电压模拟量输出引脚输出的电压值，就可以计算对应的距离值，传感器出厂默认以0-5000mV的电压值对应0-1000mm的距离值，每5mV的电压对应1mm的距离。
            当探测的距离大于1000mm时输出≈5000mV电压，没有探测到液体时输出小于50mV。
        例如：当前读取模拟量输出引脚电压值为3000mV时
        由：S=U/5000*1000 ；
        得：距离=3000/5000*1000=600mm；
        故：当前探测到液体的距离值为600mm。
    */
    static uint32_t scan_cnt = millis();
    if ( millis() - scan_cnt > 500 )
    {
        uint8_t  read_count = 10;
        uint16_t adc_val    = 0;
        for ( int i = 0; i < read_count; i++ ) adc_val += analogRead( COM_RX_PIN );
        adc_val /= read_count;

        // Serial.println(adc_val);
        // uint16_t volt = adc_val / 1023.0 * 5000;  // voltage unit mv
        uint16_t volt = adc_val * 5000UL / 1023;  // voltage unit mv
        // uint16_t distance_val = volt / 5000.0 * 1000;     // distance unit mm
        uint16_t distance_val = volt * 1000UL / 5000;  // distance unit mm

        char out_txt[ 10 ] = { 0 };
        sprintf( out_txt, "%d mm", distance_val );
        Serial.println( out_txt );

        scan_cnt = millis();
    }

#endif

    static uint32_t led_cnt = millis();
    if ( millis() - led_cnt > 100 )
    {
        digital_toggle( LED_BUILTIN );
        led_cnt = millis();
    }
}
// main.cpp
