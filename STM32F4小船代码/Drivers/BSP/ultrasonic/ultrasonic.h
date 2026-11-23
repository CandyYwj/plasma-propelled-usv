#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "sys.h"

// 函数声明
void ultrasonic_send_measure_cmd(void);
uint16_t ultrasonic_parse_data(uint8_t *data, uint16_t len);
uint16_t ultrasonic_get_distance(void);

#endif

