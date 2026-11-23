#ifndef __LASER_H
#define __LASER_H

#include "sys.h"
#include "uart2.h"

// 函数声明
void laser_init(void);
void laser_send_start_cmd(void);
void laser_send_stop_cmd(void);
uint16_t laser_get_distance(void);

#endif
