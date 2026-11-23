/**
  ******************************************************************************
  * @file    tds.h
  * @version V1.0
  * @note    TDS传感器驱动头文件
  ******************************************************************************
  */
#ifndef __TDS_H
#define __TDS_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "sysinc.h"

// TDS传感器函数声明
void TDS_Init(void);
float TDS_GetValue(void);
u16 TDS_GetRawValue(void);

#ifdef __cplusplus
}
#endif

#endif /* __TDS_H */ 