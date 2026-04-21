#ifndef __SBUS_H
#define __SBUS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif
extern uint16_t CH[18];  // 通道值
extern uint8_t  rc_flag;

void Sbus_Data_Count(uint8_t *buf);



#ifdef __cplusplus
}
#endif

#endif