#ifndef __SBUS_H
#define __SBUS_H

#include "main.h"

//通道映射常量
#define RC_CH_MIN      172     // SBUS 最小值 (~1000us)
#define RC_CH_MAX      1811    // SBUS 最大值 (~2000us)
#define RC_CH_MID      992     // SBUS 中值 ((172+1811)/2)
#define RC_DEADBAND    20      // 摇杆中位死区

//三档开关阈值
#define SW_LOW_THRESH    500
#define SW_HIGH_THRESH   1500

extern uint16_t CH[18];  // 通道值
extern uint8_t  rc_flag;

void Sbus_Data_Count(uint8_t *buf);
void process_sbus(uint8_t *data,uint16_t len);

float   map_channel(int16_t raw, float out_min, float out_max);
float   map_stick_sym(int16_t raw, float max_out);
uint8_t map_3pos_switch(int16_t raw);


#endif
