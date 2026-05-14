#ifndef __APP_RECEIVE_H__
#define __APP_RECEIVE_H__

#include "main.h"
#include "sbus.h"

extern DMA_HandleTypeDef hdma_usart2_rx;

/* 遥控器数据结构 */
typedef struct {
    int16_t ch_raw[16];     /* 原始 SBUS 通道值 (0~2047, 11-bit) */

    float   roll_target;    /* 目标横滚角 (°)        CH0 映射 */
    float   pitch_target;   /* 目标俯仰角 (°)        CH1 映射 */
    float   throttle;       /* 油门 (0.0 ~ 1.0)      CH2 映射 */
    float   yaw_target;     /* 目标偏航角速度 (°/s)   CH3 映射 */

    uint8_t ch5_sw;         /* AUX1 三档开关: 0/1/2 */
    uint8_t ch6_sw;         /* AUX2 三档开关: 0/1/2 */

    uint8_t updated;        /* 本周期有新遥控数据 */
} RC_Data_t;

extern RC_Data_t rc_data;

void app_receive_isr_handler(void);
RC_Data_t* app_receive_getdata(void);

#endif
