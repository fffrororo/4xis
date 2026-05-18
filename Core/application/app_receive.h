#ifndef __APP_RECEIVE_H__
#define __APP_RECEIVE_H__

#include "main.h"
#include "sbus.h"

//输出限幅
#define MAX_ROLL_ANGLE   10.0f   // 横滚最大角 (°)
#define MAX_PITCH_ANGLE  10.0f   // 俯仰最大角 (°)
#define MAX_YAW_RATE     200.0f  // 偏航最大角速度 (°/s)

//SBUS 连接超时 (ms)
#define SBUS_TIMEOUT_MS  100

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

typedef enum {
    REMOTE_CONNECTED = 0,
    REMOTE_DISCONNECTED,
} Remote_state;

extern RC_Data_t rc_data;
extern Remote_state remote_state;

void app_receive_isr_handler(void);
RC_Data_t* app_receive_getdata(void);

#endif
