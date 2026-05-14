#include "app_receive.h"

/* ========================== 通道映射常量 ========================== */
#define RC_CH_MIN      172     /* SBUS 最小值 (~1000us) */
#define RC_CH_MAX      1811    /* SBUS 最大值 (~2000us) */
#define RC_CH_MID      1024    /* SBUS 中值   (~1500us) */
#define RC_DEADBAND    20      /* 摇杆中位死区 */

/* 输出限幅 */
#define MAX_ROLL_ANGLE   10.0f   /* 横滚最大角 (°) */
#define MAX_PITCH_ANGLE  10.0f   /* 俯仰最大角 (°) */
#define MAX_YAW_RATE     200.0f  /* 偏航最大角速度 (°/s) */

/* 三档开关阈值 */
#define SW_LOW_THRESH    500
#define SW_HIGH_THRESH   1500

/* ========================== DMA 缓冲区跟踪 ========================== */
static volatile uint16_t sbus_dma_old_idx = 0;
static volatile uint16_t sbus_dma_new_idx = 0;
static volatile uint8_t  sbus_data_pending = 0;

/* ========================== 遥控数据实例 ========================== */
RC_Data_t rc_data = {0};

/* ========================== 辅助映射函数 ========================== */

/**
 * @brief  将原始通道值线性映射到 [out_min, out_max],处理油门
 */
static float map_channel(int16_t raw, float out_min, float out_max)
{
    if (raw <= RC_CH_MIN) return out_min;
    if (raw >= RC_CH_MAX) return out_max;
    return out_min + (out_max - out_min) * (raw - RC_CH_MIN) / (float)(RC_CH_MAX - RC_CH_MIN);
}

/**
 * @brief  将摇杆通道值对称映射到 [-max_out, +max_out]，含中位死区，处理角度
 */
static float map_stick_sym(int16_t raw, float max_out)
{
    if (raw > RC_CH_MID - RC_DEADBAND && raw < RC_CH_MID + RC_DEADBAND) {
        return 0.0f;
    }

    if (raw <= RC_CH_MID) {
        float ratio = (float)(RC_CH_MID - raw) / (float)(RC_CH_MID - RC_CH_MIN);
        if (ratio > 1.0f) ratio = 1.0f;
        return -max_out * ratio;
    } 
    else {
        float ratio = (float)(raw - RC_CH_MID) / (float)(RC_CH_MAX - RC_CH_MID);
        if (ratio > 1.0f) ratio = 1.0f;
        return max_out * ratio;
    }
}

/**
 * @brief  根据通道值判定三档开关位置: 0=低, 1=中, 2=高
 */
static uint8_t map_3pos_switch(int16_t raw)
{
    if (raw < SW_LOW_THRESH)  return 0;
    if (raw > SW_HIGH_THRESH) return 2;
    return 1;
}

/* ========================== 公开接口 ========================== */

/**
 * @brief  由 USART2 空闲中断 ISR 调用，仅记录 DMA 位置并置位标志
 */
void app_receive_isr_handler(void)
{
    uint16_t new_pos = UART_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

    if (new_pos != sbus_dma_old_idx) {
        sbus_dma_new_idx = new_pos;
        sbus_data_pending = 1;
    }
}

/**
 * @brief  由 sbus_task 调用，从 DMA 缓冲区读取 SBUS 帧并映射为飞控目标值
 * @retval RC_Data_t*  指向最新遥控数据的指针（通过 rc_data.updated 判断是否更新）
 */
RC_Data_t* app_receive_getdata(void)
{
    if (!sbus_data_pending) {
        rc_data.updated = 0;
        return &rc_data;
    }

    /* 临界区：快照 ISR 写入的位置信息 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t old_idx = sbus_dma_old_idx;
    uint16_t new_idx = sbus_dma_new_idx;
    sbus_dma_old_idx = new_idx;
    sbus_data_pending = 0;

    if (!primask) {
        __enable_irq();
    }

    /* —————— 1. 从 DMA 环形缓冲区读取数据并解析 SBUS 帧 —————— */
    if (new_idx > old_idx) {
        process_sbus(&uart_dma_data_buf[old_idx], new_idx - old_idx);
    } 
    else if (new_idx < old_idx) {
        /* DMA 缓冲区回绕 */
        process_sbus(&uart_dma_data_buf[old_idx], UART_DMA_BUF_SIZE - old_idx);
        if (new_idx > 0) {
            process_sbus(&uart_dma_data_buf[0], new_idx);
        }
    }

    /* —————— 2. 检查是否有新的有效帧 —————— */
    if (!rc_flag) {
        rc_data.updated = 0;
        return &rc_data;
    }
    rc_flag = 0;

    /* —————— 3. 复制原始通道值 —————— */
    for (uint8_t i = 0; i < 16; i++) {
        rc_data.ch_raw[i] = (int16_t)CH[i];
    }

    /* —————— 4. 映射到飞控目标值 —————— */
    rc_data.roll_target  = map_stick_sym(CH[0], MAX_ROLL_ANGLE);
    rc_data.pitch_target = map_stick_sym(CH[1], MAX_PITCH_ANGLE);
    rc_data.throttle     = map_channel(CH[2], 0.0f, 1.0f);
    rc_data.yaw_target   = map_stick_sym(CH[3], MAX_YAW_RATE);

    /* —————— 5. 辅助通道 —————— */
    rc_data.ch5_sw = map_3pos_switch(CH[4]);
    rc_data.ch6_sw = map_3pos_switch(CH[5]);

    rc_data.updated = 1;
    return &rc_data;
}
