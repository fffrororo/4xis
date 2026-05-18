#include "app_receive.h"

//遥控连接状态
Remote_state remote_state = REMOTE_DISCONNECTED;

//遥控数据实例
RC_Data_t rc_data = {0};

//DMA 缓冲区跟踪
static volatile uint16_t sbus_dma_old_idx = 0;
static volatile uint16_t sbus_dma_new_idx = 0;
static volatile uint8_t  sbus_data_pending = 0;

static TickType_t last_frame_tick = 0;
static uint8_t    first_frame_ok  = 0;

/**
 *  @brief 由 USART2 空闲中断 ISR 调用，仅记录 DMA 位置并置位标志
 * 
 **/
void app_receive_isr_handler(void)
{
    uint16_t new_pos = UART_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

    if(new_pos != sbus_dma_old_idx)
    {
        sbus_dma_new_idx = new_pos;
        sbus_data_pending = 1;
    }
}

/**
 * @brief sbus_task 调用，从 DMA 缓冲区读取 SBUS 帧并映射为飞控目标值
 * 
 * @return RC_Data_t* 
 **/
RC_Data_t* app_receive_getdata(void)
{
    //连接超时检测：超过 SBUS_TIMEOUT_MS 无有效帧则判为断开
    if(first_frame_ok && (xTaskGetTickCount() - last_frame_tick) > pdMS_TO_TICKS(SBUS_TIMEOUT_MS))
    {
        remote_state = REMOTE_DISCONNECTED;
    }

    if(!sbus_data_pending)
    {
        rc_data.updated = 0;
        return &rc_data;
    }

    //临界区：快照 ISR 写入的位置信息
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t old_idx = sbus_dma_old_idx;
    uint16_t new_idx = sbus_dma_new_idx;
    sbus_dma_old_idx = new_idx;
    sbus_data_pending = 0;

    if(!primask)
        __enable_irq();

    //1. 从 DMA 环形缓冲区读取数据并解析 SBUS 帧
    if(new_idx > old_idx)
    {
        process_sbus(&uart_dma_data_buf[old_idx], new_idx - old_idx);
    }
    else if(new_idx < old_idx)
    {
        process_sbus(&uart_dma_data_buf[old_idx], UART_DMA_BUF_SIZE - old_idx);
        if(new_idx > 0)
            process_sbus(&uart_dma_data_buf[0], new_idx);
    }

    //2. 检查是否有新的有效帧
    if(!rc_flag)
    {
        rc_data.updated = 0;
        return &rc_data;
    }
    rc_flag = 0;

    //有效帧 -> 更新连接状态
    last_frame_tick = xTaskGetTickCount();
    first_frame_ok  = 1;
    remote_state    = REMOTE_CONNECTED;

    //3. 复制原始通道值
    for(uint8_t i = 0; i < 16; i++)
    {
        rc_data.ch_raw[i] = (int16_t)CH[i];
    }

    //4. 映射到飞控目标值
    rc_data.roll_target  = map_stick_sym(CH[0], MAX_ROLL_ANGLE);
    rc_data.pitch_target = map_stick_sym(CH[1], MAX_PITCH_ANGLE);
    rc_data.throttle     = map_channel(CH[2], 0.0f, 1.0f);
    rc_data.yaw_target   = map_stick_sym(CH[3], MAX_YAW_RATE);

    //5. 辅助通道
    rc_data.ch5_sw = map_3pos_switch(CH[4]);
    rc_data.ch6_sw = map_3pos_switch(CH[5]);

    rc_data.updated = 1;
    return &rc_data;
}
