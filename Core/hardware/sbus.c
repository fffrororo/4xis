#include "main.h"
#include "sbus.h"

#define SBUS_FRAME_LEN 25
uint8_t sbus_frame[SBUS_FRAME_LEN];
uint8_t sbus_index = 0;

uint16_t CH[18];  // 通道值
uint8_t  rc_flag = 0;


/**
 * @brief 解析SBUS数据
 * @param buf: 接收到的数据
CH[0]	Roll 横滚
CH[1]	Pitch 俯仰
CH[2]	Throttle 油门
CH[3]	Yaw 偏航
CH[4]	AUX1
CH[5]	AUX2
*/
//数据解析函数，传入usart缓冲区数据指针
void Sbus_Data_Count(uint8_t *buf)
{
	CH[ 0] = ((int16_t)buf[ 2] >> 0 | ((int16_t)buf[ 3] << 8 )) & 0x07FF;
	CH[ 1] = ((int16_t)buf[ 3] >> 3 | ((int16_t)buf[ 4] << 5 )) & 0x07FF;
	CH[ 2] = ((int16_t)buf[ 4] >> 6 | ((int16_t)buf[ 5] << 2 )  | (int16_t)buf[ 6] << 10 ) & 0x07FF;
	CH[ 3] = ((int16_t)buf[ 6] >> 1 | ((int16_t)buf[ 7] << 7 )) & 0x07FF;
	CH[ 4] = ((int16_t)buf[ 7] >> 4 | ((int16_t)buf[ 8] << 4 )) & 0x07FF;
	CH[ 5] = ((int16_t)buf[ 8] >> 7 | ((int16_t)buf[ 9] << 1 )  | (int16_t)buf[10] <<  9 ) & 0x07FF;
	CH[ 6] = ((int16_t)buf[10] >> 2 | ((int16_t)buf[11] << 6 )) & 0x07FF;
	CH[ 7] = ((int16_t)buf[11] >> 5 | ((int16_t)buf[12] << 3 )) & 0x07FF;
	
	CH[ 8] = ((int16_t)buf[13] << 0 | ((int16_t)buf[14] << 8 )) & 0x07FF;
	CH[ 9] = ((int16_t)buf[14] >> 3 | ((int16_t)buf[15] << 5 )) & 0x07FF;
	CH[10] = ((int16_t)buf[15] >> 6 | ((int16_t)buf[16] << 2 )  | (int16_t)buf[17] << 10 ) & 0x07FF;
	CH[11] = ((int16_t)buf[17] >> 1 | ((int16_t)buf[18] << 7 )) & 0x07FF;
	CH[12] = ((int16_t)buf[18] >> 4 | ((int16_t)buf[19] << 4 )) & 0x07FF;
	CH[13] = ((int16_t)buf[19] >> 7 | ((int16_t)buf[20] << 1 )  | (int16_t)buf[21] <<  9 ) & 0x07FF;
	CH[14] = ((int16_t)buf[21] >> 2 | ((int16_t)buf[22] << 6 )) & 0x07FF;
	CH[15] = ((int16_t)buf[22] >> 5 | ((int16_t)buf[23] << 3 )) & 0x07FF;
}

void process_sbus(uint8_t *data,uint16_t len)
{
	uint16_t i;

	for(i=0;i<len;i++)
	{
		uint8_t byte = data[i];

		//等待帧头
		if(sbus_index == 0 )
		{
			if(byte == 0x0F)
			{
				sbus_frame[sbus_index++] = byte;
			}

		}
		else
		{
			//收集数据
			sbus_frame[sbus_index++] = byte;
			//
			if(sbus_index >= SBUS_FRAME_LEN)
            {
                // 校验帧尾
                if(sbus_frame[24] == 0x00)
                {
                    Sbus_Data_Count(sbus_frame);

                    rc_flag = 1;
                }

                // 重新同步
                sbus_index = 0;
			}

		}
	}
}

//将原始通道值线性映射到 [out_min, out_max],处理油门
float map_channel(int16_t raw, float out_min, float out_max)
{
    if(raw <= RC_CH_MIN) return out_min;
    if(raw >= RC_CH_MAX) return out_max;
    return out_min + (out_max - out_min) * (raw - RC_CH_MIN) / (float)(RC_CH_MAX - RC_CH_MIN);
}

//将摇杆通道值对称映射到 [-max_out, +max_out]，含中位死区，处理角度
float map_stick_sym(int16_t raw, float max_out)
{
    if(raw > RC_CH_MID - RC_DEADBAND && raw < RC_CH_MID + RC_DEADBAND)
        return 0.0f;

    if(raw <= RC_CH_MID)
    {
        float ratio = (float)(RC_CH_MID - raw) / (float)(RC_CH_MID - RC_CH_MIN);
        if(ratio > 1.0f) ratio = 1.0f;
        return -max_out * ratio;
    }
    else
    {
        float ratio = (float)(raw - RC_CH_MID) / (float)(RC_CH_MAX - RC_CH_MID);
        if(ratio > 1.0f) ratio = 1.0f;
        return max_out * ratio;
    }
}

//根据通道值判定三档开关位置: 0=低, 1=中, 2=高
uint8_t map_3pos_switch(int16_t raw)
{
    if(raw < SW_LOW_THRESH)  return 0;
    if(raw > SW_HIGH_THRESH) return 2;
    return 1;
}
