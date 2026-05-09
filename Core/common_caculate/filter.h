#ifndef __FILTER_H
#define __FILTER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PI 3.1415926535897932384626433832795
#define alpha 0.98

typedef struct
{
    float LastP;//上一时刻的状态方差（或协方差）
    float Now_P;//当前时刻的状态方差（或协方差）
    float out;// 滤波器的输出值，即估计的状态
    float Kg;//卡尔曼增益，用于调节预测值和测量值之间的权重
    float Q;//过程噪声的方差，反映系统模型的不确定性
    float R;//测量噪声的方差，反映测量过程的不确定性
}KalmanFilter_Struct;
extern KalmanFilter_Struct kfs[3];

int16_t Filter_LowPass(int16_t newValue, int16_t preFilteredValue);

double Filter_KalmanFilter(KalmanFilter_Struct *kf, double input);
    

#ifdef __cplusplus
}
#endif
#endif