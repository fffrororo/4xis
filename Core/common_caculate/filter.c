#include "filter.h"

/**
 * @brief 低通滤波
 * @param  newValue 新数据
 *        preFilteredValue 滤波过的旧数据
 * @return 滤波结果
 */
int16_t Filter_LowPass(int16_t newValue, int16_t preFilteredValue)
{
    return alpha * newValue + (1 - alpha) * preFilteredValue;
}

/*
卡尔曼滤波实现：
（初始化）记录一组初始化状态值
（测量）获取当前一次的加速度=>方差(平均值)
（使用初始化的K值）得到当前一次的输出值=〉记录结果的方差（下一次测量之前）更新状态=>重新计算k增益系数
（测量）获取下一次的加速度=>方差(平均值)
（计算下一次的输出值）得到下一次的输出值=〉记录结果的方差
 ...
*/
/*卡尔曼滤波参数*/
KalmanFilter_Struct kfs[3] ={
        {0.02,0,0,0,0.001,0.543},
        {0.02,0,0,0,0.001,0.543},
        {0.02,0,0,0,0.001,0.543}};
/**
 * @brief  卡尔曼滤波
 * @param  *kf:卡尔曼滤波参数结构体指针
 * @param  input:输入值
 * @retval 滤波后的值
 */
double Filter_KalmanFilter(KalmanFilter_Struct *kf, double input)
{
    kf->Now_P = kf->LastP + kf->Q;
    kf->Kg= kf->Now_P / (kf->Now_P + kf->R);
    kf->out= kf->out + kf->Kg * (input - kf->out);
    kf->LastP = (1 - kf->Kg) * kf->Now_P;
    return kf->out;
}