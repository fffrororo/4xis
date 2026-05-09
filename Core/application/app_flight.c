#include "app_flight.h"

Gyro_Acc_struct imu_get_data = {0};
Euler_struct euler_angles = {0};
gyro_data last_gyro_data = {0};
void app_flight_get_euler_angles(void) 
{ 
    imu_GetGyro_Acc(&imu_get_data);

    //1.低通滤波处理角速度
    imu_get_data.gyro.gx = Filter_LowPass(imu_get_data.gyro.gx, last_gyro_data.gx);
    imu_get_data.gyro.gy = Filter_LowPass(imu_get_data.gyro.gy, last_gyro_data.gy);
    imu_get_data.gyro.gz = Filter_LowPass(imu_get_data.gyro.gz, last_gyro_data.gz);

    last_gyro_data.gx = imu_get_data.gyro.gx;
    last_gyro_data.gy = imu_get_data.gyro.gy;
    last_gyro_data.gz = imu_get_data.gyro.gz;

    //2.卡尔曼滤波处理加速度
    imu_get_data.acc.ax = Filter_KalmanFilter(&kfs[0], imu_get_data.acc.ax);
    imu_get_data.acc.ay = Filter_KalmanFilter(&kfs[1], imu_get_data.acc.ay);
    imu_get_data.acc.az = Filter_KalmanFilter(&kfs[2], imu_get_data.acc.az);

    //3.四元数姿态解算解算欧拉角
    Quaternion_GetEuler(&imu_get_data, &euler_angles,0.006f);

}