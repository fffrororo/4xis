#include "app_flight.h"
#include "app_receive.h"

Gyro_Acc_struct imu_get_data = {0};
Euler_struct euler_angles = {0};
gyro_data last_gyro_data = {0};

//俯仰角pid控制
PID_Struct pitch_pid = {.kp = 0, .ki = 0, .kd = 0};
PID_Struct gyro_y_pid = {.kp = 0, .ki = 0, .kd = 0};//对应俯仰角内环


/**
 * @brief 获取姿态角
 * 
 */
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

void app_flight_pid_process(void)
{
    //1.外环角度控制 —— 目标值来自遥控器
    pitch_pid.target = rc_data.pitch_target;
    pitch_pid.measure = euler_angles.pitch;

    //2.内环角速度控制
    gyro_y_pid.target = pitch_pid.output;
    gyro_y_pid.measure = imu_get_data.gyro.gy;

    //3.串级pid控制
    PID_Calc_Chain(&pitch_pid, &gyro_y_pid);
}
