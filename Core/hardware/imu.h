#ifndef __IMU_H
#define __IMU_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif
// MPU6050,从设备地址
#define IMU_ADDR 0x68
//读写地址
#define IMU_ADDR_WRITE (IMU_ADDR << 1)//0xD0
#define IMU_ADDR_READ ((IMU_ADDR << 1) | 1)//0xD1


//陀螺仪数据
typedef struct{
    int16_t gx;
    int16_t gy;
    int16_t gz;
} gyro_data;

typedef struct{
    int16_t ax;
    int16_t ay;
    int16_t az;
} acc_data;
//陀螺仪和加速度数据
typedef struct{
    gyro_data gyro;
    acc_data acc;
} Gyro_Acc_struct;
//解算得到欧拉角
typedef struct{
    float roll;
    float pitch;
    float yaw;
} Euler_struct;


void imu_init(void);
void imu_GetGyro(gyro_data *gyro);
void imu_GetAcc(acc_data *acc);
void imu_GetGyro_Acc(Gyro_Acc_struct *Gyro_Acc);


#ifdef __cplusplus
}
#endif
#endif