#ifndef __QUATERNION_H
#define __QUATERNION_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif
void Quaternion_Init(void);
void Quaternion_GetEuler(Gyro_Acc_struct *imu,
                         Euler_struct *euler,
                         float dt);

#ifdef __cplusplus
}
#endif
#endif