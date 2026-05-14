#ifndef __APP_FLIGHT_H_
#define __APP_FLIGHT_H_



#include "main.h"
#include "imu.h"
#include "filter.h"
#include "quaternion.h"
#include "pid.h"
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif



void app_flight_get_euler_angles(void);

void app_flight_pid_process(void);

#ifdef __cplusplus
}
#endif
#endif




