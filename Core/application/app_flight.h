#ifndef __APP_FLIGHT_H_
#define __APP_FLIGHT_H_



#include "main.h"
#include "imu.h"
#include "filter.h"
#include "quaternion.h"
#include "pid.h"
#include "motor.h"
#include "app_receive.h"
#include "LED.h"


typedef enum{
    IDLE = 0,
    NORMAL,
    FIX_HEIGHT,
    FAIL,
}FLIGHT_STATE;
extern FLIGHT_STATE flight_state;

extern Motor_Struct left_top_motor;
extern Motor_Struct right_top_motor;
extern Motor_Struct left_bottom_motor;
extern Motor_Struct right_bottom_motor;


void app_flight_get_euler_angles(void);

void app_flight_pid_process(void);

void app_flight_motor_control(void);


#endif




