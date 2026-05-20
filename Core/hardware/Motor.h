#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"


typedef struct
{
	TIM_HandleTypeDef *htim;
    uint16_t channel;
    uint16_t speed;
}Motor_Struct;

void Motor_Setspeed(Motor_Struct *motor);
void Motor_Init(Motor_Struct *motor);




#endif