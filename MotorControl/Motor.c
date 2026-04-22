#include "Motor.h"

/**
  * @brief  初始化电机
  * @param  motor:电机结构体指针
  * @retval None
  */
void Motor_Init(Motor_Struct *motor){
	HAL_TIM_PWM_Start(motor->htim, motor->channel);

}
/**
 * @brief  设置电机速度,使用电调，最大pwm为10%，最小为5%
 * @param motor:电机结构体指针
 * @retval None
 */
void Motor_Setspeed(Motor_Struct *motor){
	__HAL_TIM_SET_COMPARE(motor->htim, motor->channel, motor->speed);

}

