#ifndef __PID_H__
#define __PID_H__ 

typedef struct
{
	float kp;
	float ki;
	float kd;

	float error;
	float last_error;
	float integral;          //积分误差
    float target;
    float output;
    float measure;

}PID_Struct;

void PID_Calc(PID_Struct *pid);

void PID_Calc_Chain(PID_Struct *out_pid, PID_Struct *in_pid);


#endif