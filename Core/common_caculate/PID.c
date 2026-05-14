#include "PID.h"

void PID_Calc(PID_Struct *pid)
{   
    //1.计算误差
    pid->error = pid->measure -pid->target;
    //2.计算积分误差
    pid->integral += pid->error;
    //3.计算微分误差
    if (pid->last_error == 0)
    {
        pid->last_error = pid->error;
    }
    float derivative = pid->error - pid->last_error;
    //4.计算PID
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * derivative;
    //5.保存误差
    pid->last_error = pid->error;
}

void PID_Calc_Chain(PID_Struct *out_pid, PID_Struct *in_pid)
{
    //1.计算外环
    PID_Calc(out_pid);
    //2.内环目标值 = 外环输出值
    in_pid->target = out_pid->output;
    //3.计算内环
    PID_Calc(in_pid);
}

