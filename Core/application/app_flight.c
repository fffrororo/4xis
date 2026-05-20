#include "app_flight.h"

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

//飞行状态
FLIGHT_STATE flight_state = IDLE;

//获取的imu数据
Gyro_Acc_struct imu_get_data = {0};
Euler_struct euler_angles = {0};
gyro_data last_gyro_data = {0};

//俯仰角pid控制
PID_Struct pitch_pid = {.kp = 0, .ki = 0, .kd = 0};
PID_Struct gyro_y_pid = {.kp = 0, .ki = 0, .kd = 0};//对应俯仰角内环

//横滚角pid控制
PID_Struct roll_pid = {.kp = 0, .ki = 0, .kd = 0};
PID_Struct gyro_x_pid = {.kp = 0, .ki = 0, .kd = 0};

//偏航角pid控制
PID_Struct yaw_pid = {.kp = 0, .ki = 0, .kd = 0};
PID_Struct gyro_z_pid = {.kp = 0, .ki = 0, .kd = 0};

//电机结构体
Motor_Struct left_top_motor = {.htim = &htim3, .channel = TIM_CHANNEL_1, .speed = 0};
Motor_Struct right_top_motor = {.htim = &htim2, .channel = TIM_CHANNEL_2, .speed = 0};
Motor_Struct left_bottom_motor = {.htim = &htim4, .channel = TIM_CHANNEL_4, .speed = 0};
Motor_Struct right_bottom_motor = {.htim = &htim1, .channel = TIM_CHANNEL_3, .speed = 0};

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
    //俯仰角
    //1.外环角度控制 —— 目标值来自遥控器
    pitch_pid.target = rc_data.pitch_target;
    pitch_pid.measure = euler_angles.pitch;

    //2.内环角速度控制
    gyro_y_pid.target = pitch_pid.output;
    gyro_y_pid.measure = imu_get_data.gyro.gy;

    //3.串级pid控制
    PID_Calc_Chain(&pitch_pid, &gyro_y_pid);

    //横滚角
    //1.外环角度控制 —— 目标值来自遥控器
    roll_pid.target = rc_data.roll_target;
    roll_pid.measure = euler_angles.roll;

    //2.内环角速度控制
    gyro_x_pid.target = roll_pid.output;
    gyro_x_pid.measure = imu_get_data.gyro.gx;

    //3.串级pid控制
    PID_Calc_Chain(&roll_pid, &gyro_x_pid);

    //偏航角
    //1.外环角度控制 —— 目标值来自遥控器
    yaw_pid.target = rc_data.yaw_target;
    yaw_pid.measure = euler_angles.yaw;

    //2.内环角速度控制
    gyro_z_pid.target = yaw_pid.output;
    gyro_z_pid.measure = imu_get_data.gyro.gz;

    //3.串级pid控制
    PID_Calc_Chain(&yaw_pid, &gyro_z_pid);
}

void app_flight_motor_control(void)
{
    switch (flight_state)
    {
    case IDLE:
        //一旦进入加锁状态所有电机速度都为0
        left_top_motor.speed = 0;
        right_top_motor.speed = 0;
        left_bottom_motor.speed = 0;
        right_bottom_motor.speed = 0;
        break;
    case NORMAL:
    {
        // 油门基值映射到 PWM 范围 (1000~2000us)
        uint16_t base_throttle = (uint16_t)(1000.0f + rc_data.throttle * 1000.0f);

        // 获取串级 PID 内环输出
        float pitch_out = gyro_y_pid.output;
        float roll_out  = gyro_x_pid.output;
        float yaw_out   = gyro_z_pid.output;

        // X 型四轴电机混控
        // M1 前左(CCW):  油门 + 俯仰 + 横滚 - 偏航
        // M2 前右(CW):   油门 + 俯仰 - 横滚 + 偏航
        // M3 后左(CW):   油门 - 俯仰 + 横滚 + 偏航
        // M4 后右(CCW):  油门 - 俯仰 - 横滚 - 偏航
        int16_t m1 = (int16_t)(base_throttle + pitch_out + roll_out - yaw_out);
        int16_t m2 = (int16_t)(base_throttle + pitch_out - roll_out + yaw_out);
        int16_t m3 = (int16_t)(base_throttle - pitch_out + roll_out + yaw_out);
        int16_t m4 = (int16_t)(base_throttle - pitch_out - roll_out - yaw_out);

        // 限幅到电调有效范围
        left_top_motor.speed     = (uint16_t)CLAMP(m1, 1000, 2000);
        right_top_motor.speed    = (uint16_t)CLAMP(m2, 1000, 2000);
        left_bottom_motor.speed  = (uint16_t)CLAMP(m3, 1000, 2000);
        right_bottom_motor.speed = (uint16_t)CLAMP(m4, 1000, 2000);
        break;
    }
    case FIX_HEIGHT:


        break;

    case FAIL:

        break;
    default:
        break;
    }

    //设置电机速度
    Motor_Setspeed(&left_top_motor);
    Motor_Setspeed(&right_top_motor);
    Motor_Setspeed(&left_bottom_motor);
    Motor_Setspeed(&right_bottom_motor);
}
