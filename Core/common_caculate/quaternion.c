#include "quaternion.h"

/* ================= 四元数 ================= */

static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

/* ================= PI参数 ================= */

static float Kp = 0.8f;
static float Ki = 0.003f;

/* ================= 积分误差 ================= */

static float exInt = 0.0f;
static float eyInt = 0.0f;
static float ezInt = 0.0f;


/* ================= 宏定义 ================= */

#define GYRO_SCALE    16.4f      // ±2000dps
#define ACC_SCALE     16384      // ±2g

#define RAD_TO_DEG    57.2957795f
#define DEG_TO_RAD    0.0174533f


/* ================= invSqrt ================= */

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}


/* ================= 初始化 ================= */

void Quaternion_Init(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    exInt = 0.0f;
    eyInt = 0.0f;
    ezInt = 0.0f;
}


/* ================= 欧拉角解算 ================= */

void Quaternion_GetEuler(Gyro_Acc_struct *imu,
                         Euler_struct *euler,
                         float dt)
{
    float gx, gy, gz;
    float ax, ay, az;

    float norm;

    float vx, vy, vz;
    float ex, ey, ez;

    float qa, qb, qc;

    /* ========= 数据转换 ========= */

    gx = ((float)imu->gyro.gx / GYRO_SCALE) * DEG_TO_RAD;
    gy = ((float)imu->gyro.gy / GYRO_SCALE) * DEG_TO_RAD;
    gz = ((float)imu->gyro.gz / GYRO_SCALE) * DEG_TO_RAD;

    ax = (float)imu->acc.ax / ACC_SCALE;
    ay = (float)imu->acc.ay / ACC_SCALE;
    az = (float)imu->acc.az / ACC_SCALE;

    /* ========= 加速度归一化 ========= */

    norm = invSqrt(ax * ax + ay * ay + az * az);

    ax *= norm;
    ay *= norm;
    az *= norm;

    /* ========= 重力方向 ========= */

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* ========= 误差叉积 ========= */

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    /* ========= 积分 ========= */

    exInt += ex * Ki * dt;
    eyInt += ey * Ki * dt;
    ezInt += ez * Ki * dt;

    /* ========= PI修正 ========= */

    gx += Kp * ex + exInt;
    gy += Kp * ey + eyInt;
    gz += Kp * ez + ezInt;

    /* ========= 四元数更新 ========= */

    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    /* ========= 归一化 ========= */

    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    /* ========= 欧拉角 ========= */

    euler->roll =
        atan2f(
            2.0f * (q0 * q1 + q2 * q3),
            1.0f - 2.0f * (q1 * q1 + q2 * q2)
        ) * RAD_TO_DEG;

    euler->pitch =
        asinf(
            2.0f * (q0 * q2 - q3 * q1)
        ) * RAD_TO_DEG;

    euler->yaw =
        atan2f(
            2.0f * (q0 * q3 + q1 * q2),
            1.0f - 2.0f * (q2 * q2 + q3 * q3)
        ) * RAD_TO_DEG;
}