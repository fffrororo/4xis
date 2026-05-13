#include "imu.h"

/**************imu为mpu6050*****************/

/**
 * @brief 计算零偏数据
 * 
 * @param 无
 */
//零偏数据
int32_t acc_x_offset = 0;
int32_t acc_y_offset = 0;
int32_t acc_z_offset = 0;

int32_t gyro_x_offset = 0;
int32_t gyro_y_offset = 0;
int32_t gyro_z_offset = 0;
void imu_calculate_offset(void)
{
    acc_data new_acc = {0};
    acc_data last_acc = {0};
    uint16_t count = 0;

    imu_GetAcc(&last_acc);
    //1.等待静止,abs(new.acc-last.acc)<400达到100次
    while (count < 100)
    {    
        imu_GetAcc(&new_acc);
        if(abs(new_acc.ax - last_acc.ax) <400 && abs(new_acc.ay - last_acc.ay) <400 && abs(new_acc.az - last_acc.az) <400)
        {
            count++;
        }
        else
        {
            count = 0;
        }
        last_acc = new_acc;

        vTaskDelay(6);
    }
    //2.静止后计算零偏
    Gyro_Acc_struct gyro_acc_data = {0};
    int32_t acc_x_offset_sum = 0;
    int32_t acc_y_offset_sum = 0;
    int32_t acc_z_offset_sum = 0;

    int32_t gyro_x_offset_sum = 0;
    int32_t gyro_y_offset_sum = 0;
    int32_t gyro_z_offset_sum = 0;

    for(uint8_t i = 0; i<100; i++)
    {
        imu_GetGyro_Acc(&gyro_acc_data);
        acc_x_offset_sum += gyro_acc_data.acc.ax - 0;
        acc_y_offset_sum += gyro_acc_data.acc.ay - 0;
        acc_z_offset_sum += gyro_acc_data.acc.az - 16384;

        gyro_x_offset_sum += gyro_acc_data.gyro.gx - 0;
        gyro_y_offset_sum += gyro_acc_data.gyro.gy - 0;
        gyro_z_offset_sum += gyro_acc_data.gyro.gz - 0;

        vTaskDelay(6);
    }
    //取平均值
    acc_x_offset = acc_x_offset_sum / 100;
    acc_y_offset = acc_y_offset_sum / 100;
    acc_z_offset = acc_z_offset_sum / 100;
    gyro_x_offset = gyro_x_offset_sum / 100;
    gyro_y_offset = gyro_y_offset_sum / 100;
    gyro_z_offset = gyro_z_offset_sum / 100;
}

/**
 * @brief  Write a byte to the IMU
 * @param  reg: Register address
 * @param  data: Data to be written
 * @retval None
 */
void imu_Write_Reg(uint8_t reg, uint8_t data)
{
    //1. Write the data to the register
    HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

}

/**
 * @brief  Read a byte from the IMU
 * @param  reg: Register address
 * @param  data: Pointer to store the read data
 * @retval None
 */
void imu_Read_Reg(uint8_t reg, uint8_t *data)
{ 
    //1. Read the data from the register
    HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}
/**
 * @brief  Initialize the IMU
 * @param  None
 * @retval None
 */
void imu_init(void) 
{ 
    //1，重启芯片，重置寄存器的值 => 写电源管理器0x6B
    imu_Write_Reg(0x6B,0x80);
    uint8_t data;
    //2. 等待复位完成
    while(data != 0x40){
        imu_Read_Reg(0x6B,&data);

    }
    //唤醒mpu6050
    imu_Write_Reg(0x6B,0x00);
    //选择合适量程
    imu_Write_Reg(0x1B,3<<3);//角速度量程为2000
    imu_Write_Reg(0x1C,0x00);//加速度量程为2g
    //关闭中断使能
    imu_Write_Reg(0x38,0x00);
    //关闭用户控制寄存器，不使用fifo队列
    imu_Write_Reg(0x6A,0x00);
    //配置采样频率 => 默认1000Hz，
    //基本逻辑：采样率必须>=2倍后续数据使用频率，
    imu_Write_Reg(0x19,0x00);//采样频率为1000/1=1000Hz
    //配置DLPF（数字低通滤波器）
    imu_Write_Reg(0x1A,0x01);//配置DLPF为184Hz 188Hz
    //配置使用的系统时钟为添加ppl
    imu_Write_Reg(0x6B,0x01);
    //使能加速度角速度传感器
    imu_Write_Reg(0x6C,0x00);

}

/**
 * @brief  Get the gyroscope data
 * @param  gyro: Pointer to store the gyroscope data
 * @retval None
 */
void imu_GetGyro(gyro_data *gyro)
{
    //xyz顺序，高八位在前
    uint8_t hight = 0;
    uint8_t low = 0;
    //x轴
    imu_Read_Reg(0x43,&hight);
    imu_Read_Reg(0x44,&low);
    gyro->gx = (int16_t)((hight<<8)|low) / 32768.0f * 2000 - gyro_x_offset;
    //y轴
    imu_Read_Reg(0x45,&hight);
    imu_Read_Reg(0x46,&low);
    gyro->gy = (int16_t)((hight<<8)|low) / 32768.0f * 2000 - gyro_y_offset;
    //z轴
    imu_Read_Reg(0x47,&hight);
    imu_Read_Reg(0x48,&low);
    gyro->gz = (int16_t)((hight<<8)|low) / 32768.0f * 2000 - gyro_z_offset;

}
/**
 * @brief  Get the acceleration data
 * @param  acc: Pointer to store the acceleration data
 * @retval None
 */
void imu_GetAcc(acc_data *acc)
{
    //xyz顺序，高八位在前
    uint8_t hight = 0;
    uint8_t low = 0;
    //x轴
    imu_Read_Reg(0x3B,&hight);
    imu_Read_Reg(0x3C,&low);
    acc->ax = (int16_t)((hight<<8)|low) / 32768.0f * 2 - acc_x_offset;
    //y轴
    imu_Read_Reg(0x3D,&hight);
    imu_Read_Reg(0x3E,&low);
    acc->ay = (int16_t)((hight<<8)|low) / 32768.0f * 2 - acc_y_offset;
    //z轴
    imu_Read_Reg(0x3F,&hight);
    imu_Read_Reg(0x40,&low);
    acc->az = (int16_t)((hight<<8)|low) / 32768.0f * 2 - acc_z_offset;
}
/**
 * @brief  Get the gyroscope and acceleration data
 * @param  Gyro_Acc: Pointer to store the gyroscope and acceleration data
 * @retval None
 */
void imu_GetGyro_Acc(Gyro_Acc_struct *Gyro_Acc)
{
    //1.读取角速度
    imu_GetGyro(&Gyro_Acc->gyro);
    //2.读取加速度
    imu_GetAcc(&Gyro_Acc->acc);
}


