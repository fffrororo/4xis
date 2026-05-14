#include "app_freertos.h"
#include "app_receive.h"

// 飞行任务
void flight_task(void *pvParameters);
#define FLIGHT_TASK_STACK_SIZE 128 //栈空间最小推荐值
#define FLIGHT_TASK_PRIORITY 4
TaskHandle_t flight_task_handle;
#define FLIGHT_TASK_PERIOD_MS 6

//通信任务
void sbus_task(void *pvParameters);
#define SBUS_TASK_STACK_SIZE 128
#define SBUS_TASK_PRIORITY 3
TaskHandle_t sbus_task_handle;
#define SBUS_TASK_PERIOD_MS 6

//led 任务
void led_task(void *pvParameters);
#define LED_TASK_STACK_SIZE 128
#define LED_TASK_PRIORITY 2
TaskHandle_t led_task_handle;
#define LED_TASK_PERIOD_MS 6

/**
 * @brief  启动FreeRTOS task
 * @param  None
 * @retval None
 */
void app_freertos_start(void)
{
    //1.创建任务
    //flight_task
    xTaskCreate(
                flight_task, 
                "flight_task", 
                FLIGHT_TASK_STACK_SIZE, 
                NULL, 
                FLIGHT_TASK_PRIORITY, 
                &flight_task_handle
                );
    //sbus_task
    xTaskCreate(
                sbus_task, 
                "sbus_task", 
                SBUS_TASK_STACK_SIZE, 
                NULL, 
                SBUS_TASK_PRIORITY, 
                &sbus_task_handle
                );
    //led_task
    xTaskCreate(
                led_task, 
                "led_task", 
                LED_TASK_STACK_SIZE, 
                NULL, 
                LED_TASK_PRIORITY, 
                &led_task_handle
                );
    
    //2.启动调度器
    vTaskStartScheduler();
}

void flight_task(void *pvParameters)
{
    //获取对应基准时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    imu_init();
    while(1)
    {
        //1.执行任务
        app_flight_get_euler_angles();
        app_flight_pid_process();
        //2.任务延时
        vTaskDelayUntil(&xLastWakeTime, FLIGHT_TASK_PERIOD_MS);
    }
}

void sbus_task(void *pvParameters)
{
    //获取对应基准时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        //1.执行任务：从 DMA 缓冲区读取 SBUS 数据并处理为遥控目标值
        RC_Data_t *p_rc = app_receive_getdata();
        if (p_rc->updated) {
            // 遥控数据已更新，rc_data 可供 flight_task 直接引用
        }

        //2.任务延时
        vTaskDelayUntil(&xLastWakeTime, SBUS_TASK_PERIOD_MS);
    }
}

void led_task(void *pvParameters)
{
    //获取对应基准时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        //1.执行任务：LED 闪烁
        

        //2.任务延时
        vTaskDelayUntil(&xLastWakeTime, LED_TASK_PERIOD_MS);
    }
}
