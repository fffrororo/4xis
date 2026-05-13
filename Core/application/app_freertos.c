#include "app_freertos.h"

// flight task
void flight_task(void *pvParameters);
#define FLIGHT_TASK_STACK_SIZE 128 //栈空间最小推荐值
#define FLIGHT_TASK_PRIORITY 5
TaskHandle_t flight_task_handle;

/**
 * @brief  启动FreeRTOS task
 * @param  None
 * @retval None
 */
void app_freertos_start(void)
{
    //1.创建任务
    xTaskCreate(
                flight_task, 
                "flight_task", 
                FLIGHT_TASK_STACK_SIZE, 
                NULL, FLIGHT_TASK_PRIORITY, 
                &flight_task_handle
                );
    
    //2.启动调度器
    vTaskStartScheduler();
}

void flight_task(void *pvParameters)
{
    while(1)
    {
        //1.执行任务
        app_flight_get_euler_angles();
        //2.任务延时
        vTaskDelay(6);
    }
}
