/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "sbus.h"
#include "usart.h"
#include "tim.h"
#include "Motor.h"
#include "i2c.h"
#include "spi.h"
#include "dma.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RIGHT_0_Pin GPIO_PIN_1
#define RIGHT_0_GPIO_Port GPIOA
#define SBus_Pin GPIO_PIN_3
#define SBus_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define Laser_SCL_Pin GPIO_PIN_10
#define Laser_SCL_GPIO_Port GPIOB
#define Laser_SDA_Pin GPIO_PIN_11
#define Laser_SDA_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_14
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOB
#define SI_EN_Pin GPIO_PIN_8
#define SI_EN_GPIO_Port GPIOA
#define SI_IRQ_Pin GPIO_PIN_9
#define SI_IRQ_GPIO_Port GPIOA
#define RIGHT_1_Pin GPIO_PIN_10
#define RIGHT_1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_3
#define IMU_INT_GPIO_Port GPIOB
#define LEFT_0_Pin GPIO_PIN_4
#define LEFT_0_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_6
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_7
#define IMU_SDA_GPIO_Port GPIOB
#define LEFT_1_Pin GPIO_PIN_9
#define LEFT_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define USART2_RX_BUFFER_SIZE  26    // 接收缓冲区大小

extern uint8_t  USART2_DMA_RX_BUF[USART2_RX_BUFFER_SIZE];  // DMA接收缓冲区
extern uint8_t  USART2_RX_BUF[USART2_RX_BUFFER_SIZE];      // 数据处理缓冲区
extern uint16_t USART2_RX_LEN;                             // 一帧数据长度

// FreeRTOS 信号量（用于通知任务数据已收到）
extern osSemaphoreId_t uart2RxSemaphoreHandle;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
