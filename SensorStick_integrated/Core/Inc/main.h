/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define AIRSPEED_DRDY_Pin GPIO_PIN_3
#define AIRSPEED_DRDY_GPIO_Port GPIOC
#define AIRSPEED_DRDY_EXTI_IRQn EXTI3_IRQn
#define GYRO_CS_Pin GPIO_PIN_8
#define GYRO_CS_GPIO_Port GPIOC
#define ACCEL_CS_Pin GPIO_PIN_9
#define ACCEL_CS_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define INT_A_Pin GPIO_PIN_0
#define INT_A_GPIO_Port GPIOC
#define INT_A_EXTI_IRQn EXTI0_IRQn
#define INT_G_Pin GPIO_PIN_1
#define INT_G_GPIO_Port GPIOC
#define INT_G_EXTI_IRQn EXTI1_IRQn
#define BMI085a_NSS_Pin GPIO_PIN_5
#define BMI085a_NSS_GPIO_Port GPIOC
#define BMI085g_NSS_Pin GPIO_PIN_6
#define BMI085g_NSS_GPIO_Port GPIOC
#define BMI085_PS_Pin GPIO_PIN_8
#define BMI085_PS_GPIO_Port GPIOC
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
