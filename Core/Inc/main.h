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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTORB_IN1_Pin GPIO_PIN_5
#define MOTORB_IN1_GPIO_Port GPIOE
#define MOTORB_IN2_Pin GPIO_PIN_6
#define MOTORB_IN2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOE
#define MOTORC_IN2_Pin GPIO_PIN_9
#define MOTORC_IN2_GPIO_Port GPIOE
#define MOTORC_IN1_Pin GPIO_PIN_11
#define MOTORC_IN1_GPIO_Port GPIOE
#define IMU_SCL_Pin GPIO_PIN_10
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_11
#define IMU_SDA_GPIO_Port GPIOB
#define SERVO_IN_Pin GPIO_PIN_6
#define SERVO_IN_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOA
#define MOTORB_H1_Pin GPIO_PIN_4
#define MOTORB_H1_GPIO_Port GPIOB
#define MOTORB_H2_Pin GPIO_PIN_5
#define MOTORB_H2_GPIO_Port GPIOB
#define MOTORC_H1_Pin GPIO_PIN_6
#define MOTORC_H1_GPIO_Port GPIOB
#define MOTORC_H2_Pin GPIO_PIN_7
#define MOTORC_H2_GPIO_Port GPIOB
#define USERBTN_Pin GPIO_PIN_0
#define USERBTN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
