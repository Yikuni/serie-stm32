/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define GL9_Pin GPIO_PIN_14
#define GL9_GPIO_Port GPIOC
#define GL8_Pin GPIO_PIN_15
#define GL8_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOA
#define GH1_Pin GPIO_PIN_4
#define GH1_GPIO_Port GPIOA
#define GH2_Pin GPIO_PIN_5
#define GH2_GPIO_Port GPIOA
#define GH3_Pin GPIO_PIN_6
#define GH3_GPIO_Port GPIOA
#define GH4_Pin GPIO_PIN_7
#define GH4_GPIO_Port GPIOA
#define GL10_Pin GPIO_PIN_2
#define GL10_GPIO_Port GPIOB
#define GL1_Pin GPIO_PIN_12
#define GL1_GPIO_Port GPIOB
#define GL2_Pin GPIO_PIN_13
#define GL2_GPIO_Port GPIOB
#define GL3_Pin GPIO_PIN_14
#define GL3_GPIO_Port GPIOB
#define GL4_Pin GPIO_PIN_15
#define GL4_GPIO_Port GPIOB
#define GL5_Pin GPIO_PIN_8
#define GL5_GPIO_Port GPIOA
#define GL6_Pin GPIO_PIN_11
#define GL6_GPIO_Port GPIOA
#define GL7_Pin GPIO_PIN_12
#define GL7_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
