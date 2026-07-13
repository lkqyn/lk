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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_4
#define LED_R_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_5
#define LED_G_GPIO_Port GPIOE
#define ENCODER_A_Pin GPIO_PIN_0
#define ENCODER_A_GPIO_Port GPIOA
#define ENCODER_B_Pin GPIO_PIN_1
#define ENCODER_B_GPIO_Port GPIOA
#define CURR_A1_ADC_Pin GPIO_PIN_4
#define CURR_A1_ADC_GPIO_Port GPIOC
#define CURR_B1_ADC_Pin GPIO_PIN_5
#define CURR_B1_ADC_GPIO_Port GPIOC
#define INVCC1_ADC_Pin GPIO_PIN_0
#define INVCC1_ADC_GPIO_Port GPIOB
#define NTC1_ADC_Pin GPIO_PIN_1
#define NTC1_ADC_GPIO_Port GPIOB
#define PWM_BN1_Pin GPIO_PIN_14
#define PWM_BN1_GPIO_Port GPIOE
#define PWM_AP1_Pin GPIO_PIN_8
#define PWM_AP1_GPIO_Port GPIOA
#define PWM_AN1_Pin GPIO_PIN_9
#define PWM_AN1_GPIO_Port GPIOA
#define PWM_BP1_Pin GPIO_PIN_10
#define PWM_BP1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
