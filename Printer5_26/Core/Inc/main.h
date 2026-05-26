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
#define STEP_CH1_Pin GPIO_PIN_6
#define STEP_CH1_GPIO_Port GPIOE
#define DAC_SDA_Pin GPIO_PIN_0
#define DAC_SDA_GPIO_Port GPIOF
#define DAC_SCL_Pin GPIO_PIN_1
#define DAC_SCL_GPIO_Port GPIOF
#define DAC_RDY_Pin GPIO_PIN_2
#define DAC_RDY_GPIO_Port GPIOF
#define FR_CH1_Pin GPIO_PIN_5
#define FR_CH1_GPIO_Port GPIOF
#define EN_CH1_Pin GPIO_PIN_6
#define EN_CH1_GPIO_Port GPIOF
#define LOD_CH1_Pin GPIO_PIN_7
#define LOD_CH1_GPIO_Port GPIOF
#define MD0_CH1_Pin GPIO_PIN_8
#define MD0_CH1_GPIO_Port GPIOF
#define MD1_CH1_Pin GPIO_PIN_9
#define MD1_CH1_GPIO_Port GPIOF
#define MD2_CH1_Pin GPIO_PIN_10
#define MD2_CH1_GPIO_Port GPIOF
#define PRINT_REQ_Pin GPIO_PIN_0
#define PRINT_REQ_GPIO_Port GPIOG
#define PRINT_DONE_Pin GPIO_PIN_1
#define PRINT_DONE_GPIO_Port GPIOG
#define PRINT_DONE_OUT_Pin GPIO_PIN_15
#define PRINT_DONE_OUT_GPIO_Port GPIOE
#define STAN_Pin GPIO_PIN_12
#define STAN_GPIO_Port GPIOD
#define IN_CN19_1_Pin GPIO_PIN_14
#define IN_CN19_1_GPIO_Port GPIOD
#define IN_CN19_2_Pin GPIO_PIN_15
#define IN_CN19_2_GPIO_Port GPIOD
#define IN_CN19_3_Pin GPIO_PIN_2
#define IN_CN19_3_GPIO_Port GPIOG
#define IN_CN19_4_Pin GPIO_PIN_3
#define IN_CN19_4_GPIO_Port GPIOG
#define PRINT_REQ_IN_Pin GPIO_PIN_4
#define PRINT_REQ_IN_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
