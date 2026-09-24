/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/*
 * 定时器资源说明：TIM1=PWM，TIM2=编码器，TIM3_CH2=STEP 捕获，
 * TIM4_CC2=20 kHz 电流采样触发，TIM5=独立 4 kHz 外环。
 */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

/* 独立4kHz外环节拍；不得复用TIM4（TIM4用于20kHz电流采样触发）。 */
extern TIM_HandleTypeDef htim5;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* 初始化 TIM1 四路互补 PWM。 */
void MX_TIM1_Init(void);
/* 初始化 TIM2 AB 正交编码器接口。 */
void MX_TIM2_Init(void);
/* 初始化 TIM3_CH2 的 PC7 STEP 输入捕获与自由运行时间基。 */
void MX_TIM3_Init(void);
/* 初始化由 TIM1 同步的 20 kHz 电流采样触发。 */
void MX_TIM4_Init(void);
/* 初始化独立 4 kHz 外部脉冲位置控制节拍。 */
void MX_TIM5_Init(void);

/* 完成 PWM 输出 GPIO 的复用配置。 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
