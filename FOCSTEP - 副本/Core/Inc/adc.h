/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/*
 * ADC1 外设接口：常规 DMA 采集两相电流和母线电压；同步注入组由
 * CurrentSense 模块在电流环启动后单独配置。
 */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */

#define ADC_REGULAR_CHANNEL_COUNT    (3U)

typedef enum
{
  ADC_REGULAR_CURRENT_A = 0,
  ADC_REGULAR_CURRENT_B,
  ADC_REGULAR_BUS_VOLTAGE
} ADC_RegularChannel_t;

/* USER CODE END Private defines */

/* 配置 ADC1、DMA 和默认常规采样序列。 */
void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

/* 启动用于后台监控的 ADC 常规组 DMA。 */
uint8_t ADC_RegularStart(void);
/* 停止 ADC 常规组 DMA，供同步注入采样接管 ADC 前调用。 */
uint8_t ADC_RegularStop(void);
/* 读取一个常规通道最新 DMA 原始值。 */
uint16_t ADC_RegularGetRaw(ADC_RegularChannel_t channel);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
