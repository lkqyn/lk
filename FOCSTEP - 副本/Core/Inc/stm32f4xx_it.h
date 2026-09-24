/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/*
 * 中断向量声明。
 * ADC 为 20 kHz 电流环入口；TIM3 为 STEP 捕获；TIM5 为 4 kHz 外环节拍。
 */

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
/* Cortex-M 系统异常处理入口。 */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
/* 1 ms HAL 系统节拍入口。 */
void SysTick_Handler(void);
/* ADC 注入转换完成后分发 20 kHz 电流环。 */
void ADC_IRQHandler(void);
/* TIM1 PWM 更新中断分发。 */
void TIM1_UP_TIM10_IRQHandler(void);
/* TIM2 编码器定时器中断分发。 */
void TIM2_IRQHandler(void);
/* TIM3 STEP 输入捕获及时间戳溢出分发。 */
void TIM3_IRQHandler(void);
/* TIM4 电流采样触发/4 kHz 外环相关中断分发。 */
void TIM4_IRQHandler(void);
/* TIM5 独立 4 kHz 外环节拍分发。 */
void TIM5_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void OTG_FS_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
