#ifndef __ADC_TASK_H__
#define __ADC_TASK_H__


#include "main.h"
#include <stdint.h>

void ADC_Task_Init(void);
void ADC_Task_StartInjected(void);

// 校准/数据读取（给 main 里打印用）
uint8_t  ADC_Task_IsCalibrated(void);
uint16_t ADC_Task_OffA(void);
uint16_t ADC_Task_OffB(void);
uint16_t ADC_Task_RawA(void);
uint16_t ADC_Task_RawB(void);
int32_t  ADC_Task_DeltaA(void);
int32_t  ADC_Task_DeltaB(void);

// 给回调喂数据（在 HAL_ADCEx_InjectedConvCpltCallback 里调用）
void ADC_Task_OnInjected(uint16_t rawA, uint16_t rawB);

#endif

