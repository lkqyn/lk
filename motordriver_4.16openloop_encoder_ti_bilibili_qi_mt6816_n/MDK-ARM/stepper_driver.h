#ifndef __STEPPER_DRIVER_H
#define __STEPPER_DRIVER_H

#include <stdint.h>
//初始化
void Stepper_Init(void);
// 0=正向 1=反向
void Stepper_SetDir(uint8_t dir);       
//速度设置
void Stepper_SetStepDelayMs(uint16_t ms);  
void Stepper_StepOnce_Blocking(void);      // 先用阻塞版，确保转起来
void Stepper_Stop(void);
#endif
