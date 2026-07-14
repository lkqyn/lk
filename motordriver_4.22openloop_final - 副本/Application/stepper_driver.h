#ifndef __DRIVER_EG2131_GPIO_H
#define __DRIVER_EG2131_GPIO_H

#include "motor_driver.h"
#include <stdint.h>
void Stepper_Init(void);
void Stepper_SetDir(uint8_t dir);          // 0=正向 1=反向
void Stepper_SetPWMPercent(uint8_t percent);
void Stepper_SetDelayMs(uint16_t ms);   // 如果你之前叫 Stepper_SetStepDelayMs，就保持一致
void Stepper_SetStepDelayMs(uint16_t ms);  // 先用慢速 50~150ms
void Stepper_StepOnce_Blocking(void);      // 先用阻塞版，确保转起来
void Stepper_Stop(void);

void Stepper_RunSteps_Accel(uint32_t steps,
                            uint16_t start_ms,
                            uint16_t target_ms,
                            uint16_t accel_every_n_steps);
#endif

