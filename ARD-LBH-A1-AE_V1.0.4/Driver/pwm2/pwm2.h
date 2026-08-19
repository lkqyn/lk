#ifndef _PWM2_H_
#define _PWM2_H_

#include "stm32f1xx_hal.h"

void PWM2_config(void);
void PWM2_start(void);
void PWM2_stop(void);
void PWM2_setFrequent(uint32_t f);

extern volatile uint32_t PWM2_pulseCount;

#endif
