#ifndef _PWM4_H_
#define _PWM4_H_

#include "stm32f1xx_hal.h"

void PWM4_config(void);
void PWM4_start(void);
void PWM4_stop(void);

void PWM4_setFrequent(uint32_t f);

extern volatile uint32_t PWM4_pulseCount;

#endif
