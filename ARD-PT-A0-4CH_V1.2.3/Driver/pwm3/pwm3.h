#ifndef _PWM3_H_
#define _PWM3_H_

#include "stm32f1xx_hal.h"

void PWM3_config(void);
void PWM3_start(void);
void PWM3_stop(void);

void PWM3_setFrequent(uint32_t f);

extern volatile uint32_t PWM3_pulseCount;

#endif
