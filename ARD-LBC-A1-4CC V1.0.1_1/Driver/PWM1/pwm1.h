/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-08     ylj       the first version
 */
#ifndef DRIVERS_PWM1_PWM1_H_
#define DRIVERS_PWM1_PWM1_H_

#include "stm32f1xx_hal.h"

void PWM1_config(void);
void PWM1_start(void);
void PWM1_stop(void);
void PWM1_setFrequent(uint32_t f);

extern volatile uint32_t PWM1_pulseCount;

#endif /* DRIVERS_PWM1_PWM1_H_ */
