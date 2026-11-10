/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-07     ylj       the first version
 */
#ifndef DRIVERS_DELAY_DELAY_H_
#define DRIVERS_DELAY_DELAY_H_

#include "stm32f1xx_hal.h"

void delay_us(uint16_t t);
void delay_ms(uint16_t t);
void delay_short(uint16_t t);
void delay_ms_iwdg(uint16_t t);


#endif /* DRIVERS_DELAY_DELAY_H_ */
