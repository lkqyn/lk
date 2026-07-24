/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-02     ylj       the first version
 */
#ifndef DRIVERS_LED_LED_H_
#define DRIVERS_LED_LED_H_

#include "stm32f1xx_hal.h"

void LED_config(void);
void LED_task(void *pv);


#endif /* DRIVERS_LED_LED_H_ */
