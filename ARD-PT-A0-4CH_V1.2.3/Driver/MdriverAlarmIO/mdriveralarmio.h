/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-10     ylj       the first version
 */
#ifndef DRIVERS_MDRIVERALARMIO_SENSORGPIO_H_
#define DRIVERS_MDRIVERALARMIO_SENSORGPIO_H_

#include "stm32f1xx_hal.h"
#include "LV8731V/lv8731v.h"
#include "sysconfig/sysconfig.h"
#include "delay/delay.h"



void MDAIO_config(void);
uint8_t MDAIO_getInput(uint8_t no, uint8_t delay_en);


            
#endif /* DRIVERS_MDRIVERALARMIO_SENSORGPIO_H_ */
