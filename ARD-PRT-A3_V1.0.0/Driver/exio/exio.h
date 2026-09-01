/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-10     ylj       the first version
 */
#ifndef DRIVERS_EXIO_INBOARDIO_H_
#define DRIVERS_EXIO_INBOARDIO_H_

#include "stm32f1xx_hal.h"

void EXIO_config(void);
void EXIO_setOutput(uint8_t no,uint8_t level);
uint8_t EXIO_getOutput(uint8_t no);
uint8_t EXIO_getInput(uint8_t no);

#endif /* DRIVERS_EXIO_INBOARDIO_H_ */
