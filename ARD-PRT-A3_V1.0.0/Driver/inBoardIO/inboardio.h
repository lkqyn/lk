/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-10     ylj       the first version
 */
#ifndef DRIVERS_INBOARDIO_SENSORGPIO_H_
#define DRIVERS_INBOARDIO_SENSORGPIO_H_

#include "stm32f1xx_hal.h"

void IBIO_config(void);
uint8_t IBIO_getInput(uint8_t no);
uint8_t IBIO_getOutput(uint8_t no);
void IBIO_setOutput(uint8_t no,uint8_t level);

            
#endif /* DRIVERS_INBOARDIO_SENSORGPIO_H_ */
