/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-01     ylj       the first version
 */
#ifndef _USART1_H_
#define _USART1_H_

#include "stm32f1xx_hal.h"


void USART1_config(void);
void USART1_sendBuf(uint8_t *p,uint8_t len);
//void USART1_sendBufDMA(char *p,uint8_t len);



#endif /* DRIVERS_USART2_USART2_H_ */

