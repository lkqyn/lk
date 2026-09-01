/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-01     ylj       the first version
 */
#ifndef _USART2_H_
#define _USART2_H_

#include "stm32f1xx_hal.h"


void USART2_config(void);
void USART2_sendBuf(char *p,uint8_t len);
void USART2_sendBufDMA(char *p,uint8_t len);



#endif /* DRIVERS_USART2_USART2_H_ */

