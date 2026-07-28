/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-05     ylj       the first version
 */
#ifndef APPLICATIONS_EVENTHANDLER_EVENTHANDLER_H_
#define APPLICATIONS_EVENTHANDLER_EVENTHANDLER_H_

#include "stm32f1xx_hal.h"
#include "paramanager/paramanager.h"

void EVENT_task(void *pv);
void startupTask(void);
void startupInit(void *pv);
void start_mainsongliao(void *pv);
void start_mainboliao(void *pv); 

typedef void (*EventCallback)(void * pv);

extern EventCallback eventCallback[ADDR6000_NUM];

#endif /* APPLICATIONS_EVENTHANDLER_EVENTHANDLER_H_ */
