#ifndef __LED_TASK_H__
#define __LED_TASK_H__

#include "main.h"

void LED_Task_Init(void);
void LED_SetNormal(void);
void LED_SetFault(void);
void LED_SetAllOff(void);

#endif