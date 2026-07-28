#ifndef _IWDG_H_
#define _IWDG_H_

#include "stm32f1xx_hal.h"

void IWDG_config(void);
void IWDG_task(void *pv);

#define IWDG_RELOAD()       IWDG->KR = IWDG_KEY_RELOAD

#endif
