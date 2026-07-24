#ifndef _INPUTSCAN_H_
#define _INPUTSCAN_H_

#include "stm32f1xx_hal.h"

void IN_scan(void *pv);
void IN_refreshUICmd(uint8_t cmd);
void IN_exTrigger(void *pv);
void IN_checkMetal(void);

#endif
