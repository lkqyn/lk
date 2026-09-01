/*
 * easyModbus.h
 *
 *  Created on: 2021年7月12日
 *      Author: ylj
 */

#ifndef EASYMODBUS_EASYMODBUS_H_
#define EASYMODBUS_EASYMODBUS_H_

#include "stm32f1xx_hal.h"

void easymodbus_handler(void);

extern uint8_t gNewComRecv;
extern uint8_t recvBuffer[20];

extern uint16_t recvOffsetPulseCount;
extern uint16_t recvDistance, recvDistance1;
extern uint8_t recvOffsetdir;

#endif /* EASYMODBUS_EASYMODBUS_H_ */
