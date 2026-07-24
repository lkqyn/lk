/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-07     ylj       the first version
 */
#ifndef DRIVERS_IIC_IIC_H_
#define DRIVERS_IIC_IIC_H_

#include "stm32f1xx_hal.h"

#define IIC4728_SDA_GPIOx       GPIOC
#define IIC4728_SDA_PIN         GPIO_PIN_2

#define IIC4728_SCL_GPIOx       GPIOC
#define IIC4728_SCL_PIN         GPIO_PIN_3

void IIC4728_Init(void);
void IIC4728_Start(void);
void IIC4728_Stop(void);
uint8_t IIC4728_Wait_Ack(void);
void IIC4728_Ack(void);
void IIC4728_NAck(void);
void IIC4728_Send_Byte(uint8_t txd);
uint8_t IIC4728_Read_Byte(unsigned char ack);

#endif /* DRIVERS_IIC_IIC_H_ */
