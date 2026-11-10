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

#define IIC_SDA_GPIOx       GPIOA
#define IIC_SDA_PIN         GPIO_PIN_12

#define IIC_SCL_GPIOx       GPIOA
#define IIC_SCL_PIN         GPIO_PIN_11

void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(unsigned char ack);

#endif /* DRIVERS_IIC_IIC_H_ */
