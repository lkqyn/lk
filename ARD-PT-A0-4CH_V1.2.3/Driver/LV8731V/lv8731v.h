/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-09     ylj       the first version
 */
#ifndef DRIVERS_LV8731V_LV8731V_H_
#define DRIVERS_LV8731V_LV8731V_H_

#include "stm32f1xx_hal.h"

typedef struct _gpio_pin
{
    GPIO_TypeDef * GPIOx;
    uint16_t pin;
}GpioPin;

typedef struct _LV8731V_PIN
{
    GpioPin M_EN;
    GpioPin M_FR;
    GpioPin M_MD1;
    GpioPin M_MD2;
}LV8731Pin;

//typedef enum _LV8731V_MicroStep
//{
//    Full_Step,
//    Half_Step,
//    Quarter_Step,
//    MS_16
//}LV8731V_MicroStep;

#define     Full_Step       0
#define     Half_Step       1
#define     Quarter_Step    2
#define     MS_16           3

void LV8731V_cmd(uint8_t channel,uint8_t en);
void LV8731V_setMicroStep(uint8_t channel,uint8_t ms);
void LV8731V_config(void);
void LV8731V_setCW(uint8_t channel,uint8_t cw);

#endif /* DRIVERS_LV8731V_LV8731V_H_ */
