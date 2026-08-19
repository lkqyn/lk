/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-02     ylj       the first version
 */
#include "led/led.h"

void LED_config(void)
{
    GPIO_InitTypeDef gpio;

    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pin = GPIO_PIN_14;
    gpio.Pull = GPIO_NOPULL ;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOC,&gpio);

    GPIOC->ODR &= ~(GPIO_PIN_14);
}

void LED_task(void *pv)
{
    static uint8_t flag = 0;

	if(flag)
	{
		flag=0;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	}
	else
	{
		flag=1;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	}

}
