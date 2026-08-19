/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-07     ylj       the first version
 */

#include "delay/delay.h"

/**
 delay t us via software.
*/
void delay_us(uint16_t t)
{
    uint8_t i;
    for(;t>0;t--)
        for(i=50;i>0;i--);
}

/**
 delay t ms via software.
*/
void delay_ms(uint16_t t)
{
    for(;t>0;t--)
        delay_us(100);
}

/**
 simple delay
*/
void delay_short(uint16_t t)
{
    while(t--);
}

/**
 delay t ms and clear IWDG counter.
*/
void delay_ms_iwdg(uint16_t t)
{
    for(;t>0;t--)
    {
        delay_us(100);
        //IWDG_Feed();
    }
}
