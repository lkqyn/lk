/*
#include <inBoardIO/sensorGPIO.h>
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           20230804
 * 2020-01-10     xxk       the first version
 */

#include "MdriverAlarmIO/mdriveralarmio.h"

static const GpioPin MdriverAlarmIO[] = {
        /* M0inuputAlar ~ M2inuputAlar */
        {GPIOB,GPIO_PIN_5},{GPIOB,GPIO_PIN_4},
};

void MDAIO_config(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;

    for(i=0;i<2;i++)
    {
        gpio.Pin = MdriverAlarmIO[i].pin;
        HAL_GPIO_Init(MdriverAlarmIO[i].GPIOx, &gpio);
    }
}


/**
 @brief 驱动报警信号输入信号
 @param -no-同时数量 delay_ en-软件延时
 @return 0-M0电机; 1-M2电机
*/
uint8_t MDAIO_getInput(uint8_t no, uint8_t delay_en)
{
    uint8_t rtl=1;
    if(HAL_GPIO_ReadPin(MdriverAlarmIO[no].GPIOx,MdriverAlarmIO[no].pin) == GPIO_PIN_RESET)
    {
    	if(delay_en)
    	{
    		delay_ms(CONF_SOFTWARE_WAIT_INPUT_TIME);
    	}
        if(HAL_GPIO_ReadPin(MdriverAlarmIO[no].GPIOx,MdriverAlarmIO[no].pin) == GPIO_PIN_RESET)
        {
            rtl = 0;
        }
    }
    return rtl;
}


