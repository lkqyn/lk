/*
#include <inBoardIO/sensorGPIO.h>
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-10     ylj       the first version
 */

#include "LV8731V/lv8731v.h"
#include "inboardio/inboardio.h"

static const GpioPin inBoardIO[] = {
        /* INPUT_1 ~ INPUT_12 */
        {GPIOD,GPIO_PIN_5},{GPIOD,GPIO_PIN_4},
        {GPIOC,GPIO_PIN_10},{GPIOA,GPIO_PIN_15},
        {GPIOD,GPIO_PIN_14},{GPIOD,GPIO_PIN_13},
        {GPIOD,GPIO_PIN_12},{GPIOD,GPIO_PIN_11},
        {GPIOE,GPIO_PIN_12},{GPIOE,GPIO_PIN_13},
        {GPIOE,GPIO_PIN_14},{GPIOE,GPIO_PIN_15},
        /* OUTPUT_1 ~ OUTPUT_8 */
        {GPIOE,GPIO_PIN_11},{GPIOE,GPIO_PIN_10},
        {GPIOE,GPIO_PIN_9},{GPIOE,GPIO_PIN_8},
        {GPIOE,GPIO_PIN_7},{GPIOB,GPIO_PIN_1},
        {GPIOC,GPIO_PIN_5},{GPIOC,GPIO_PIN_4}
};

void IBIO_config(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    
    for(i=12;i<=19;i++)
    {
        HAL_GPIO_WritePin(inBoardIO[i].GPIOx,inBoardIO[i].pin,GPIO_PIN_SET);
    }
    
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	for(i=12;i<20;i++)
	{
		gpio.Pin = inBoardIO[i].pin;
		HAL_GPIO_Init(inBoardIO[i].GPIOx, &gpio);
	}

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;

    for(i=0;i<12;i++)
    {
        gpio.Pin = inBoardIO[i].pin;
        HAL_GPIO_Init(inBoardIO[i].GPIOx, &gpio);
    }
}

/**
 @breif ����OUTPUT�����ƽ
 @param -no- ָ�����ͨ�� 1~8
 @param -level- �����ƽ
*/
void IBIO_setOutput(uint8_t no,uint8_t level)
{
    switch(level)
    {
        case 0:
            HAL_GPIO_WritePin(inBoardIO[no+11].GPIOx,inBoardIO[no+11].pin,GPIO_PIN_RESET);
            break;
        
        case 1:
            HAL_GPIO_WritePin(inBoardIO[no+11].GPIOx,inBoardIO[no+11].pin,GPIO_PIN_SET);
            break;
    }
}

/**
 @brief ���INPUT��ƽ
 @param -no- ָ������ͨ��
 @return 0-�͵�ƽ; 1-�ߵ�ƽ
*/
uint8_t IBIO_getInput(uint8_t no)
{
    uint8_t rtl=1;
//    if(HAL_GPIO_ReadPin(inBoardIO[no-1].GPIOx,inBoardIO[no-1].pin) == GPIO_PIN_RESET)
//    {
//#ifdef CONF_SOFTWARE_WAIT_INPUT_STABLE
//        delay_ms(CONF_SOFTWARE_WAIT_INPUT_TIME);
//#endif
//        if(HAL_GPIO_ReadPin(inBoardIO[no-1].GPIOx,inBoardIO[no-1].pin) == GPIO_PIN_RESET)
//        {
//            rtl = 0;
//        }
//    }
    if(HAL_GPIO_ReadPin(inBoardIO[no-1].GPIOx,inBoardIO[no-1].pin) == GPIO_PIN_RESET)
    {
        rtl = 0;
    }
    return rtl;
}


/**
 @brief ���OUTPUT��ƽ
 @param -no- ָ������ͨ��
 @return 0-�͵�ƽ; 1-�ߵ�ƽ
*/
uint8_t IBIO_getOutput(uint8_t no)
{
    uint8_t rtl=1;
    if(HAL_GPIO_ReadPin(inBoardIO[no+11].GPIOx,inBoardIO[no+11].pin) == GPIO_PIN_RESET)
    {
#ifdef CONF_SOFTWARE_WAIT_INPUT_STABLE
        delay_ms(CONF_SOFTWARE_WAIT_INPUT_TIME);
#endif
        if(HAL_GPIO_ReadPin(inBoardIO[no+11].GPIOx,inBoardIO[no+11].pin) == GPIO_PIN_RESET)
        {
            rtl = 0;
        }
    }
    return rtl;
}
