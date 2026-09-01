/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-09     ylj       the first version
 */

#include "lv8731v/lv8731v.h"

LV8731Pin const motor[4] = {
        {{GPIOC,GPIO_PIN_9},{GPIOC,GPIO_PIN_8 },{GPIOC,GPIO_PIN_6},{GPIOC,GPIO_PIN_7}},
        {{GPIOD,GPIO_PIN_7},{GPIOC,GPIO_PIN_11},{GPIOD,GPIO_PIN_6},{GPIOC,GPIO_PIN_12}},
        {{GPIOB,GPIO_PIN_7},{GPIOB,GPIO_PIN_3},{GPIOB,GPIO_PIN_5},{GPIOB,GPIO_PIN_4}},
        {{GPIOE,GPIO_PIN_2},{GPIOE,GPIO_PIN_5},{GPIOE,GPIO_PIN_3},{GPIOE,GPIO_PIN_4}},
};
     //因引脚冲突，4个电机细分做出调整


/**
 * @brief LV8731V���ų�ʼ��
 */
void LV8731V_config(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;

    RCC->APB2ENR |= ((0x7f) << 2);          // enable GPIOA~GPIOG's clock

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL ;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    for(i=0;i<4;i++)
    {
        gpio.Pin = motor[i].M_EN.pin;
        HAL_GPIO_Init(motor[i].M_EN.GPIOx, &gpio);
        HAL_GPIO_WritePin(motor[i].M_EN.GPIOx, motor[i].M_EN.pin, GPIO_PIN_RESET); //关闭电机使能
    }

    for(i=0;i<4;i++)
    {
        gpio.Pin = motor[i].M_FR.pin;
        HAL_GPIO_Init(motor[i].M_FR.GPIOx, &gpio);
    }

    //电机0和2 不用配置细分  电机1和电机3 配置细分
    // 配置 M_MD1 引脚
    gpio.Pin = motor[1].M_MD1.pin;
    HAL_GPIO_Init(motor[1].M_MD1.GPIOx, &gpio);
    gpio.Pin = motor[3].M_MD1.pin;
    HAL_GPIO_Init(motor[3].M_MD1.GPIOx, &gpio);
    // 配置 M_MD2 引脚
    gpio.Pin = motor[1].M_MD2.pin;
    HAL_GPIO_Init(motor[1].M_MD2.GPIOx, &gpio);
    gpio.Pin = motor[3].M_MD2.pin;
	HAL_GPIO_Init(motor[3].M_MD2.GPIOx, &gpio);

}

/**
 * @brief LV8731Vʹ�ܿ���
 * @param channel- ѡ��ͨ�� 0-CH1; 1-ch2; 2-ch3; 3-ch4
 * @param en- 0-ʧ��; 1-ʹ��
 */
void LV8731V_cmd(uint8_t channel,uint8_t en)
{
    if(en)
    {
        HAL_GPIO_WritePin(motor[channel].M_EN.GPIOx, motor[channel].M_EN.pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(motor[channel].M_EN.GPIOx, motor[channel].M_EN.pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief ����LV8731Vϸ��
 * @param channel ����ͨ��
 *          @arg 0:CH1
 *          @arg 1:CH2
 *          @arg 2:CH3
 *          @arg 3:CH4
 * @param ms ϸ����
 *          @arg Full_Step:ȫ��
 *          @arg Half_Step:�벽
 *          @arg Quarter_Step:1/4ϸ��
 *          @arg MS_16:1/16ϸ��
 */
void LV8731V_setMicroStep(uint8_t channel,uint8_t ms)
{
    ms --;
    
    switch(ms)
    {
    case Full_Step:
        HAL_GPIO_WritePin(motor[channel].M_MD1.GPIOx, motor[channel].M_MD1.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[channel].M_MD2.GPIOx, motor[channel].M_MD2.pin, GPIO_PIN_RESET);
        break;
    case Half_Step:
        HAL_GPIO_WritePin(motor[channel].M_MD1.GPIOx, motor[channel].M_MD1.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor[channel].M_MD2.GPIOx, motor[channel].M_MD2.pin, GPIO_PIN_RESET);
        break;
    case Quarter_Step:
        HAL_GPIO_WritePin(motor[channel].M_MD1.GPIOx, motor[channel].M_MD1.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[channel].M_MD2.GPIOx, motor[channel].M_MD2.pin, GPIO_PIN_SET);
        break;
    case MS_16:
        HAL_GPIO_WritePin(motor[channel].M_MD1.GPIOx, motor[channel].M_MD1.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor[channel].M_MD2.GPIOx, motor[channel].M_MD2.pin, GPIO_PIN_SET);
        break;
    default:
        HAL_GPIO_WritePin(motor[channel].M_MD1.GPIOx, motor[channel].M_MD1.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor[channel].M_MD2.GPIOx, motor[channel].M_MD2.pin, GPIO_PIN_SET);
        break;

    }
}

void LV8731V_setCW(uint8_t channel,uint8_t cw)
{
    HAL_GPIO_WritePin(motor[channel].M_FR.GPIOx,motor[channel].M_FR.pin,(GPIO_PinState)cw);
}
