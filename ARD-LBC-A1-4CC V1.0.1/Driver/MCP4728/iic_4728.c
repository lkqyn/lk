/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-07     ylj       the first version
 */

#include "mcp4728/iic_4728.h"
#include "delay/delay.h"

#define SET_SDA()       HAL_GPIO_WritePin(IIC4728_SDA_GPIOx, IIC4728_SDA_PIN, GPIO_PIN_SET)
#define SET_SCL()       HAL_GPIO_WritePin(IIC4728_SCL_GPIOx, IIC4728_SCL_PIN, GPIO_PIN_SET)
#define RESET_SDA()     HAL_GPIO_WritePin(IIC4728_SDA_GPIOx, IIC4728_SDA_PIN, GPIO_PIN_RESET)
#define RESET_SCL()     HAL_GPIO_WritePin(IIC4728_SCL_GPIOx, IIC4728_SCL_PIN, GPIO_PIN_RESET)
#define SDA_DATA        HAL_GPIO_ReadPin(IIC4728_SDA_GPIOx, IIC4728_SDA_PIN)

static void SDA_OUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStructure.Pin = IIC4728_SDA_PIN;
    HAL_GPIO_Init(IIC4728_SDA_GPIOx, &GPIO_InitStructure);
}

static void SDA_IN(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Pin = IIC4728_SDA_PIN;

    HAL_GPIO_Init(IIC4728_SDA_GPIOx,&GPIO_InitStructure);
}


void IIC4728_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStructure.Pin = IIC4728_SDA_PIN;
    HAL_GPIO_Init(IIC4728_SDA_GPIOx, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = IIC4728_SCL_PIN;
    HAL_GPIO_Init(IIC4728_SCL_GPIOx, &GPIO_InitStructure);

    HAL_GPIO_WritePin(IIC4728_SDA_GPIOx, IIC4728_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IIC4728_SCL_GPIOx, IIC4728_SCL_PIN, GPIO_PIN_SET);
}
//²úÉúIICÆðÊ¼ÐÅºÅ
void IIC4728_Start(void)
{
    SDA_OUT();     //sdaÏßÊä³ö
    SET_SDA();
    SET_SCL();
    delay_us(4);
    RESET_SDA();//START:when CLK is high,DATA change form high to low
    delay_us(4);
    RESET_SCL();//Ç¯×¡I2C×ÜÏß£¬×¼±¸·¢ËÍ»ò½ÓÊÕÊý¾Ý
}
//²úÉúIICÍ£Ö¹ÐÅºÅ
void IIC4728_Stop(void)
{
    SDA_OUT();//sdaÏßÊä³ö
    RESET_SCL();
    RESET_SDA();//STOP:when CLK is high DATA change form low to high
    delay_us(4);
    SET_SCL();
    SET_SDA();//·¢ËÍI2C×ÜÏß½áÊøÐÅºÅ
    delay_us(4);
}
//µÈ´ýÓ¦´ðÐÅºÅµ½À´
//·µ»ØÖµ£º1£¬½ÓÊÕÓ¦´ðÊ§°Ü
//        0£¬½ÓÊÕÓ¦´ð³É¹¦
uint8_t IIC4728_Wait_Ack(void)
{
    uint8_t ucErrTime=0;
    SDA_IN();      //SDAÉèÖÃÎªÊäÈë
    SET_SDA();delay_us(1);
    SET_SCL();delay_us(1);
    while(SDA_DATA)
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            IIC4728_Stop();
            return 1;
        }
    }
    RESET_SCL();//Ê±ÖÓÊä³ö0
    return 0;
}
//²úÉúACKÓ¦´ð
void IIC4728_Ack(void)
{
    RESET_SCL();
    SDA_OUT();
    RESET_SDA();
    delay_us(2);
    SET_SCL();
    delay_us(2);
    RESET_SCL();
}
//²»²úÉúACKÓ¦´ð
void IIC4728_NAck(void)
{
    RESET_SCL();
    SDA_OUT();
    SET_SDA();
    delay_us(2);
    SET_SCL();
    delay_us(2);
    RESET_SCL();
}

void IIC4728_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    RESET_SCL();//À­µÍÊ±ÖÓ¿ªÊ¼Êý¾Ý´«Êä
    for(t=0;t<8;t++)
    {
        //IIC4728_SDA=(txd&0x80)>>7;
        if((txd&0x80)>>7)
            SET_SDA();
        else
            RESET_SDA();
        txd<<=1;
        delay_us(2);   //¶ÔTEA5767ÕâÈý¸öÑÓÊ±¶¼ÊÇ±ØÐëµÄ
        SET_SCL();
        delay_us(2);
        RESET_SCL();
        delay_us(2);
    }
}
//¶Á1¸ö×Ö½Ú£¬ack=1Ê±£¬·¢ËÍACK£¬ack=0£¬·¢ËÍnACK
uint8_t IIC4728_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN();//SDAÉèÖÃÎªÊäÈë
    for(i=0;i<8;i++ )
    {
        RESET_SCL();
        delay_us(2);
        SET_SCL();
        receive<<=1;
        if(SDA_DATA)receive++;
        delay_us(1);
    }
    if (!ack)
        IIC4728_NAck();//·¢ËÍnACK
    else
        IIC4728_Ack(); //·¢ËÍACK
    return receive;
}
