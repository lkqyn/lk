/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-07     ylj       the first version
 */
#ifndef DRIVERS_AT24CXX_AT24CXX_H_
#define DRIVERS_AT24CXX_AT24CXX_H_

#include "stm32f1xx_hal.h"

#define AT24C01     127
#define AT24C02     255
#define AT24C04     511
#define AT24C08     1023
#define AT24C16     2047
#define AT24C32     4095
#define AT24C64     8191
#define AT24C128    16383
#define AT24C256    32767
//Mini STM32¿ª·¢°åÊ¹ÓÃµÄÊÇ24c02£¬ËùÒÔ¶¨ÒåEE_TYPEÎªAT24C02
#define EE_TYPE AT24C02

uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr);                           //Ö¸¶¨µØÖ·¶ÁÈ¡Ò»¸ö×Ö½Ú
void AT24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite);        //Ö¸¶¨µØÖ·Ð´ÈëÒ»¸ö×Ö½Ú
void AT24CXX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len);//Ö¸¶¨µØÖ·¿ªÊ¼Ð´ÈëÖ¸¶¨³¤¶ÈµÄÊý¾Ý
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr,uint8_t Len);                   //Ö¸¶¨µØÖ·¿ªÊ¼¶ÁÈ¡Ö¸¶¨³¤¶ÈÊý¾Ý
void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);   //´ÓÖ¸¶¨µØÖ·¿ªÊ¼Ð´ÈëÖ¸¶¨³¤¶ÈµÄÊý¾Ý
void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);      //´ÓÖ¸¶¨µØÖ·¿ªÊ¼¶Á³öÖ¸¶¨³¤¶ÈµÄÊý¾Ý

uint8_t AT24CXX_Check(void);  //检查AT24CXX芯片是否正常工作

void AT24CXX_Init(void); //初始化IIC通信


#endif /* DRIVERS_AT24CXX_AT24CXX_H_ */
