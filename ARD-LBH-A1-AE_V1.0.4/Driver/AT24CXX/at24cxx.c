/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-07     ylj       the first version
 */

#include "at24cxx/at24cxx.h"
#include "iic/iic.h"
#include "delay/delay.h"

void AT24CXX_Init(void)
{
    IIC_Init();
}
/**
 * 在AT24CXX芯片读取一个字节的数据
 */
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr)
{
    uint8_t temp=0;
    IIC_Start();
    if(EE_TYPE>AT24C16)
    {
        IIC_Send_Byte(0XA0);       //发送写控制字节
        IIC_Wait_Ack();
        IIC_Send_Byte(ReadAddr>>8);//发送高地址
        IIC_Wait_Ack();
    }else IIC_Send_Byte(0XA0+((ReadAddr/256)<<1));   //发送器件地址0xA0,写入数据

    IIC_Wait_Ack();
    IIC_Send_Byte(ReadAddr%256);   //发送低地址
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(0XA1);           //进入接收模式
    IIC_Wait_Ack();
    temp=IIC_Read_Byte(0);
    IIC_Stop();//产生一个停止条件
    return temp;
}
/**
 * 向AT24CXX芯片中写入一个字节的数据
 */
void AT24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite)
{
    IIC_Start();
    if(EE_TYPE>AT24C16)
    {
        IIC_Send_Byte(0XA0);        //发送写控制字节
        IIC_Wait_Ack();
        IIC_Send_Byte(WriteAddr>>8);//发送高地址
    }else
    {
        IIC_Send_Byte(0XA0+((WriteAddr/256)<<1));   //发送器件地址0xA0，写入数据
    }
    IIC_Wait_Ack();
    IIC_Send_Byte(WriteAddr%256);   //发送低地址
    IIC_Wait_Ack();
    IIC_Send_Byte(DataToWrite);     // 发送数据
    IIC_Wait_Ack();
    IIC_Stop();// 产生一个停止条件
    delay_ms(10);
}

/**
 * 向AT24CXX芯片中写入定长的数据
 */
void AT24CXX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len)
{
    uint8_t t;
    for(t=0;t<Len;t++)
    {
        AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
    }
}

uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr,uint8_t Len)
{
    uint8_t t;
    uint32_t temp=0;
    for(t=0;t<Len;t++)
    {
        temp<<=8;
        temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1);
    }
    return temp;
}
/**
 * 芯片检测函数
 * 返回0 芯片正常 返回1 芯片检测失败
 */
uint8_t AT24CXX_Check(void)
{
    uint8_t temp;
    temp=AT24CXX_ReadOneByte(255);// 读取指定地址的数据
    if(temp==0X55)
    {
        return 0;
    }
    else
    {
        AT24CXX_WriteOneByte(255,0X55); // 写入指定地址的数据
        temp=AT24CXX_ReadOneByte(255);// 重新读取指定地址的数据
        if(temp==0X55)return 0;
    }
    return 1;
}

void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
    while(NumToRead)
    {
        *pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);
        NumToRead--;
    }
}

void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
{
    while(NumToWrite--)
    {
        AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
        WriteAddr++;
        pBuffer++;
    }
}
