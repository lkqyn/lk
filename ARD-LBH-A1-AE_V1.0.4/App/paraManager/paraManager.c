/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-04     ylj       the first version
 * 2020-01-05     ylj       增加对0x7000地址段的支持
 * 2020-01-07     ylj       添加读取参数指令
 * 2020-01-10     ylj       添加写8位参数函数
 */

#include "paraManager/paraManager.h"
#include "motorctrl/motorctrl.h"

uint8_t ADDR_5000_L[ADDR5000_NUM];
uint8_t ADDR_5000_H[ADDR5000_NUM];
uint8_t ADDR_6000_L[ADDR6000_NUM];
uint8_t ADDR_6000_H[ADDR6000_NUM];
uint8_t ADDR_7000_L[ADDR7000_NUM];
uint8_t ADDR_7000_H[ADDR7000_NUM];

volatile SystemPara systemPara;
volatile ControlPara controlPara;
/**
 * @brief 获取指定地址的参数
 * @param addr 地址将从0x5000开始
 * @return
 */
uint16_t PARA_readParameter(uint16_t addr)
{
    uint16_t temp;

    if(addr >= 0x5000)
    {
        if(addr <= 0x5999)
        {
            addr -= 0x5000;
            temp = ADDR_5000_H[addr] << 8 | ADDR_5000_L[addr];
        }
        else if(addr <= 0x6999)
        {
            addr -= 0x6000;
            temp = ADDR_6000_H[addr] << 8 | ADDR_6000_L[addr];
        }
        else if(addr <= 0x7999)
        {
            addr -= 0x7000;
            temp = ADDR_7000_H[addr] << 8 | ADDR_7000_L[addr];
        }
    }
    return temp;
}

/**
 * @brief 写参数
 * @param addr 写入地址
 * @param data 写入数据
 */
void PARA_writeParameter(uint16_t addr,uint16_t data)
{
    if(addr >= 0x5000)
    {
        if(addr <= 0x5999)
        {
            addr -= 0x5000;
            
            ADDR_5000_H[addr] = data >> 8;
            ADDR_5000_L[addr] = data & 0xff;
            
            //MC_5000toPara(addr);
        }
        else if(addr <= 0x6999)
        {
            addr -= 0x6000;
            ADDR_6000_H[addr] = data >> 8;
            ADDR_6000_L[addr] = data & 0xff;
        }
        else if(addr <= 0x7999)
        {
            addr -= 0x7000;
            ADDR_7000_H[addr] = data >> 8;
            ADDR_7000_L[addr] = data & 0xff;
        }
    }
}

/**
 * @brief 按高低8位写参数
 * @param addr 待写地址
 * @param high 高8位
 * @param low 低8位
 */
void PARA_writeParameter8bit(uint16_t addr,uint8_t high,uint8_t low)
{
    if(addr >= 0x5000)
    {
        if(addr <= 0x5999)
        {
            addr -= 0x5000;
            ADDR_5000_H[addr] = high;
            ADDR_5000_L[addr] = low;
            
            MC_5000toPara(addr);
        }
        else if(addr <= 0x6999)
        {
            addr -= 0x6000;
            ADDR_6000_H[addr] = high;
            ADDR_6000_L[addr] = low;
        }
        else if(addr <= 0x7999)
        {
            addr -= 0x7000;
            ADDR_7000_H[addr] = high;
            ADDR_7000_L[addr] = low;
        }
    }
}

/**
 * @brief 设置系统参数密码
 * @param -which- 0-设置user密码; 1-设置vendor密码
 * @param -pw-
 */
void PARA_setPassword(uint8_t which,char *pw)
{
    uint8_t i;

    switch(which)
    {
    case 0:
        for(i=0;i<6;i++)
        {
            systemPara.userPassword[i] = pw[i];
        }
        break;

    case 1:
        for(i=0;i<6;i++)
        {
            systemPara.vendorPassword[i] = pw[i];
        }
        break;
        
    case 2:
        for(i=0;i<6;i++)
        {
            systemPara.superPassword[i] = pw[i];
        }
        break;
    }
}

void PARA_setPasswordInt(uint8_t which,uint32_t pw)
{
    uint8_t i;
    switch(which)
    {
    case 0:
        for(i=6;i>0;i--)
        {
            systemPara.userPassword [i-1] = pw % 10 + '0';
            pw /= 10;
        }
        break;

    case 1:
        for(i=6;i>0;i--)
        {
            systemPara.vendorPassword [i-1] = pw % 10 + '0';
            pw /= 10;
        }
        break;
    }
}

float PARA_read(uint8_t addr)
{
    float rtl=0;

    rtl = ((ADDR_5000_H[addr] << 8) + ADDR_5000_L[addr]) / 100.0;

    return rtl;
}
