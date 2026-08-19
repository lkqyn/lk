/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-02     ylj       the first version
 * 2020-01-08     ylj       检查参数是否超过范围，若超过范围则不保存参数，并跳转信息提醒
 */

#include "dwin/gui.h"
#include "usart2/usart2.h"
#include "paramanager/paramanager.h"
#include "storage/storage.h"
#include "motorctrl/motorctrl.h"
#include "queue/queue.h"
#include "eventHandler/eventHandler.h"
#include "userfunc/userfunc.h"
#include "string.h"

//#define         SEND2SCREEN(data,l)         USART2_sendBuf(data,l)
#define         SEND2SCREEN(data,l)         GUI_send(data,l)

char GUI_resetCmd[]={0x5a,0xa5,0x07,0x82,0x00,0x04,0x55,0xaa,0x5a,0xa5};
char GUI_switchPageCmd[]={0x5a,0xa5,0x07,0x82, 0x00, 0x84, 0x5a, 0x01, 0x00, 0x00};
char GUI_getCurrentPageNoCmd[] = {0x5A, 0xA5, 0x04, 0x83, 0x00,0x14, 0x01};

static void passwordSymbol(void);
static uint8_t isCmdLegality(char * cmd);

uint16_t pageNoBeforMessage;
uint16_t tempGMOffset=0;

uint32_t logout_count1 = 0;	//	自动登出
uint8_t resetFeederflag = 0;	//	恢复出厂设置计时标志位
uint64_t resetFeederTime = 0;	//	恢复出厂设置计时
uint8_t changeoverflag = 0; // 换料计时标志
uint64_t ChangeoverTime = 0;// 换料计时
typedef struct _sendStruct
{
    char data[100];
    uint8_t length;
}SendStruct;

/**
 @brief 触摸屏复位
*/
void GUI_reset(void)
{
    SEND2SCREEN(GUI_resetCmd, 10);
}

/**
 @brief 发送进程
 处理所有发送到触摸屏的数据.检测发送队列里有没有需要发送的指令
*/
void GUI_sendTask(void *pv)
{
	struct CommData data;

    while(QUEUE_get(&sendQueue,&data))
    {
		USART2_sendBuf(data.data,data.length);
    }
}

/**
 @brief 发送数据到触摸屏
 将待发送数据加入发送消息队列.
*/
void GUI_send(char *data,uint8_t length)
{
	struct CommData sendData;
    
    memcpy(sendData.data,data,length);
    sendData.length = length;
    
    QUEUE_add(&sendQueue, sendData);
//    rt_mq_send(&sendMQ,&send,sizeof(send));
}

/**
 * 切换显示页面
 * @param pageNo 页面序号
 */
void GUI_switchPage(uint16_t pageNo)
{

	if(systemPara.Language == 1)
		pageNo += 50;


    GUI_switchPageCmd[8] = pageNo >> 8;
    GUI_switchPageCmd[9] = pageNo & 0xff;

    SEND2SCREEN(GUI_switchPageCmd, 10);
}

/**
 @brief 切换页面显示
 @param pageNo 页面序号
 @note 该函数将直接发送切换指令, 而不经过发送进程. 主要用于相关进程还未启动时.
*/
void GUI_switchPageIsolate(uint16_t pageNo)
{

	if(systemPara.Language == 1)
		pageNo += 50;

    GUI_switchPageCmd[8] = pageNo >> 8;
    GUI_switchPageCmd[9] = pageNo & 0xff;

    USART2_sendBuf(GUI_switchPageCmd, 10);
}

/**
 * 发送显示数据到屏
 * @param addr 显示缓存区地址
 * @param data 显示的数据
 * @param len 显示数据的长度
 * @warning 仅支持ASCII显示
 */
void GUI_showText(uint16_t addr,char *data,uint8_t len)
{
    char sendBuf[100];
    uint8_t index = 3;
    uint8_t i;

    sendBuf[index++] = 0x82;

    sendBuf[index++] = addr >> 8;
    sendBuf[index++] = addr & 0xff;

    for(i=0;i<len;i++)
    {
        sendBuf[index++] = *data;
        data ++;
    }

    sendBuf[0] = 0x5a;
    sendBuf[1] = 0xa5;
    sendBuf[2] = index - 3;

    SEND2SCREEN(sendBuf, index);
}

/**
 * 发送显示数据到屏
 * @param addr 显示缓存区地址
 * @param data 显示的数据
 * @param len 显示数据的长度
 * @warning 仅支持ASCII显示
 * @note 仅应用于初始化阶段
 */
void GUI_showTextIsolate(uint16_t addr,char *data,uint8_t len)
{
    char sendBuf[40];
    uint8_t index = 3;
    uint8_t i;

    sendBuf[index++] = 0x82;

    sendBuf[index++] = addr >> 8;
    sendBuf[index++] = addr & 0xff;

    for(i=0;i<len;i++)
    {
        sendBuf[index++] = *data;
        data ++;
    }

    sendBuf[0] = 0x5a;
    sendBuf[1] = 0xa5;
    sendBuf[2] = index - 3;

    USART2_sendBuf(sendBuf, index);
}

/**
 * @brief 显示信息页面.
 * @param message 信息页显示的内容
 * @param len 信息字节长度
 * 该函数仅设置信息页显示内容，然后发送查询当前页序号指令，当GUI_task线程
 * 最大长度-46
 * 接收到当前页序号指令返回时，将跳转到消息页。
 */
void GUI_showMessage(char * message,uint8_t len)
{
    GUI_showText(INFO_DISP_ADDR,"                                                   ",46);
    GUI_showText(INFO_DISP_ADDR,message,len);
    SEND2SCREEN(GUI_getCurrentPageNoCmd,7);
}

/**
 设置主页消息框显示内容
*/
void GUI_mainMessageDisp(char * message,uint8_t len)
{
    GUI_showText(MAINPAGE_MESSAGE_ADDR,MAINPAGE_MESAGE_CLEAR);
    GUI_showText(MAINPAGE_MESSAGE_ADDR,message,len);
}

void GUI_mainMessageDispIsolate(char * message,uint8_t len)
{
    GUI_showTextIsolate(MAINPAGE_MESSAGE_ADDR,MAINPAGE_MESAGE_CLEAR);
    GUI_showTextIsolate(MAINPAGE_MESSAGE_ADDR,message,len);
}

/**
 * @brief 控制图标显示
 * @param addr 图标变量位置
 * @param no 显示图标编号，一般为0或1
 */
void GUI_iconShow(uint16_t addr,uint8_t no)
{
    char cmd[] = {0x5a,0xa5,0x05,0x82,00,00,00,00};

    cmd[4] = addr >> 8;
    cmd[5] = addr & 0xff;
    cmd[7] = no;

    SEND2SCREEN(cmd, 8);
}

/**
 * @brief 检查指令合法性
 * @param cmd
 * @return 0-不合法; 1-合法
 */
static uint8_t isCmdLegality(char * cmd)
{
    uint8_t rtl = 1;
    uint16_t address;
    uint16_t data;

    address = (cmd[4] << 8) | cmd[5];
    data = (cmd[7] << 8) | cmd[8];

    switch(address)
    {
	case 0x5000:
	case 0x5004:
	case 0x5015:
#if	MAXSPEED == 20000
        if(data > 20000)
        {
            char value[]= {0x4e,0x20};
            GUI_showText(address,value,2);
            if(systemPara.Language == 0)
            	OVER_RANGE_1_200();
            else if(systemPara.Language == 1)
            	EN_OVER_RANGE_1_200();
            data = 20000;
            if(address == 0x5000 )
            {
            	ADDR_5000_H[0] = 0x4e;
            	ADDR_5000_L[0] = 0x20;
            	STORAGE_saveSingalParameter(STORAGE_GIVENSPEED_ADDR,data);
            }
            else if(address == 0x5004 )
            {
            	ADDR_5000_H[4] = 0x4e;
            	ADDR_5000_L[4] = 0x20;
                STORAGE_saveSingalParameter(STORAGE_BOSPEED_ADDR,data);
            }
            else if(address == 0x5015 )
            {
            	ADDR_5000_H[21] = 0x4e;
            	ADDR_5000_L[21] = 0x20;
                STORAGE_saveSingalParameter(STORAGE_RSTSPEED_ADDR,data);
            }
            rtl = 0;
        }
#else
        if(data > 50000)//最大速度500
		{
			char value[]= {0x4e,0x20};
			GUI_showText(address,value,2);

            if(systemPara.Language == 0)
            	OVER_RANGE_1_500();
            else if(systemPara.Language == 1)
            	EN_OVER_RANGE_1_500();

			data = 50000;
			if(address == 0x5000 )
			{
				ADDR_5000_H[0] = 0xc3;
				ADDR_5000_L[0] = 0x50;
				STORAGE_saveSingalParameter(STORAGE_GIVENSPEED_ADDR,data);
			}
			else if(address == 0x5004 )
			{
				ADDR_5000_H[4] = 0xc3;
				ADDR_5000_L[4] = 0x50;
				STORAGE_saveSingalParameter(STORAGE_BOSPEED_ADDR,data);
			}
			else if(address == 0x5015 )
			{
				ADDR_5000_H[21] = 0xc3;
				ADDR_5000_L[21] = 0x50;
				STORAGE_saveSingalParameter(STORAGE_RSTSPEED_ADDR,data);
			}
			rtl = 0;
		}
#endif
        else if(data < 100)
        {
            char value[]= {0,0x64};
            GUI_showText(address,value,2);

            if(systemPara.Language == 0)
            	OVER_RANGE_1_200();
            else if(systemPara.Language == 1)
            	EN_OVER_RANGE_1_200();

            data = 100;
            if(address == 0x5000 )
            {
            	ADDR_5000_H[0] = 0;
            	ADDR_5000_L[0] = 0x64;
            	STORAGE_saveSingalParameter(STORAGE_GIVENSPEED_ADDR,data);
            }
            else if(address == 0x5004 )
            {
            	ADDR_5000_H[4] = 0;
            	ADDR_5000_L[4] = 0x64;
                STORAGE_saveSingalParameter(STORAGE_BOSPEED_ADDR,data);
            }
            else if(address == 0x5015 )
            {
            	ADDR_5000_H[21] = 0;
            	ADDR_5000_L[21] = 0x64;
                STORAGE_saveSingalParameter(STORAGE_RSTSPEED_ADDR,data);
            }
            rtl = 0;
        }
        break;
//	case 0x5015:
////		if(systemPara.isRSTAutoSongEnable == 1)
////		{
////			//比较送料速度和剥料速度
////			if(((ADDR_5000_H[0] << 8) | ADDR_5000_L[0]) > ((ADDR_5000_H[21] << 8) | ADDR_5000_L[21]));
////			{
////				char value[]= {ADDR_5000_H[0],ADDR_5000_L[0]};
//////				char value[]= {0,0};
////				GUI_showText(address,value,2);
////
////				OVER_RANGE1();
////				rtl = 0;
////			}
////		}
//        if(data > 20000)
//        {
//            char value[]= {0x4e,0x20};
//            GUI_showText(address,value,2);
//            OVER_RANGE();
//            rtl = 0;
//        }
//        else if(data < 100)
//        {
//            char value[]= {0,0x64};
//            GUI_showText(address,value,2);
//            OVER_RANGE();
//            rtl = 0;
//        }
//        break;

	case 0x5002:
    case 0x5006:
        if(data > 15000)
        {
            char value[]= {0x3a,0x98};
            GUI_showText(address,value,2);
            if(systemPara.Language == 0)
            	OVER_RANGE_1_150();
            else if(systemPara.Language == 1)
            	EN_OVER_RANGE_1_150();
            data = 15000;
            if(address == 0x5002 )
			{
				ADDR_5000_H[2] = 0x3a;
				ADDR_5000_L[2] = 0x98;
				STORAGE_saveSingalParameter(STORAGE_LETSPEED_ADDR,data);
			}
			else if(address == 0x5006 )
			{
				ADDR_5000_H[6] = 0x3a;
				ADDR_5000_L[6] = 0x98;
				STORAGE_saveSingalParameter(STORAGE_SHOUSPEED_ADDR,data);
			}
            rtl = 0;
        }
        else if(data < 100)
        {
            char value[]= {0,0x64};
            GUI_showText(address,value,2);
            if(systemPara.Language == 0)
            	OVER_RANGE_1_150();
            else if(systemPara.Language == 1)
            	EN_OVER_RANGE_1_150();
            data = 100;
            if(address == 0x5002 )
			{
				ADDR_5000_H[2] = 0;
				ADDR_5000_L[2] = 0x64;
				STORAGE_saveSingalParameter(STORAGE_LETSPEED_ADDR,data);
			}
			else if(address == 0x5006 )
			{
				ADDR_5000_H[6] = 0;
				ADDR_5000_L[6] = 0x64;
				STORAGE_saveSingalParameter(STORAGE_SHOUSPEED_ADDR,data);
			}
            rtl = 0;
        }
        break;

    case 0x5001://显示4位整数，一位小数，
	case 0x5003:
    case 0x5007:
    	if(data > 20000)
		{
			char value[]= {0x4e,0x20};
			GUI_showText(address,value,2);

            if(systemPara.Language == 0)
            	OVER_RANGE_2_2000();
            else if(systemPara.Language == 1)
            	EN_OVER_RANGE_2_2000();

            data = 20000;
            if(address == 0x5001 )
			{
				ADDR_5000_H[1] = 0x4e;
				ADDR_5000_L[1] = 0x20;
				STORAGE_saveSingalParameter(STORAGE_GIVENOFFSET_ADDR,data);
			}
			else if(address == 0x5003 )
			{
				ADDR_5000_H[3] = 0x4e;
				ADDR_5000_L[3] = 0x20;
				STORAGE_saveSingalParameter(STORAGE_LETOFFSET_ADDR,data);
			}
			else if(address == 0x5007 )
			{
				ADDR_5000_H[7] = 0x4e;
				ADDR_5000_L[7] = 0x20;
				STORAGE_saveSingalParameter(STORAGE_SHOUDISTANCE_ADDR,data);
			}
			rtl = 0;
		}
		else if(data < 20)
        {
            char value[]= {0x00,0x14};
            GUI_showText(address,value,2);

            if(systemPara.Language == 0)
            	OVER_RANGE_2_2000();
            else if(systemPara.Language == 1)
            	EN_OVER_RANGE_2_2000();

            data = 20;
            if(address == 0x5001 )
			{
				ADDR_5000_H[1] = 0;
				ADDR_5000_L[1] = 0x14;
				STORAGE_saveSingalParameter(STORAGE_GIVENOFFSET_ADDR,data);
			}
			else if(address == 0x5003 )
			{
				ADDR_5000_H[3] = 0;
				ADDR_5000_L[3] = 0x14;
				STORAGE_saveSingalParameter(STORAGE_LETOFFSET_ADDR,data);
			}
			else if(address == 0x5007 )
			{
				ADDR_5000_H[7] = 0;
				ADDR_5000_L[7] = 0x14;
				STORAGE_saveSingalParameter(STORAGE_SHOUDISTANCE_ADDR,data);
			}
            rtl = 0;
        }
        break;

    case 0x5012:
    case 0x5014:
        if(data > 50000)
        {
            char value[]= {0xC3,0x50};
            GUI_showText(address,value,2);

            if(systemPara.Language == 0)
            	OVER_RANGE_0_300();
            else if(systemPara.Language == 1)
            	EN_OVER_RANGE_0_300();

            data = 50000;
			if(address == 0x5012 )
			{
				ADDR_5000_H[18] = 0xC3;
				ADDR_5000_L[18] = 0x50;
				STORAGE_saveSingalParameter(STORAGE_GMDISTANCE_ADDR,data);
			}
			if(address == 0x5014 )
			{
				ADDR_5000_H[18] = 0xC3;
				ADDR_5000_L[18] = 0x50;
				STORAGE_saveSingalParameter(STORAGE_CHANGEOVER_ADDR,data);
			}
            rtl = 0;
        }
        break;
    case 0x5005://剥料长度1
    	if(data > 50000)
		{
		   char value[]= {0xc3,0x50};
		   GUI_showText(address,value,2);

           if(systemPara.Language == 0)
        	   OVER_RANGE_0_500();
           else if(systemPara.Language == 1)
        	   EN_OVER_RANGE_0_500();

		   data = 50000;
			if(address == 0x5005 )
			{
				ADDR_5000_H[5] = 0xc3;
				ADDR_5000_L[5] = 0x50;
				STORAGE_saveSingalParameter(STORAGE_BODISTANCE_ADDR,data);
			}
			rtl = 0;
		}
//    	else if(data < 100)
//		{
//		   char value[]= {0x01,0xf4};
//		   GUI_showText(address,value,2);
//
//           if(systemPara.Language == 0)
//           	OVER_RANGE_1_500();
//           else if(systemPara.Language == 1)
//           	EN_OVER_RANGE_1_500();
//
//		   data = 50000;
//			if(address == 0x5005 )
//			{
//				ADDR_5000_H[5] = 0x0;
//				ADDR_5000_L[5] = 0x64;
//				STORAGE_saveSingalParameter(STORAGE_BODISTANCE_ADDR,data);
//			}
//			rtl = 0;
//		}
		break;
    case 0x5013://剥料长度2
    case 0x5016://变速位置
    case 0x5018://慢速剥料位置
	   if(data > 50000)
	   {
		   char value[]= {0xc3,0x50};
		   GUI_showText(address,value,2);

           if(systemPara.Language == 0)
           	OVER_RANGE_0_500();
           else if(systemPara.Language == 1)
           	EN_OVER_RANGE_0_500();

           data = 50000;

           if(address == 0x5013 )
			{
				ADDR_5000_H[19] = 0xc3;
				ADDR_5000_L[19] = 0x50;
				STORAGE_saveSingalParameter(STORAGE_HCD2_ADDR,data);
			}
			else if(address == 0x5016 )
			{
				ADDR_5000_H[22] = 0xc3;
				ADDR_5000_L[22] = 0x50;
				STORAGE_saveSingalParameter(STORAGE_BS1POINT_ADDR ,data);
			}
			else if(address == 0x5018 )
			{
				ADDR_5000_H[24] = 0xc3;
				ADDR_5000_L[24] = 0x50;
				STORAGE_saveSingalParameter(STORAGE_LOWBODISTANCE_ADDR,data);
			}
		   rtl = 0;
	   }
	   break;

    case 0x5008://初始化速度
   	 if(data > 10000)
		{
			char value[]= {0x27,0x10};
			GUI_showText(address,value,2);
			if(systemPara.Language == 0)
				OVER_RANGE_1_100();
			else if(systemPara.Language == 1)
				EN_OVER_RANGE_1_100();
			data = 10000;
			if(address == 0x5008 )
			{
				ADDR_5000_H[8] = 0x27;
				ADDR_5000_L[8] = 0x10;
				STORAGE_saveSingalParameter(STORAGE_ORISPEED_ADDR,data);
			}
			rtl = 0;
		}
       else if(data < 100)
       {
           char value[] = {0x00,0x64};
           GUI_showText(address,value,2);

			if(systemPara.Language == 0)
				OVER_RANGE_1_100();
			else if(systemPara.Language == 1)
				EN_OVER_RANGE_1_100();

			data = 100;
			if(address == 0x5008 )
			{
				ADDR_5000_H[8] = 0;
				ADDR_5000_L[8] = 0x64;
				STORAGE_saveSingalParameter(STORAGE_ORISPEED_ADDR,data);
			}
           rtl = 0;
       }
        break;
    case 0x5017://慢速送料
    case 0x5019://慢速剥料
    	 if(data > 10000)
		{
			char value[]= {0x27,0x10};
			GUI_showText(address,value,2);
			if(systemPara.Language == 0)
				OVER_RANGE_1_100();
			else if(systemPara.Language == 1)
				EN_OVER_RANGE_1_100();
			data = 10000;
			if(address == 0x5017 )
			{
				ADDR_5000_H[23] = 0x27;
				ADDR_5000_L[23] = 0x10;
				STORAGE_saveSingalParameter(STORAGE_LOWGMSPEED_ADDR,data);
			}
			if(address == 0x5019 )
			{
				ADDR_5000_H[25] = 0x27;
				ADDR_5000_L[25] = 0x10;
				STORAGE_saveSingalParameter(STORAGE_LOWBO_SPEED_ADDR,data);
			}
			rtl = 0;
		}
        else if(data < 100)
        {
            char value[] = {0x00,0x64};
            GUI_showText(address,value,2);

			if(systemPara.Language == 0)
				OVER_RANGE_1_100();
			else if(systemPara.Language == 1)
				EN_OVER_RANGE_1_100();

			data = 100;
			if(address == 0x5008 )
			{
				ADDR_5000_H[8] = 0;
				ADDR_5000_L[8] = 0x64;
				STORAGE_saveSingalParameter(STORAGE_ORISPEED_ADDR,data);
			}
			if(address == 0x5017 )
			{
				ADDR_5000_H[23] = 0;
				ADDR_5000_L[23] = 0x64;
				STORAGE_saveSingalParameter(STORAGE_LOWGMSPEED_ADDR,data);
			}
			if(address == 0x5019 )
			{
				ADDR_5000_H[25] = 0;
				ADDR_5000_L[25] = 0x64;
				STORAGE_saveSingalParameter(STORAGE_LOWBO_SPEED_ADDR,data);
			}
            rtl = 0;
        }
        break;
        
    case 0x5009:
        if(data > 1000)
        {
            char value[] = {0x03,0xe8};
            GUI_showText(address,value,2);

			if(systemPara.Language == 0)
				OVER_RANGE_0_10();
			else if(systemPara.Language == 1)
				EN_OVER_RANGE_0_10();

			data = 1000;
			if(address == 0x5008 )
			{
				ADDR_5000_H[9] = 0x03;
				ADDR_5000_L[9] = 0xe8;
				STORAGE_saveSingalParameter(STORAGE_ORIOFFSET_ADDR,data);
			}
            rtl = 0;
        }
        else if(data < 0)
        {
            char value[] = {0x00,0x00};
            GUI_showText(address,value,2);

			if(systemPara.Language == 0)
				OVER_RANGE_0_10();
			else if(systemPara.Language == 1)
				EN_OVER_RANGE_0_10();

            rtl = 0;
        }
        break;
    
    case 0x500a:
    case 0x500e:
        if(data > 1000)
        {
            char value[] = {0x03,0xe7};
            GUI_showText(address,value,2);

			if(systemPara.Language == 0)
				OVER_RANGE_1_999();
			else if(systemPara.Language == 1)
				EN_OVER_RANGE_1_999();
			data = 999;
			if(address == 0x500a )
			{
				ADDR_5000_H[10] = 0x03;
				ADDR_5000_L[10] = 0xe7;
				STORAGE_saveSingalParameter(STORAGE_GIVENCUR_ADDR,data);
			}
			else if(address == 0x500e )
			{
				ADDR_5000_H[14] = 0x03;
				ADDR_5000_L[14] = 0xe7;
				STORAGE_saveSingalParameter(STORAGE_BOCUR_ADDR,data);
			}
            rtl = 0;
        }
        else if(data < 1)
        {
            char value[] = {0x00,0x01};
            GUI_showText(address,value,2);

			if(systemPara.Language == 0)
				OVER_RANGE_1_999();
			else if(systemPara.Language == 1)
				EN_OVER_RANGE_1_999();

			data = 1000;
			if(address == 0x500a )
			{
				ADDR_5000_H[10] = 0x00;
				ADDR_5000_L[10] = 0x01;
				STORAGE_saveSingalParameter(STORAGE_GIVENCUR_ADDR,data);
			}
			else if(address == 0x500e )
			{
				ADDR_5000_H[14] = 0x00;
				ADDR_5000_L[14] = 0x01;
				STORAGE_saveSingalParameter(STORAGE_BOCUR_ADDR,data);
			}
            rtl = 0;
        }
        break;

    case 0x500c:
    case 0x5010:
        if(data > 250)
        {
            char value[] = {0x00,0xfa};
            GUI_showText(address,value,2);

			if(systemPara.Language == 0)
				OVER_RANGE_01_25();
			else if(systemPara.Language == 1)
				EN_OVER_RANGE_01_25();

			data = 250;
			if(address == 0x500c )
			{
				ADDR_5000_H[10] = 0x00;
				ADDR_5000_L[10] = 0xfa;
				STORAGE_saveSingalParameter(STORAGE_LETCUR_ADDR,data);
			}
			else if(address == 0x5010 )
			{
				ADDR_5000_H[16] = 0x00;
				ADDR_5000_L[16] = 0xfa;
				STORAGE_saveSingalParameter(STORAGE_SHOUCUR_ADDR,data);
			}
            rtl = 0;
        }
        else if(data < 10)
        {
            char value[] = {0x00,0x0a};
            GUI_showText(address,value,2);
			if(systemPara.Language == 0)
				OVER_RANGE_01_25();
			else if(systemPara.Language == 1)
				EN_OVER_RANGE_01_25();
			data = 10;
			if(address == 0x500c )
			{
				ADDR_5000_H[10] = 0x00;
				ADDR_5000_L[10] = 0x0a;
				STORAGE_saveSingalParameter(STORAGE_LETCUR_ADDR,data);
			}
			else if(address == 0x5010 )
			{
				ADDR_5000_H[16] = 0x00;
				ADDR_5000_L[16] = 0xfa;
				STORAGE_saveSingalParameter(STORAGE_SHOUCUR_ADDR,data);
			}
            rtl = 0;
        }
        break;
        
    case 0x500b:
    case 0x500d:
    case 0x500f:
    case 0x5011:
        if(data > 4)
        {
            char value[] = {0x00,0x04};
            GUI_showText(address,value,2);
			if(systemPara.Language == 0)
				OVER_RANGE_1_4();
			else if(systemPara.Language == 1)
				EN_OVER_RANGE_1_4();
            data = 4;
			if(address == 0x500b )
			{
				ADDR_5000_H[11] = 0x00;
				ADDR_5000_L[11] = 0x04;
				STORAGE_saveSingalParameter(STORAGE_GIVENMS_ADDR,data);
			}
			else if(address == 0x500d )
			{
				ADDR_5000_H[13] = 0x00;
				ADDR_5000_L[13] = 0x04;
				STORAGE_saveSingalParameter(STORAGE_LETMS_ADDR,data);
			}
			else if(address == 0x500f )
			{
				ADDR_5000_H[15] = 0x00;
				ADDR_5000_L[15] = 0x04;
				STORAGE_saveSingalParameter(STORAGE_BOMS_ADDR,data);
			}
			else if(address == 0x5011 )
			{
				ADDR_5000_H[17] = 0x00;
				ADDR_5000_L[17] = 0x04;
				STORAGE_saveSingalParameter(STORAGE_SHOUMS_ADDR,data);
			}
            rtl = 0;
        }
        else if(data < 1)
        {
            char value[] = {0x0,0x01};
            GUI_showText(address,value,2);
			if(systemPara.Language == 0)
				OVER_RANGE_1_4();
			else if(systemPara.Language == 1)
				EN_OVER_RANGE_1_4();
            data = 1;
			if(address == 0x500b )
			{
				ADDR_5000_H[11] = 0x00;
				ADDR_5000_L[11] = 0x01;
				STORAGE_saveSingalParameter(STORAGE_GIVENMS_ADDR,data);
			}
			else if(address == 0x500d )
			{
				ADDR_5000_H[13] = 0x00;
				ADDR_5000_L[13] = 0x01;
				STORAGE_saveSingalParameter(STORAGE_LETMS_ADDR,data);
			}
			else if(address == 0x500f )
			{
				ADDR_5000_H[15] = 0x00;
				ADDR_5000_L[15] = 0x01;
				STORAGE_saveSingalParameter(STORAGE_BOMS_ADDR,data);
			}
			else if(address == 0x5011 )
			{
				ADDR_5000_H[17] = 0x00;
				ADDR_5000_L[17] = 0x01;
				STORAGE_saveSingalParameter(STORAGE_SHOUMS_ADDR,data);
			}
            rtl = 0;
        }
        break;
        
//	case 0x5013:
////	case 0x5014:
//		if(data > 0)
//		{
//			char value[] = {0x00,0x00};
//			GUI_showText(address,value,2);
//			UNUSED();
//			rtl = 0;
//		}
//		break;

    default:rtl=1;
    }

    return rtl;
}

static void passwordSymbol(void)
{
    GUI_showText(PASSWORD_DISP_ADDR,"******",6);
}

void GUI_initDisplay(void)
{
    uint8_t i;

    for(i=0;i<ADDR5000_NUM;i++)
    {
        GUI_sendWordIsolate(0x5000+i, ADDR_5000_H[i], ADDR_5000_L[i]);
    }

//    for(i=0;i<11;i++)//初始化输出按钮颜色
//    {
//    	GUI_sendWord(0x7660 + i - 1, 0, 1);
//    }
//
//    for(i=0;i<3;i++)//初始化输出按钮颜色
//    {
//    	GUI_sendWord(0x78d0 + i -1, 0, 1);
//    }

	if(systemPara.logginStatus == 11)
		GUI_showText(LOGGINSTATE_DISP_ADDR,"User    ",8);
	else if(systemPara.logginStatus == 12)
		GUI_showText(LOGGINSTATE_DISP_ADDR,"Vendor  ",8);
	else if(systemPara.logginStatus == 22)
		GUI_showText(LOGGINSTATE_DISP_ADDR,"Super   ",8);
	else if(systemPara.logginStatus < 11)
	{
		if(systemPara.Language == 0)
			GUI_showTextIsolate(LOGGINSTATE_DISP_ADDR,USERNAME_DISP_NAME,USERNAME_DISP_NAME_LEN);
		if(systemPara.Language == 1)
			GUI_showTextIsolate(LOGGINSTATE_DISP_ADDR,EN_USERNAME_DISP_NAME,EN_USERNAME_DISP_NAME_LEN);
	}

    if(systemPara.Language == 0)
    {
		// 输出页内部控制信号名称显示
		GUI_showTextIsolate(SIGNAL_NAME_1_ADDR,CONF_INNER_CTRL_NAME_1,CONF_INNER_CTRL_NAME_1_LEN);
		GUI_showTextIsolate(SIGNAL_NAME_2_ADDR,CONF_INNER_CTRL_NAME_2,CONF_INNER_CTRL_NAME_2_LEN);
		if(systemPara.isCylinderSensorEnable)
			GUI_showTextIsolate(SIGNAL_NAME_3_ADDR,CONF_INNER_CTRL_NAME_3,CONF_INNER_CTRL_NAME_3_LEN);
		else
			GUI_showTextIsolate(SIGNAL_NAME_3_ADDR,CONF_INNER_CTRL_NAME_03,CONF_INNER_CTRL_NAME_03_LEN);

		GUI_showTextIsolate(SIGNAL_NAME_4_ADDR,CONF_INNER_CTRL_NAME_4,CONF_INNER_CTRL_NAME_4_LEN);
		GUI_showTextIsolate(SIGNAL_NAME_5_ADDR,CONF_INNER_CTRL_NAME_5,CONF_INNER_CTRL_NAME_5_LEN);
		GUI_showTextIsolate(SIGNAL_NAME_6_ADDR,CONF_INNER_CTRL_NAME_6,CONF_INNER_CTRL_NAME_6_LEN);
		GUI_showTextIsolate(SIGNAL_NAME_7_ADDR,CONF_INNER_CTRL_NAME_7,CONF_INNER_CTRL_NAME_7_LEN);
		GUI_showTextIsolate(SIGNAL_NAME_8_ADDR,CONF_INNER_CTRL_NAME_8,CONF_INNER_CTRL_NAME_8_LEN);

		// 输出页与外部控制信号名称显示
		GUI_showTextIsolate(OUTPUT_1_NAME_ADDR,CONF_OUTER_CTRL_NAME_1,CONF_OUTER_CTRL_NAME_1_LEN);
		GUI_showTextIsolate(OUTPUT_2_NAME_ADDR,CONF_OUTER_CTRL_NAME_2,CONF_OUTER_CTRL_NAME_2_LEN);
		GUI_showTextIsolate(OUTPUT_3_NAME_ADDR,CONF_OUTER_CTRL_NAME_3,CONF_OUTER_CTRL_NAME_3_LEN);
		GUI_showTextIsolate(OUTPUT_4_NAME_ADDR,CONF_OUTER_CTRL_NAME_4,CONF_OUTER_CTRL_NAME_4_LEN);
		GUI_showTextIsolate(OUTPUT_5_NAME_ADDR,CONF_OUTER_CTRL_NAME_5,CONF_OUTER_CTRL_NAME_5_LEN);
		GUI_showTextIsolate(OUTPUT_6_NAME_ADDR,CONF_OUTER_CTRL_NAME_6,CONF_OUTER_CTRL_NAME_6_LEN);
		GUI_showTextIsolate(OUTPUT_7_NAME_ADDR,CONF_OUTER_CTRL_NAME_7,CONF_OUTER_CTRL_NAME_7_LEN);
		GUI_showTextIsolate(OUTPUT_8_NAME_ADDR,CONF_OUTER_CTRL_NAME_8,CONF_OUTER_CTRL_NAME_8_LEN);

		// 供料器与外部交互信号 EXI
		GUI_showTextIsolate(EXINPUT_00_NAME_ADDR,CONF_EXI0_NAME,CONF_EXI0_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_01_NAME_ADDR,CONF_EXI1_NAME,CONF_EXI1_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_02_NAME_ADDR,CONF_EXI2_NAME,CONF_EXI2_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_03_NAME_ADDR,CONF_EXI3_NAME,CONF_EXI3_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_04_NAME_ADDR,CONF_EXI4_NAME,CONF_EXI4_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_05_NAME_ADDR,CONF_EXI5_NAME,CONF_EXI5_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_06_NAME_ADDR,CONF_EXI6_NAME,CONF_EXI6_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_07_NAME_ADDR,CONF_EXI7_NAME,CONF_EXI7_NAME_LEN);

		// 版本信息页显示
		GUI_showTextIsolate(INFOR_DEVICE_NAME_ADDR,CONF_DEVICE_NAME,CONF_DEVICE_NAME_LEN);
		GUI_showTextIsolate(INFOR_DEVICE_NO_ADDR,CONF_DEVICE_NO,CONF_DEVICE_NO_LEN);
		GUI_showTextIsolate(INFOR_HARDWARE_VER_ADDR,CONF_HARDWARE_VER,CONF_HARDWARE_VER_LEN);
		GUI_showTextIsolate(INFOR_SOFTWARE_VER_ADDR,CONF_SOFTWARE_VER,CONF_SOFTWARE_VER_LEN);

		// 输入页通道名称显示
		GUI_showTextIsolate(INPUT_00_NAME_ADDR,CONF_INPUT_00_NAME,CONF_INPUT_00_NAME_LEN);
#if CONF_Fiber_VER == 0
		if(systemPara.isLowspeedSensorEnable)
		{
			GUI_showTextIsolate(INPUT_01_NAME_ADDR,CONF_INPUT_001_NAME,CONF_INPUT_001_NAME_LEN);
			GUI_showTextIsolate(INPUT_02_NAME_ADDR,CONF_INPUT_002_NAME,CONF_INPUT_002_NAME_LEN);
		}
		else if(systemPara.isUpSensorDownWichoutEnable)
	   {
			GUI_showTextIsolate(INPUT_01_NAME_ADDR,CONF_INPUT_0001_NAME,CONF_INPUT_0001_NAME_LEN);
			GUI_showTextIsolate(INPUT_02_NAME_ADDR,CONF_INPUT_0002_NAME,CONF_INPUT_0002_NAME_LEN);
	   }
		else {
			GUI_showTextIsolate(INPUT_01_NAME_ADDR,CONF_INPUT_01_NAME,CONF_INPUT_01_NAME_LEN);
			GUI_showTextIsolate(INPUT_02_NAME_ADDR,CONF_INPUT_02_NAME,CONF_INPUT_02_NAME_LEN);
		}
#else
		GUI_showTextIsolate(INPUT_01_NAME_ADDR,CONF_INPUT_01_NAME,CONF_INPUT_01_NAME_LEN);
		GUI_showTextIsolate(INPUT_02_NAME_ADDR,CONF_INPUT_02_NAME,CONF_INPUT_02_NAME_LEN);
#endif

		GUI_showTextIsolate(INPUT_03_NAME_ADDR,CONF_INPUT_03_NAME,CONF_INPUT_03_NAME_LEN);

		if(systemPara.isUpShouSensorEnable == 1 && systemPara.isUpShouEnable){
			GUI_showTextIsolate(INPUT_04_NAME_ADDR,CONF_INPUT_04_NAME,CONF_INPUT_04_NAME_LEN);
			GUI_showTextIsolate(INPUT_05_NAME_ADDR,CONF_INPUT_05_NAME,CONF_INPUT_05_NAME_LEN);
		}
		else if(systemPara.isUpShouSensorEnable == 2 && systemPara.isUpShouEnable){
			GUI_showTextIsolate(INPUT_04_NAME_ADDR,CONF_INPUT_004_NAME,CONF_INPUT_004_NAME_LEN);
			GUI_showTextIsolate(INPUT_05_NAME_ADDR,CONF_INPUT_05_NAME,CONF_INPUT_05_NAME_LEN);
		}
		else{
			GUI_showTextIsolate(INPUT_04_NAME_ADDR,CONF_INPUT_004_NAME,CONF_INPUT_004_NAME_LEN);
			GUI_showTextIsolate(INPUT_05_NAME_ADDR,CONF_INPUT_005_NAME,CONF_INPUT_005_NAME_LEN);
		}

#if		CONF_LET_VER == 0
		if(systemPara.isDownShouSensorEnable == 1 && systemPara.isDownShouEnable){
			GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_06_NAME,CONF_INPUT_06_NAME_LEN);
			GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_07_NAME,CONF_INPUT_07_NAME_LEN);
		}
		else if(systemPara.isDownShouSensorEnable == 2 && systemPara.isDownShouEnable){
			GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_006_NAME,CONF_INPUT_006_NAME_LEN);
			GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_07_NAME,CONF_INPUT_07_NAME_LEN);
		}
		else{
			GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_006_NAME,CONF_INPUT_006_NAME_LEN);
			GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_007_NAME,CONF_INPUT_007_NAME_LEN);
		}
#else
		if(systemPara.isAutoLetMetalEnable){
			GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_06_NAME,CONF_INPUT_06_NAME_LEN);
			GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_07_NAME,CONF_INPUT_07_NAME_LEN);
		}
		else{
			GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_006_NAME,CONF_INPUT_006_NAME_LEN);
			GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_007_NAME,CONF_INPUT_007_NAME_LEN);
		}
#endif
		if(systemPara.isCylinderSensorEnable){
			GUI_showTextIsolate(INPUT_08_NAME_ADDR,CONF_INPUT_08_NAME,CONF_INPUT_08_NAME_LEN);
			GUI_showTextIsolate(INPUT_09_NAME_ADDR,CONF_INPUT_09_NAME,CONF_INPUT_09_NAME_LEN);
		}
		else
		{
			GUI_showTextIsolate(INPUT_08_NAME_ADDR,CONF_INPUT_008_NAME,CONF_INPUT_008_NAME_LEN);
			GUI_showTextIsolate(INPUT_09_NAME_ADDR,CONF_INPUT_009_NAME,CONF_INPUT_009_NAME_LEN);
		}

		GUI_showTextIsolate(INPUT_10_NAME_ADDR,CONF_INPUT_10_NAME,CONF_INPUT_10_NAME_LEN);

		if(systemPara.isInEmergencyStopEnable){
			GUI_showTextIsolate(INPUT_11_NAME_ADDR,CONF_INPUT_011_NAME,CONF_INPUT_011_NAME_LEN);
		}
		else {
			GUI_showTextIsolate(INPUT_11_NAME_ADDR,CONF_INPUT_11_NAME,CONF_INPUT_11_NAME_LEN);
		}

		GUI_showTextIsolate(LANGUAGE_DISP_ADDR,EN_LANGUAGE_DISP_NAME,EN_LANGUAGE_DISP_NAME_LEN);

    }
    else if(systemPara.Language == 1)
    {
		// 输出页内部控制信号名称显示
		GUI_showTextIsolate(SIGNAL_NAME_1_ADDR,EN_CONF_INNER_CTRL_NAME_1,EN_CONF_INNER_CTRL_NAME_1_LEN);
		GUI_showTextIsolate(SIGNAL_NAME_2_ADDR,EN_CONF_INNER_CTRL_NAME_2,EN_CONF_INNER_CTRL_NAME_2_LEN);
		if(systemPara.isCylinderSensorEnable)
			GUI_showTextIsolate(SIGNAL_NAME_3_ADDR,EN_CONF_INNER_CTRL_NAME_3,EN_CONF_INNER_CTRL_NAME_3_LEN);
		else
			GUI_showTextIsolate(SIGNAL_NAME_3_ADDR,EN_CONF_INNER_CTRL_NAME_03,EN_CONF_INNER_CTRL_NAME_03_LEN);

		GUI_showTextIsolate(SIGNAL_NAME_4_ADDR,EN_CONF_INNER_CTRL_NAME_4,EN_CONF_INNER_CTRL_NAME_4_LEN);
		GUI_showTextIsolate(SIGNAL_NAME_5_ADDR,EN_CONF_INNER_CTRL_NAME_5,EN_CONF_INNER_CTRL_NAME_5_LEN);
		GUI_showTextIsolate(SIGNAL_NAME_6_ADDR,EN_CONF_INNER_CTRL_NAME_6,EN_CONF_INNER_CTRL_NAME_6_LEN);
		GUI_showTextIsolate(SIGNAL_NAME_7_ADDR,EN_CONF_INNER_CTRL_NAME_7,EN_CONF_INNER_CTRL_NAME_7_LEN);
		GUI_showTextIsolate(SIGNAL_NAME_8_ADDR,EN_CONF_INNER_CTRL_NAME_8,EN_CONF_INNER_CTRL_NAME_8_LEN);

		// 输出页与外部控制信号名称显示
		GUI_showTextIsolate(OUTPUT_1_NAME_ADDR,EN_CONF_OUTER_CTRL_NAME_1,EN_CONF_OUTER_CTRL_NAME_1_LEN);
		GUI_showTextIsolate(OUTPUT_2_NAME_ADDR,EN_CONF_OUTER_CTRL_NAME_2,EN_CONF_OUTER_CTRL_NAME_2_LEN);
		GUI_showTextIsolate(OUTPUT_3_NAME_ADDR,EN_CONF_OUTER_CTRL_NAME_3,EN_CONF_OUTER_CTRL_NAME_3_LEN);
		GUI_showTextIsolate(OUTPUT_4_NAME_ADDR,EN_CONF_OUTER_CTRL_NAME_4,EN_CONF_OUTER_CTRL_NAME_4_LEN);
		GUI_showTextIsolate(OUTPUT_5_NAME_ADDR,EN_CONF_OUTER_CTRL_NAME_5,EN_CONF_OUTER_CTRL_NAME_5_LEN);
		GUI_showTextIsolate(OUTPUT_6_NAME_ADDR,EN_CONF_OUTER_CTRL_NAME_6,EN_CONF_OUTER_CTRL_NAME_6_LEN);
		GUI_showTextIsolate(OUTPUT_7_NAME_ADDR,EN_CONF_OUTER_CTRL_NAME_7,EN_CONF_OUTER_CTRL_NAME_7_LEN);
		GUI_showTextIsolate(OUTPUT_8_NAME_ADDR,EN_CONF_OUTER_CTRL_NAME_8,EN_CONF_OUTER_CTRL_NAME_8_LEN);

		// 供料器与外部交互信号 EXI
		GUI_showTextIsolate(EXINPUT_00_NAME_ADDR,EN_CONF_EXI0_NAME,EN_CONF_EXI0_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_01_NAME_ADDR,EN_CONF_EXI1_NAME,EN_CONF_EXI1_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_02_NAME_ADDR,EN_CONF_EXI2_NAME,EN_CONF_EXI2_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_03_NAME_ADDR,EN_CONF_EXI3_NAME,EN_CONF_EXI3_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_04_NAME_ADDR,EN_CONF_EXI4_NAME,EN_CONF_EXI4_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_05_NAME_ADDR,EN_CONF_EXI5_NAME,EN_CONF_EXI5_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_06_NAME_ADDR,EN_CONF_EXI6_NAME,EN_CONF_EXI6_NAME_LEN);
		GUI_showTextIsolate(EXINPUT_07_NAME_ADDR,EN_CONF_EXI7_NAME,EN_CONF_EXI7_NAME_LEN);

		// 版本信息页显示
		GUI_showTextIsolate(INFOR_DEVICE_NAME_ADDR,EN_CONF_DEVICE_NAME,EN_CONF_DEVICE_NAME_LEN);
		GUI_showTextIsolate(INFOR_DEVICE_NO_ADDR,CONF_DEVICE_NO,CONF_DEVICE_NO_LEN);
		GUI_showTextIsolate(INFOR_HARDWARE_VER_ADDR,CONF_HARDWARE_VER,CONF_HARDWARE_VER_LEN);
		GUI_showTextIsolate(INFOR_SOFTWARE_VER_ADDR,CONF_SOFTWARE_VER,CONF_SOFTWARE_VER_LEN);

		// 输入页通道名称显示
		GUI_showTextIsolate(INPUT_00_NAME_ADDR,EN_CONF_INPUT_00_NAME,EN_CONF_INPUT_00_NAME_LEN);
#if CONF_Fiber_VER == 0
		if(systemPara.isLowspeedSensorEnable)
		{
			GUI_showTextIsolate(INPUT_01_NAME_ADDR,EN_CONF_INPUT_001_NAME,EN_CONF_INPUT_001_NAME_LEN);
			GUI_showTextIsolate(INPUT_02_NAME_ADDR,EN_CONF_INPUT_002_NAME,EN_CONF_INPUT_002_NAME_LEN);
		}
		else if(systemPara.isUpSensorDownWichoutEnable)
	   {
			GUI_showTextIsolate(INPUT_01_NAME_ADDR,EN_CONF_INPUT_0001_NAME,EN_CONF_INPUT_0001_NAME_LEN);
			GUI_showTextIsolate(INPUT_02_NAME_ADDR,EN_CONF_INPUT_0002_NAME,EN_CONF_INPUT_0002_NAME_LEN);
	   }
		else {
			GUI_showTextIsolate(INPUT_01_NAME_ADDR,EN_CONF_INPUT_01_NAME,EN_CONF_INPUT_01_NAME_LEN);
			GUI_showTextIsolate(INPUT_02_NAME_ADDR,EN_CONF_INPUT_02_NAME,EN_CONF_INPUT_02_NAME_LEN);
		}
#else
		GUI_showTextIsolate(INPUT_01_NAME_ADDR,EN_CONF_INPUT_01_NAME,EN_CONF_INPUT_01_NAME_LEN);
		GUI_showTextIsolate(INPUT_02_NAME_ADDR,EN_CONF_INPUT_02_NAME,EN_CONF_INPUT_02_NAME_LEN);
#endif
		GUI_showTextIsolate(INPUT_03_NAME_ADDR,EN_CONF_INPUT_03_NAME,EN_CONF_INPUT_03_NAME_LEN);

		if(systemPara.isUpShouSensorEnable == 1 && systemPara.isUpShouEnable){
			GUI_showTextIsolate(INPUT_04_NAME_ADDR,EN_CONF_INPUT_04_NAME,EN_CONF_INPUT_04_NAME_LEN);
			GUI_showTextIsolate(INPUT_05_NAME_ADDR,EN_CONF_INPUT_05_NAME,EN_CONF_INPUT_05_NAME_LEN);
		}
		else if(systemPara.isUpShouSensorEnable== 2 && systemPara.isUpShouEnable){
			GUI_showTextIsolate(INPUT_04_NAME_ADDR,EN_CONF_INPUT_004_NAME,EN_CONF_INPUT_004_NAME_LEN);
			GUI_showTextIsolate(INPUT_05_NAME_ADDR,EN_CONF_INPUT_05_NAME,EN_CONF_INPUT_05_NAME_LEN);
		}
		else{
			GUI_showTextIsolate(INPUT_04_NAME_ADDR,EN_CONF_INPUT_004_NAME,EN_CONF_INPUT_004_NAME_LEN);
			GUI_showTextIsolate(INPUT_05_NAME_ADDR,EN_CONF_INPUT_005_NAME,EN_CONF_INPUT_005_NAME_LEN);
		}
#if		CONF_LET_VER == 0
		if(systemPara.isDownShouSensorEnable == 1 && systemPara.isDownShouEnable){
			GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_06_NAME,EN_CONF_INPUT_06_NAME_LEN);
			GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_07_NAME,EN_CONF_INPUT_07_NAME_LEN);
		}
		else if(systemPara.isDownShouSensorEnable== 2 && systemPara.isDownShouEnable){
			GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_006_NAME,EN_CONF_INPUT_006_NAME_LEN);
			GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_07_NAME,EN_CONF_INPUT_07_NAME_LEN);
		}
		else{
			GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_006_NAME,EN_CONF_INPUT_006_NAME_LEN);
			GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_007_NAME,EN_CONF_INPUT_007_NAME_LEN);
		}
#else
		if(systemPara.isAutoLetMetalEnable){
			GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_06_NAME,CONF_INPUT_06_NAME_LEN);
			GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_07_NAME,CONF_INPUT_07_NAME_LEN);
		}
		else{
			GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_006_NAME,CONF_INPUT_006_NAME_LEN);
			GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_007_NAME,CONF_INPUT_007_NAME_LEN);
		}
#endif
		if(systemPara.isCylinderSensorEnable){
			GUI_showTextIsolate(INPUT_08_NAME_ADDR,EN_CONF_INPUT_08_NAME,EN_CONF_INPUT_08_NAME_LEN);
			GUI_showTextIsolate(INPUT_09_NAME_ADDR,EN_CONF_INPUT_09_NAME,EN_CONF_INPUT_09_NAME_LEN);
		}
		else
		{
			GUI_showTextIsolate(INPUT_08_NAME_ADDR,EN_CONF_INPUT_008_NAME,EN_CONF_INPUT_008_NAME_LEN);
			GUI_showTextIsolate(INPUT_09_NAME_ADDR,EN_CONF_INPUT_009_NAME,EN_CONF_INPUT_009_NAME_LEN);
		}

		GUI_showTextIsolate(INPUT_10_NAME_ADDR,EN_CONF_INPUT_10_NAME,EN_CONF_INPUT_10_NAME_LEN);

		if(systemPara.isInEmergencyStopEnable){
			GUI_showTextIsolate(INPUT_11_NAME_ADDR,EN_CONF_INPUT_011_NAME,EN_CONF_INPUT_011_NAME_LEN);
		}
		else {
			GUI_showTextIsolate(INPUT_11_NAME_ADDR,EN_CONF_INPUT_11_NAME,EN_CONF_INPUT_11_NAME_LEN);
		}

		GUI_showTextIsolate(LANGUAGE_DISP_ADDR,LANGUAGE_DISP_NAME,LANGUAGE_DISP_NAME_LEN);
    }

    // 清除主页消息框显示
    GUI_showTextIsolate(MAINPAGE_MESSAGE_ADDR,MAINPAGE_MESAGE_CLEAR);
    
    // 补偿模式选择
//    if(MOTOR_offsetOrigin == 1)
//    {
//        GUI_sendWordIsolate(SINGAL_SELECT_1_ADDR,0,1);
//        GUI_sendWordIsolate(SINGAL_SELECT_2_ADDR,0,0);
//    }
//    else
//    {
//        GUI_sendWordIsolate(SINGAL_SELECT_1_ADDR,0,0);
//        GUI_sendWordIsolate(SINGAL_SELECT_2_ADDR,0,1);
//    }
    
    GUI_sendWordIsolate(0,0,1);
    
    //选中停止
    if(systemPara.Initstatus == 0)
    {
		GUI_sendWordIsolate(STOP_FLAG_ADDR,0,1);
		GUI_sendWordIsolate(INIT_FLAG_ADDR,0,0);
    }
    else
    {
    	GUI_sendWordIsolate(STOP_FLAG_ADDR,0,0);
    	GUI_sendWordIsolate(INIT_FLAG_ADDR,0,1);
    }
    
    //下收料传感器选择模式
    if(systemPara.isDownShouSensorEnable == 1)
    {
        GUI_sendWord(SINGAL_DUAL_SENSOR_DOWN_ADDR,0,1);
        GUI_sendWord(SINGAL_SINGLE_SENSOR_DOWN_ADDR,0,0);
        GUI_sendWord(SINGAL_NOT_SENSOR_DOWN_ADDR,0,0);
    }
    else if(systemPara.isDownShouSensorEnable == 2)
    {
        GUI_sendWord(SINGAL_DUAL_SENSOR_DOWN_ADDR,0,0);
        GUI_sendWord(SINGAL_SINGLE_SENSOR_DOWN_ADDR,0,1);
        GUI_sendWord(SINGAL_NOT_SENSOR_DOWN_ADDR,0,0);
    }
    else
    {
        GUI_sendWord(SINGAL_DUAL_SENSOR_DOWN_ADDR,0,0);
        GUI_sendWord(SINGAL_SINGLE_SENSOR_DOWN_ADDR,0,0);
        GUI_sendWord(SINGAL_NOT_SENSOR_DOWN_ADDR,0,1);
    }

    //上收料传感器选择模式
    if(systemPara.isUpShouSensorEnable == 1)
    {
        GUI_sendWord(SINGAL_DUAL_SENSOR_UP_ADDR,0,1);
        GUI_sendWord(SINGAL_SINGLE_SENSOR_UP_ADDR,0,0);
        GUI_sendWord(SINGAL_NOT_SENSOR_UP_ADDR,0,0);
    }
    else if(systemPara.isUpShouSensorEnable == 2)
	{
		GUI_sendWord(SINGAL_DUAL_SENSOR_UP_ADDR,0,0);
		GUI_sendWord(SINGAL_SINGLE_SENSOR_UP_ADDR,0,1);
		GUI_sendWord(SINGAL_NOT_SENSOR_UP_ADDR,0,0);
	}
    else
    {
		GUI_sendWord(SINGAL_DUAL_SENSOR_UP_ADDR,0,0);
		GUI_sendWord(SINGAL_SINGLE_SENSOR_UP_ADDR,0,0);
		GUI_sendWord(SINGAL_NOT_SENSOR_UP_ADDR,0,1);
    }

    // 到位感应器选择显示
    if(systemPara.sensorChosen == 1)
    {
        GUI_sendWordIsolate(SINGAL_SENSOR_DOWN_ADDR,0,1);
        GUI_sendWordIsolate(SINGAL_SENSOR_UP_ADDR,0,0);
        GUI_sendWordIsolate(SINGAL_SENSOR_NO_ADDR,0,0);
    }
    else if(systemPara.sensorChosen == 2)
    {
        GUI_sendWordIsolate(SINGAL_SENSOR_DOWN_ADDR,0,0);
        GUI_sendWordIsolate(SINGAL_SENSOR_UP_ADDR,0,1);
        GUI_sendWordIsolate(SINGAL_SENSOR_NO_ADDR,0,0);
    }
    else if(systemPara.sensorChosen == 3)
    {
        GUI_sendWordIsolate(SINGAL_SENSOR_DOWN_ADDR,0,0);
        GUI_sendWordIsolate(SINGAL_SENSOR_UP_ADDR,0,0);
        GUI_sendWordIsolate(SINGAL_SENSOR_NO_ADDR,0,1);
    }
    
    // 送料模式选择显示
    if(systemPara.givenMode == 1)
    {
        GUI_sendWordIsolate(SINGAL_GIVENMODE_N,0,1);
        GUI_sendWordIsolate(SINGAL_GIVENMODE_Y,0,0);
    }
    else
    {
        GUI_sendWordIsolate(SINGAL_GIVENMODE_N,0,0);
        GUI_sendWordIsolate(SINGAL_GIVENMODE_Y,0,1);
    }
    
    // 根据status的状态，设置工作状态显示
    switch(systemPara.status)
    {
    	case 1://联机模式
    		GUI_sendWordIsolate(AUTOPRODUCT_FLAG_ADDR,0,1);
			GUI_sendWordIsolate(MANUALPRODUCT_FALG_ADDR,0,0);
			GUI_sendWordIsolate(FREERUN_FLAG_ADDR,0,0);
    	break;
    	case 2://自动模式
    		GUI_sendWordIsolate(AUTOPRODUCT_FLAG_ADDR,0,0);
			GUI_sendWordIsolate(MANUALPRODUCT_FALG_ADDR,0,1);
			GUI_sendWordIsolate(FREERUN_FLAG_ADDR,0,0);
		break;
    	case 3://空跑模式
			GUI_sendWordIsolate(AUTOPRODUCT_FLAG_ADDR,0,0);
			GUI_sendWordIsolate(MANUALPRODUCT_FALG_ADDR,0,0);
			GUI_sendWordIsolate(FREERUN_FLAG_ADDR,0,1);
		break;
    }

	GUI_sendWordIsolate(FUNCTION_CHOSE_1_ADDR, 0, systemPara.isAutoLetMetalEnable);
	GUI_sendWordIsolate(FUNCTION_CHOSE_2_ADDR, 0, systemPara.isUpShouEnable);
	GUI_sendWordIsolate(FUNCTION_CHOSE_3_ADDR, 0, systemPara.isDownShouEnable);
	GUI_sendWordIsolate(FUNCTION_CHOSE_4_ADDR, 0, systemPara.isAutoSongEnable);
	GUI_sendWordIsolate(FUNCTION_CHOSE_5_ADDR, 0, systemPara.isCylinderSensorEnable);
//	GUI_sendWordIsolate(FUNCTION_CHOSE_6_ADDR, 0, systemPara.isUpShouSensorEnable);
	GUI_sendWordIsolate(FUNCTION_CHOSE_7_ADDR, 0, systemPara.isRSTAutoSongEnable);

	GUI_sendWordIsolate(FUNCTION_CHOSE_8_ADDR, 0, systemPara.isCloseHCEnable);
	GUI_sendWordIsolate(FUNCTION_CHOSE_9_ADDR, 0, systemPara.isLackMaterral_NC);
	GUI_sendWordIsolate(FUNCTION_CHOSE_10_ADDR, 0, systemPara.isUpSensorIsRealtime);
	GUI_sendWordIsolate(FUNCTION_CHOSE_11_ADDR, 0, systemPara.isLowspeedSensorEnable);
	GUI_sendWordIsolate(FUNCTION_CHOSE_12_ADDR, 0, systemPara.isGivenEndNoAlarmEnable);

	GUI_sendWordIsolate(FUNCTION_CHOSE_13_ADDR, 0, systemPara.isInEmergencyStopEnable);
	GUI_sendWordIsolate(FUNCTION_CHOSE_14_ADDR, 0, systemPara.isUpSensorDownWichoutEnable);
	GUI_sendWordIsolate(FUNCTION_CHOSE_15_ADDR, 0, systemPara.isInitSwOnLINEEnable);
//	GUI_sendWordIsolate(FUNCTION_CHOSE_16_ADDR, 0, systemPara.isDownShouSensorEnable);
//	GUI_sendWordIsolate(FUNCTION_CHOSE_17_ADDR, 0, systemPara.isUpShouSensorEnable);

	GUI_sendWordIsolate(MOTOR1DIR_ADDR, 0, systemPara.MotorGivenDir);
	GUI_sendWordIsolate(MOTOR2DIR_ADDR, 0, systemPara.MotorLetDir);
	GUI_sendWordIsolate(MOTOR3DIR_ADDR, 0, systemPara.MotorBoDir);
	GUI_sendWordIsolate(MOTOR4DIR_ADDR, 0, systemPara.MotorShouDir);
}

void GUI_sendWord(uint16_t addr,uint8_t high,uint8_t low)
{
    char sendBuf[10];
    uint8_t index = 3;

    sendBuf[index++] = 0x82;

    sendBuf[index++] = addr >> 8;
    sendBuf[index++] = addr & 0xff;

    sendBuf[index++] = high;
    sendBuf[index++] = low;

    sendBuf[0] = 0x5a;
    sendBuf[1] = 0xa5;
    sendBuf[2] = index - 3;

    SEND2SCREEN(sendBuf, index);
}

void GUI_sendWordIsolate(uint16_t addr,uint8_t high,uint8_t low)
{
    char sendBuf[10];
    uint8_t index = 3;

    sendBuf[index++] = 0x82;

    sendBuf[index++] = addr >> 8;
    sendBuf[index++] = addr & 0xff;

    sendBuf[index++] = high;
    sendBuf[index++] = low;

    sendBuf[0] = 0x5a;
    sendBuf[1] = 0xa5;
    sendBuf[2] = index - 3;

    USART2_sendBuf(sendBuf, index);
}

void GUI_switchModeDisp(uint8_t mode)
{
    switch(mode)
    {
        case STATUS_ONLINE:
            GUI_sendWord(AUTOPRODUCT_FLAG_ADDR,0,1);
            GUI_sendWord(MANUALPRODUCT_FALG_ADDR,0,0);
            GUI_sendWord(FREERUN_FLAG_ADDR,0,0);
            break;

        case STATUS_AUTO:
            GUI_sendWord(AUTOPRODUCT_FLAG_ADDR,0,0);
            GUI_sendWord(MANUALPRODUCT_FALG_ADDR,0,1);
            GUI_sendWord(FREERUN_FLAG_ADDR,0,0);
            break;

        case STATUS_FREERUN:
            GUI_sendWord(AUTOPRODUCT_FLAG_ADDR,0,0);
            GUI_sendWord(MANUALPRODUCT_FALG_ADDR,0,0);
            GUI_sendWord(FREERUN_FLAG_ADDR,0,1);
            break;
    }
}

/**
 控制初始化和停止按钮上的红框显示
@param -isRun- 显示状态 0: 停止上显示框 1: 初始化上显示框 2: 两个都不显示框
*/
void GUI_runStatusDisp(uint8_t isRun)
{
    switch(isRun)
    {
        case 0:
            GUI_sendWord(STOP_FLAG_ADDR,0,1);
            GUI_sendWord(INIT_FLAG_ADDR,0,0);
            systemPara.Initstatus = 0;
            break;
        case 1:
            GUI_sendWord(STOP_FLAG_ADDR,0,0);
            GUI_sendWord(INIT_FLAG_ADDR,0,1);
            //systemPara.Initstatus = 1;
            break;
        case 2:
            GUI_sendWord(STOP_FLAG_ADDR,0,0);
            GUI_sendWord(INIT_FLAG_ADDR,0,0);
            break;
    }
}

/**
 到位感应器选择显示
 @param -downSensor- 1-选中下光纤感应器 2-选中上光纤感应器 other-都不选中
*/
void GUI_sensorSelectDisp(uint8_t downSensor)
{
    switch(downSensor)
    {
        case 1:
            GUI_sendWord(SINGAL_SENSOR_DOWN_ADDR,0,1);
            GUI_sendWord(SINGAL_SENSOR_UP_ADDR,0,0);
            break;
        
        case 2:
            GUI_sendWord(SINGAL_SENSOR_DOWN_ADDR,0,0);
            GUI_sendWord(SINGAL_SENSOR_UP_ADDR,0,1);
            break;
        
        default:
            GUI_sendWord(SINGAL_SENSOR_DOWN_ADDR,0,0);
            GUI_sendWord(SINGAL_SENSOR_UP_ADDR,0,0);
            break;
    }
}

/**
 送料模式选择显示
 @param -no- 1-有料不可送 2-有料可送 other-都不选择
*/
void GUI_givenModeSelectDisp(uint8_t no)
{
    switch(no)
    {
        case 1:
            GUI_sendWord(SINGAL_GIVENMODE_N,0,1);
            GUI_sendWord(SINGAL_GIVENMODE_Y,0,0);
            break;
        
        case 2:
            GUI_sendWord(SINGAL_GIVENMODE_N,0,0);
            GUI_sendWord(SINGAL_GIVENMODE_Y,0,1);
            break;
        
        default:
            GUI_sendWord(SINGAL_GIVENMODE_N,0,0);
            GUI_sendWord(SINGAL_GIVENMODE_Y,0,0);
            break;
    }
}

/** 
 设置输出信号块颜色
 @param -outputNo- 输出信号序号. 从供料器内部控制信号开始, 序号从0开始
 @param -color- 显示颜色
    @arg 0- 绿色
    @arg 1- 红色
*/
void GUI_setOutputSignalColorDisp(uint8_t outputNo,uint8_t color)
{
    uint16_t addr;
    
    if(outputNo <= 11)
    {
    	addr = 0x7660;
        addr += outputNo;
        GUI_sendWord(addr,0,color);
    }
    else
    {
    	addr = 0x78d0;
        addr += outputNo-12;
        GUI_sendWord(addr,0,color);
    }
}

void GUI_handler(void)
{
	struct CommData recvData;
	uint16_t addr;
	uint16_t data;
	uint8_t i;

	while (QUEUE_get(&recvQueue, &recvData))
	{
		switch (recvData.data[3])
		{
		case 0x83:
			addr = (recvData.data[4] << 8) | recvData.data[5];
			if (0 == isCmdLegality(recvData.data))
			{
				continue;
			}

			for (i = 0; i < recvData.data[6]; i++)
			{
				data = (recvData.data[7 + (i << 1)] << 8) | recvData.data[8 + (i << 1)];
				PARA_writeParameter(addr + i, data);
			}

			if((addr >= 0x6000) & (addr <= 0x6fff))
			{
				eventCallback[addr - 0x6000](&data);
			}

			logout_count1 = 0;//有按下触摸屏，清除标志

			if (addr >= 0x5000 && addr <= 0x5999)
			{
				// 由0x5000地址计算eeprom地址, 每个0x5000地址对应eeprom两个内存单元
				STORAGE_saveSingalParameter(0x10 + ((addr - 0x5000) << 1),
						data);
				if((addr == 0x500E))//更新剥料电机齿轮比后更新补偿数组
					InitOffsetSpeedArr(PARA_readParameter(0x500E));
			}

			switch (addr)
			{
			case 0x14: //接收到当前页序号指令返回值
				pageNoBeforMessage = (recvData.data[7] << 8) | recvData.data[8];
				if (pageNoBeforMessage == 12 || pageNoBeforMessage == 62) //如果当前在消息页, 返回主页
				{
					if(systemPara.logginStatus < 10 || systemPara.status == STATUS_ONLINE)
					{//未登录和工作模式
						pageNoBeforMessage = 32;
					}
					else
					{//其他
						pageNoBeforMessage = 0;
					}
				}

				GUI_switchPage(12); //跳转到消息页
				break;

			case 0x7000:
				GUI_showText(PASSWORD_DISP_ADDR,"******",6);
				systemPara.passwordStatus = 1;
				break;

			case 0x7002:
				GUI_showText(NPASS_DISP_ADDR, "******", 6);
				systemPara.passwordStatus = 2;
				break;

			case 0x7004:
				GUI_showText(NCPASS_DISP_ADDR, "******", 6);
				systemPara.passwordStatus = 3;
				break;

			case 0x768E:
				if(EmergencyStop() == 1)
					return;
				MC_GMOffsetRun();
				break;

			case 0x768F:
				MC_motorStop(0);
				tempGMOffset += MC_pulse2mm(0);
				PARA_writeParameter(0X5012, tempGMOffset);
				GUI_sendWord(0x5012, tempGMOffset >> 8, tempGMOffset & 0xff);
				break;

			case 0x7691:
				if(EmergencyStop() == 1)
					return;
				MC_GMOffsetRun();
				break;

			case 0x7692:
				MC_motorStop(0);
				break;

			case 0x7900:
				if(EmergencyStop() == 1)
					return;

				//MC_BMOffsetRun();
				Changeover();
				break;
			case 0x7901:

//				MC_motorStop(2);
//				MC_cmd(1,0);
//				MC_cmd(2,0);//换料结束后取消电机使能，可以推动
//				MC_cmd(3,0);
//				MC_cmd(4,0);
				break;

			case 0x7910:
				resetFeederflag = 1;
				break;

			case 0x7911://恢复出厂设置
				resetFeederflag = 1;
				if(resetFeederTime >= 2000)//2秒有效
				{
					resetFeederTime = 0;
					resetFeederflag = 0;
					GUI_switchPage(13);
//					resetFeederPara(0);
				}
				break;
			case 0x7912:
				resetFeederflag = 0;
				break;

			case 0x7915:
				changeoverflag = 1;
				break;
			case 0x7916:
				changeoverflag = 1;
				if(ChangeoverTime >= 3000)// 3秒以上
				{
					ChangeoverTime = 0;
					changeoverflag = 0;
					if(systemPara.logginStatus > 10)	 // 登录状态下
						 GUI_switchPage(0);
					else
						GUI_switchPage(32);
				}
				break;
			case 0x7917:
				changeoverflag = 0;
				break;
			}
			break;
		}
	}
}
