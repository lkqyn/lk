/*
 * easyModbus.c
 *
 *  Created on: 2021年7月12日
 *      Author: ylj
 */

#include "easyModbus/easyModbus.h"
#include "motorctrl/motorctrl.h"
#include "usart1/usart1.h"
#include "usart2/usart2.h"
#include "eventHandler/eventHandler.h"
#include "paraManager/paraManager.h"
#include "dwin/gui.h"
#include "inboardio/inboardio.h"
#include "storage/storage.h"
#include "userfunc/userfunc.h"

uint8_t gNewComRecv = 0;
uint8_t recvBuffer[20];

//extern uint16_t recvOffsetPulseCount;
//extern uint16_t recvDistance, recvDistance1;
//extern uint8_t recvOffsetdir;

//例如：0x00 0x06 0x7919 0x0000 0xffff

uint8_t sendBuffer[46][8] ={
		{0x00, 0x03, 0x7d, 0x00, 0x00, 0x00, 0xff, 0xff  },//0清除停止状态
		{0x00, 0x03, 0x7d, 0x00, 0x00, 0x01, 0xff, 0xff  },//1停止状态

		{0x00, 0x03, 0x7d, 0x01, 0x00, 0x00, 0xff, 0xff  },//2清除暂停状态
		{0x00, 0x03, 0x7d, 0x01, 0x00, 0x01, 0xff, 0xff  },//3暂停状态

		{0x00, 0x03, 0x7d, 0x02, 0x00, 0x00, 0xff, 0xff  },//4清除初始化完成状态
		{0x00, 0x03, 0x7d, 0x02, 0x00, 0x01, 0xff, 0xff  },//5初始化完成状态

		{0x00, 0x03, 0x7d, 0x03, 0x00, 0x00, 0xff, 0xff  },//6清除复位完成状态
		{0x00, 0x03, 0x7d, 0x03, 0x00, 0x01, 0xff, 0xff  },//7复位完成状态

		{0x00, 0x03, 0x7d, 0x04, 0x00, 0x00, 0xff, 0xff  },//8清除送料完成状态
		{0x00, 0x03, 0x7d, 0x04, 0x00, 0x01, 0xff, 0xff  },//9送料完成状态

		{0x00, 0x03, 0x7d, 0x05, 0x00, 0x00, 0xff, 0xff  },//10清除剥料完成状态
		{0x00, 0x03, 0x7d, 0x05, 0x00, 0x01, 0xff, 0xff  },//11剥料完成状态

		{0x00, 0x03, 0x7d, 0x06, 0x00, 0x00, 0xff, 0xff  },//12清除换料状态
		{0x00, 0x03, 0x7d, 0x06, 0x00, 0x01, 0xff, 0xff  },//13换料状态

		{0x00, 0x03, 0x7d, 0x07, 0x00, 0x01, 0xff, 0xff  },//14送料前进微调完成
		{0x00, 0x03, 0x7d, 0x07, 0x00, 0x02, 0xff, 0xff  },//15送料后退微调完成

		{0x00, 0x03, 0x7d, 0x08, 0x00, 0x01, 0xff, 0xff  },//16剥料前进微调完成
		{0x00, 0x03, 0x7d, 0x08, 0x00, 0x02, 0xff, 0xff  },//17剥料后退微调完成

		{0x00, 0x03, 0x7d, 0x09, 0x00, 0x00, 0xff, 0xff  },//18
		{0x00, 0x03, 0x7d, 0x09, 0x00, 0x01, 0xff, 0xff  },//19预留

		{0x00, 0x03, 0x7d, 0x0a, 0x00, 0x00, 0xff, 0xff  },//20
		{0x00, 0x03, 0x7d, 0x0a, 0x00, 0x01, 0xff, 0xff  },//21预留

		{0x00, 0x03, 0x7d, 0x0b, 0x00, 0x00, 0xff, 0xff  },//22未报警
		{0x00, 0x03, 0x7d, 0x0b, 0x00, 0x01, 0xff, 0xff  },//23异常报警

		{0x00, 0x03, 0x7d, 0x0c, 0x00, 0x00, 0xff, 0xff  },//24调试模式
		{0x00, 0x03, 0x7d, 0x0c, 0x00, 0x01, 0xff, 0xff  },//25联机模式

		{0x00, 0x03, 0x7d, 0x0d, 0x00, 0x01, 0xff, 0xff  },//26送料速度保存成功
		{0x00, 0x03, 0x7d, 0x0e, 0x00, 0x01, 0xff, 0xff  },//27送料限位保存成功
		{0x00, 0x03, 0x7d, 0x0f, 0x00, 0x01, 0xff, 0xff  },//28送料补偿保存成功
		{0x00, 0x03, 0x7d, 0x10, 0x00, 0x01, 0xff, 0xff  },//29送料变速位置保存成功
		{0x00, 0x03, 0x7d, 0x11, 0x00, 0x01, 0xff, 0xff  },//30送料变速速度保存成功

		{0x00, 0x03, 0x7d, 0x12, 0x00, 0x01, 0xff, 0xff  },//31剥料速度保存成功
		{0x00, 0x03, 0x7d, 0x13, 0x00, 0x01, 0xff, 0xff  },//32复位速度保存成功
		{0x00, 0x03, 0x7d, 0x14, 0x00, 0x01, 0xff, 0xff  },//33剥料距离1保存成功
		{0x00, 0x03, 0x7d, 0x15, 0x00, 0x01, 0xff, 0xff  },//34剥料距离2保存成功
		{0x00, 0x03, 0x7d, 0x16, 0x00, 0x01, 0xff, 0xff  },//35慢速剥料行程保存成功
		{0x00, 0x03, 0x7d, 0x17, 0x00, 0x01, 0xff, 0xff  },//36慢速剥料速度保存成功

		{0x00, 0x03, 0x7d, 0x18, 0x00, 0x01, 0xff, 0xff  },//37放料速度保存成功
		{0x00, 0x03, 0x7d, 0x19, 0x00, 0x01, 0xff, 0xff  },//38放料行程保存成功

		{0x00, 0x03, 0x7d, 0x1a, 0x00, 0x01, 0xff, 0xff  },//39收料速度保存成功
		{0x00, 0x03, 0x7d, 0x1b, 0x00, 0x01, 0xff, 0xff  },//40收料行程保存成功

		{0x00, 0x03, 0x7d, 0x1c, 0x00, 0x01, 0xff, 0xff  },//41切换下光纤成功
		{0x00, 0x03, 0x7d, 0x1c, 0x00, 0x02, 0xff, 0xff  },//42切换上光纤成功

		{0x00, 0x03, 0x7d, 0x1d, 0x00, 0x01, 0xff, 0xff  },//43切换有料不可送成功
		{0x00, 0x03, 0x7d, 0x1d, 0x00, 0x02, 0xff, 0xff  },//44切换有料可送成功

		{0x00, 0x03, 0x7d, 0x1e, 0x00, 0x00, 0xff, 0xff  },//45换料后退行程

};

void easymodbus_handler(void)
{
	uint16_t address;
	uint16_t value;
	uint16_t readParameter = 0;
	if(gNewComRecv)
	{
		gNewComRecv = 0;
		USART1_sendBuf(recvBuffer, 8);
		address = (recvBuffer[2] << 8) | recvBuffer[3];
		value = (recvBuffer[4] << 8) | recvBuffer[5];
		switch(recvBuffer[1])
		{
		case 0x06:
			if(address == 31000)
			{// 停止
				if(systemPara.Initstatus)
				{
					systemPara.STOPOnceTriggerByUIorIO = 1;
				}
			}

			else if(address == 31001)
			{
//				if(systemPara.RunStatus != 0)
//				{
//					systemPara.pauseOnceTriggerByUIorIO = 1;
//				}
			}

			else if(address == 31002)
			{//初始化
				if(systemPara.RunStatus == 0)// && systemPara.status == STATUS_ONLINE)
				{
					systemPara.doInit = 1;
				}
			}
			else if(address == 31003)
			{//回原点
//				if(systemPara.RunStatus == 0 )//&& systemPara.status == STATUS_ONLINE)
//				{
//					systemPara.doGohome = 1;
//				}
				//复位
				if(systemPara.RunStatus == 0)// && systemPara.status == STATUS_ONLINE)
				{
					if(systemPara.Initstatus)
					{
						systemPara.RSTOnceTriggerByUIorIO = 1;
					}
				}
			}
			else if(address == 31004)
			{//送料
				if(systemPara.RunStatus == 0 )//&& systemPara.status == STATUS_ONLINE)
				{
					if(systemPara.Initstatus)
					{
						systemPara.givenOnceTriggerByUIorIO = systemPara.sensorChosen;
					}
				}
			}
			else if(address == 31005)
			{//剥料
				if(systemPara.RunStatus == 0)// && systemPara.status == STATUS_ONLINE)
				{
					if(systemPara.Initstatus)
					{
						systemPara.HCOnceTriggerByUIorIO = 1;
					}
				}
			}
			else if(address == 31006)
			{//换料
				if(systemPara.RunStatus == 0)
				{
					systemPara.AddMetalOnceTriggerByUIorIO = 1;
					if(value>0)
						recvBLDistance = value;
					else
						recvBLDistance = 3000;//没有设置值时默认走30MM
				}
			}
			else if(address == 31007)
			{//送料前进微调
				ADJ_SL_step(systemPara.MotorGivenDir,value);
			}
			else if(address == 31008)
			{//剥刀后退微调
				ADJ_BL_step(systemPara.MotorBoDir,value);
			}
			else if(address == 31009)
			{//预留
				//ADJ_SL_step(0,value);
			}
			else if(address == 31010)
			{//预留
				//ADJ_BL_step(1,value);
			}
			else if(address == 31011)
			{//模式切换
				switchMode(value);
			}
			else if(address == 31012)
			{//13送料速度设置
				recvDistance = value;
				if(recvDistance<100 || recvDistance>20000)
				{
					sendBuffer[26][5] = 0xff;
					USART1_sendBuf(sendBuffer[26],8);
				}
				else
				{
					PARA_writeParameter(0x5000, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_GIVENSPEED_ADDR, recvDistance);
					GUI_sendWord(0x5000, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[26][5] = 0x01;
					USART1_sendBuf(sendBuffer[26],8);
				}
			}
			else if(address == 31013)
			{//14送料限位设置
				recvDistance = value;
				if(recvDistance<100 || recvDistance>20000)
				{
					sendBuffer[27][5] = 0xff;
					USART1_sendBuf(sendBuffer[27],8);
				}
				else
				{
					PARA_writeParameter(0x5001, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_GIVENOFFSET_ADDR, recvDistance);
					GUI_sendWord(0x5001, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[27][5] = 0x01;
					USART1_sendBuf(sendBuffer[27],8);
				}
			}
			else if(address == 31014)
			{//15送料补偿设置
				recvDistance = value;
				if(recvDistance<0 || recvDistance>3000)
				{
					sendBuffer[28][5] = 0xff;
					USART1_sendBuf(sendBuffer[28],8);
				}
				else
				{
					PARA_writeParameter(0x5012, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_GMDISTANCE_ADDR, recvDistance);
					GUI_sendWord(0x5012, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[28][5] = 0x01;
					USART1_sendBuf(sendBuffer[28],8);
				}
			}
			else if(address == 31015)
			{//16送料变速位置
				recvDistance = value;
				if(recvDistance<0 || recvDistance>50000)
				{
					sendBuffer[29][5] = 0xff;
					USART1_sendBuf(sendBuffer[29],8);
				}
				else
				{
					PARA_writeParameter(0x5016, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_BS1POINT_ADDR, recvDistance);
					GUI_sendWord(0x5016, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[29][5] = 0xff;
					USART1_sendBuf(sendBuffer[29],8);
				}
			}
			else if(address == 31016)
			{//17送料变速速度
				recvDistance = value;
				if(recvDistance<100 || recvDistance>10000)
				{
					sendBuffer[30][5] = 0xff;
					USART1_sendBuf(sendBuffer[30],8);
				}
				else
				{
					PARA_writeParameter(0x5017, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_LOWGMSPEED_ADDR, recvDistance);
					GUI_sendWord(0x5017, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[30][5] = 0x01;
					USART1_sendBuf(sendBuffer[30],8);
				}
			}
			else if(address == 31017)
			{//18剥料速度
				recvDistance = value;
				if(recvDistance<100 || recvDistance>20000)
				{
					sendBuffer[31][5] = 0x2ff;
					USART1_sendBuf(sendBuffer[31],8);
				}
				else
				{
					PARA_writeParameter(0x5004, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_BOSPEED_ADDR, recvDistance);
					GUI_sendWord(0x5004, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[31][5] = 0x01;
					USART1_sendBuf(sendBuffer[31],8);
				}
			}
			else if(address == 31018)
			{//19复位速度
				recvDistance = value;
				if(recvDistance<100 || recvDistance>20000)
				{
					sendBuffer[32][5] = 0xff;
					USART1_sendBuf(sendBuffer[32],8);
				}
				else
				{
					PARA_writeParameter(0x5015, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_RSTSPEED_ADDR, recvDistance);
					GUI_sendWord(0x5015, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[32][5] = 0x01;
					USART1_sendBuf(sendBuffer[32],8);
				}
			}
			else if(address == 31019)
			{//20剥料行程1
				recvDistance = value;
				if(recvDistance<0 || recvDistance>50000)
				{
					sendBuffer[33][5] = 0xff;
					USART1_sendBuf(sendBuffer[33],8);
				}
				else
				{
					PARA_writeParameter(0x5005, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_BODISTANCE_ADDR, recvDistance);
					GUI_sendWord(0x5005, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[33][5] = 0x01;
					USART1_sendBuf(sendBuffer[33],8);
				}
			}
			else if(address == 31020)
			{//21剥料行程2
				recvDistance = value;
				if(recvDistance<0 || recvDistance>50000)
				{
					sendBuffer[34][5] = 0xff;
					USART1_sendBuf(sendBuffer[34],8);
				}
				else
				{
					PARA_writeParameter(0x5013, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_HCD2_ADDR, recvDistance);
					GUI_sendWord(0x5013, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[34][5] = 0x01;
					USART1_sendBuf(sendBuffer[34],8);
				}
			}
			else if(address == 31021)
			{//22慢速剥料行程
				recvDistance = value;
				if(recvDistance<0 || recvDistance>50000)
				{
					sendBuffer[35][5] = 0xff;
					USART1_sendBuf(sendBuffer[35],8);
				}
				else
				{
					PARA_writeParameter(0x5018, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_LOWBODISTANCE_ADDR, recvDistance);
					GUI_sendWord(0x5018, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[35][5] = 0x01;
					USART1_sendBuf(sendBuffer[35],8);
				}
			}
			else if(address == 31022)
			{//23慢速剥料速度
				recvDistance = value;
				if(recvDistance<0 || recvDistance>50000)
				{
					sendBuffer[36][5] = 0xff;
					USART1_sendBuf(sendBuffer[36],8);
				}
				else
				{
					PARA_writeParameter(0x5019, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_LOWBO_SPEED_ADDR, recvDistance);
					GUI_sendWord(0x5019, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[836][5] = 0x01;
					USART1_sendBuf(sendBuffer[36],8);
				}
			}
			else if(address == 31023)
			{//24放料速度
				recvDistance = value;
				if(recvDistance<0 || recvDistance>50000)
				{
					sendBuffer[37][5] = 0xff;
					USART1_sendBuf(sendBuffer[37],8);
				}
				else
				{
					PARA_writeParameter(0x5002, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_LETSPEED_ADDR, recvDistance);
					GUI_sendWord(0x5002, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[37][5] = 0x01;
					USART1_sendBuf(sendBuffer[37],8);
				}
			}
			else if(address == 31024)
			{//25放料行程
				recvDistance = value;
				if(recvDistance<0 || recvDistance>50000)
				{
					sendBuffer[38][5] = 0xff;
					USART1_sendBuf(sendBuffer[38],8);
				}
				else
				{
					PARA_writeParameter(0x5003, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_LETOFFSET_ADDR, recvDistance);
					GUI_sendWord(0x5003, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[38][5] = 0x01;
					USART1_sendBuf(sendBuffer[38],8);
				}
			}
			else if(address == 31025)
			{//26收料速度
				recvDistance = value;
				if(recvDistance<0 || recvDistance>50000)
				{
					sendBuffer[39][5] = 0xff;
					USART1_sendBuf(sendBuffer[39],8);
				}
				else
				{
					PARA_writeParameter(0x5006, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_SHOUSPEED_ADDR, recvDistance);
					GUI_sendWord(0x5006, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[39][5] = 0x01;
					USART1_sendBuf(sendBuffer[39],8);
				}
			}
			else if(address == 31026)
			{//27收料行程
				recvDistance = value;
				if(recvDistance<0 || recvDistance>50000)
				{
					sendBuffer[40][5] = 0xff;
					USART1_sendBuf(sendBuffer[40],8);
				}
				else
				{
					PARA_writeParameter(0x5007, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_SHOUDISTANCE_ADDR, recvDistance);
					GUI_sendWord(0x5007, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[40][5] = 0x01;
					USART1_sendBuf(sendBuffer[40],8);
				}
			}
			else if(address == 31027)
			{//28传感器选择
				if(value == 1)
				{
					GUI_sendWord(SINGAL_SENSOR_DOWN_ADDR,0,1);
					GUI_sendWord(SINGAL_SENSOR_UP_ADDR,0,0);
					GUI_sendWord(SINGAL_SENSOR_NO_ADDR,0,0);

					systemPara.sensorChosen = 1;
					AT24CXX_WriteOneByte(STORAGE_SENSORSELECT_ADDR,1);
					sendBuffer[41][5] = 0x01;
					USART1_sendBuf(sendBuffer[41],8);

					if(systemPara.givenMode == 2)
					{
						GUI_sendWord(SINGAL_GIVENMODE_N,0,1);
						GUI_sendWord(SINGAL_GIVENMODE_Y,0,0);

						systemPara.givenMode = 1;
						AT24CXX_WriteOneByte(STORAGE_GIVENMODESELECT_ADDR,1);
						sendBuffer[43][5] = 0x01;
						USART1_sendBuf(sendBuffer[43],8);
					}
				}
				else if(value == 2)
				{
					if(systemPara.isLowspeedSensorEnable)
					{
						if(systemPara.Language == 0)
							GUI_showMessage("传感器减速功能已开启，请关闭！",30);
						else if(systemPara.Language == 1)
							GUI_showMessage("Please turn off sensor deceleration!",36);

						sendBuffer[41][5] = 0xff;
						USART1_sendBuf(sendBuffer[41],8);
						break;
					}

					GUI_sendWord(SINGAL_SENSOR_DOWN_ADDR,0,0);
					GUI_sendWord(SINGAL_SENSOR_UP_ADDR,0,1);
					GUI_sendWord(SINGAL_SENSOR_NO_ADDR,0,0);

					systemPara.sensorChosen = 2;
					AT24CXX_WriteOneByte(STORAGE_SENSORSELECT_ADDR,2);
					sendBuffer[41][5] = 0x02;
					USART1_sendBuf(sendBuffer[41],8);

					//上光纤不支持自动送料和复位同步送料
					systemPara.isAutoSongEnable = 0;
					GUI_sendWord(FUNCTION_CHOSE_4_ADDR, 0, 0);
					AT24CXX_WriteOneByte(STORAGE_AUTOSONG_EN_ADDR, 0);
					systemPara.isRSTAutoSongEnable = 0;
					GUI_sendWord(FUNCTION_CHOSE_7_ADDR, 0, 0);
					AT24CXX_WriteOneByte(STORAGE_RST_AUTOSONG_EN_ADDR, 0);
				}
				else if(value == 3)
				{
					GUI_sendWord(SINGAL_SENSOR_DOWN_ADDR,0,0);
					GUI_sendWord(SINGAL_SENSOR_UP_ADDR,0,0);
					GUI_sendWord(SINGAL_SENSOR_NO_ADDR,0,1);

					systemPara.sensorChosen = 3;
					AT24CXX_WriteOneByte(STORAGE_SENSORSELECT_ADDR,3);
					sendBuffer[41][5] = 0x03;
					USART1_sendBuf(sendBuffer[41],8);

					//上光纤不支持自动送料和复位同步送料
					systemPara.isAutoSongEnable = 0;
					GUI_sendWord(FUNCTION_CHOSE_4_ADDR, 0, 0);
					AT24CXX_WriteOneByte(STORAGE_AUTOSONG_EN_ADDR, 0);
					systemPara.isRSTAutoSongEnable = 0;
					GUI_sendWord(FUNCTION_CHOSE_7_ADDR, 0, 0);
					AT24CXX_WriteOneByte(STORAGE_RST_AUTOSONG_EN_ADDR, 0);
				}
				else
				{
					sendBuffer[41][5] = 0xff;
					USART1_sendBuf(sendBuffer[41],8);
				}
			}
			else if(address == 31028)
			{//29送料模式
				if(value == 1)
				{
					GUI_sendWord(SINGAL_GIVENMODE_N,0,1);
					GUI_sendWord(SINGAL_GIVENMODE_Y,0,0);

					systemPara.givenMode = 1;
					AT24CXX_WriteOneByte(STORAGE_GIVENMODESELECT_ADDR,1);
					sendBuffer[43][5] = 0x01;
					USART1_sendBuf(sendBuffer[43],8);
				}
				else if(value == 2)
				{
		        	if(systemPara.sensorChosen == 1)
		        	{
						if(systemPara.Language == 0)
							GUI_showMessage("当前感应器模式不可选此模式！",30);
						else if(systemPara.Language == 1)
							GUI_showMessage("Feed mode does not support this feature.",40);

		        		sendBuffer[43][5] = 0x03;
		        		USART1_sendBuf(sendBuffer[43],8);
		        		break;
		        	}
		        	else
		        	{
						GUI_sendWord(SINGAL_GIVENMODE_N,0,0);
						GUI_sendWord(SINGAL_GIVENMODE_Y,0,1);

						systemPara.givenMode = 2;
						AT24CXX_WriteOneByte(STORAGE_GIVENMODESELECT_ADDR,2);
						sendBuffer[43][5] = 0x02;
						USART1_sendBuf(sendBuffer[43],8);
		        	}
				}
				else
				{
					sendBuffer[43][5] = 0xff;
					USART1_sendBuf(sendBuffer[43],8);
				}
			}
			else if(address == 31029)
			{//30换料后退行程
				recvDistance = value;
				if(recvDistance<100 || recvDistance>30000)
				{
					sendBuffer[13][5] = 0xff;
					USART1_sendBuf(sendBuffer[13],8);
				}
				else
				{
					PARA_writeParameter(0x5014, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_CHANGEOVER_ADDR, recvDistance);
					GUI_sendWord(0x5014, recvDistance >> 8, recvDistance & 0xff);
					sendBuffer[13][5] = 0x01;
					USART1_sendBuf(sendBuffer[13],8);
				}
			}
			break;
	case 0x03:
		if(address == 32000)
		{//停止状态
			if(systemPara.Initstatus == 1)
			{
				USART1_sendBuf(sendBuffer[0],8);
			}
			else if(systemPara.Initstatus == 0)
			{
				USART1_sendBuf(sendBuffer[1],8);
			}
		}

		else if(address == 32001)
		{//备用
		}

		else if(address == 32002)
		{//初始化
			if(systemPara.Initstatus == 1)
			{
				USART1_sendBuf(sendBuffer[5],8);
			}
			else
			{
				USART1_sendBuf(sendBuffer[4],8);
			}
		}
		else if(address == 32003)
		{//复位状态
			if(systemPara.RSTOnceOK == 1)
				USART1_sendBuf(sendBuffer[7],8);
			else
				USART1_sendBuf(sendBuffer[6],8);
		}
		else if(address == 32004)
		{//送料状态
			if(systemPara.givenOnceOK == 1)
			{
				USART1_sendBuf(sendBuffer[9],8);
			}
			else
			{
				USART1_sendBuf(sendBuffer[8],8);
			}
		}
		else if(address == 32005)
		{//剥料状态
			if(systemPara.HCOnceOK  == 1)
			{
				USART1_sendBuf(sendBuffer[11],8);
			}
			else
			{
				USART1_sendBuf(sendBuffer[10],8);
			}
		}
		else if(address == 32006)
		{//换料动作状态
			if(systemPara.ChangeOver  == 1)
			{
				USART1_sendBuf(sendBuffer[13],8);
			}
			else
			{
				USART1_sendBuf(sendBuffer[12],8);
			}
		}
		else if(address == 32007)
		{//送料微调状态
			if(systemPara.AdjustGivenOK == 1)
				USART1_sendBuf(sendBuffer[14],8);
			else
			{
				sendBuffer[14][5] = 0x00;
				USART1_sendBuf(sendBuffer[14],8);
				sendBuffer[14][5] = 0x01;
			}

		}
		else if(address == 32008)
		{//后撤微调状态
			if(systemPara.AdjustGivenOK == 1)
				USART1_sendBuf(sendBuffer[16],8);
			else
			{
				sendBuffer[16][5] = 0x00;
				USART1_sendBuf(sendBuffer[16],8);
				sendBuffer[16][5] = 0x01;
			}
		}
		else if(address == 32009)
		{ //

		}
		else if(address == 32010)
		{
		}
		else if(address == 32011)
		{//异常报警
			sendBuffer[23][5] = systemPara.AlarmFlag;
			USART1_sendBuf(sendBuffer[23],8);
		}
		else if(address == 32012)
		{//模式
			if(systemPara.status == STATUS_ONLINE)
				USART1_sendBuf(sendBuffer[25],8);
			else if(systemPara.status == STATUS_AUTO)
				USART1_sendBuf(sendBuffer[24],8);
		}
		else if(address == 32013)
		{//送料速度
			readParameter = PARA_readParameter(0x5000);
			sendBuffer[26][4] = readParameter >> 8;
			sendBuffer[26][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[26],8);
		}
		else if(address == 32014)
		{//送料限位
			readParameter = PARA_readParameter(0x5001);
			sendBuffer[27][4] = readParameter >> 8;
			sendBuffer[27][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[27],8);
		}
		else if(address == 32015)
		{//送料补偿
			readParameter = PARA_readParameter(0x5012);
			sendBuffer[28][4] = readParameter >> 8;
			sendBuffer[28][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[28],8);
		}
		else if(address == 32016)
		{//送料变速位置
			readParameter = PARA_readParameter(0x5016);
			sendBuffer[29][4] = readParameter >> 8;
			sendBuffer[29][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[29],8);
		}
		else if(address == 32017)
		{//送料变速速度
			readParameter = PARA_readParameter(0x5017);
			sendBuffer[30][4] = readParameter >> 8;
			sendBuffer[30][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[30],8);
		}
		else if(address == 32018)
		{//剥料速度
			readParameter = PARA_readParameter(0x5004);
			sendBuffer[31][4] = readParameter >> 8;
			sendBuffer[31][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[31],8);
		}
		else if(address == 32019)
		{//复位速度
			readParameter = PARA_readParameter(0x5015);
			sendBuffer[32][4] = readParameter >> 8;
			sendBuffer[32][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[32],8);
		}
		else if(address == 32020)
		{//剥料行程1
			readParameter = PARA_readParameter(0x5005);
			sendBuffer[33][4] = readParameter >> 8;
			sendBuffer[33][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[33],8);
		}
		else if(address == 32021)
		{//剥料行程2
			readParameter = PARA_readParameter(0x5013);
			sendBuffer[34][4] = readParameter >> 8;
			sendBuffer[34][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[34],8);
		}
		else if(address == 32022)
		{//慢速剥料行程
			readParameter = PARA_readParameter(0x5018);
			sendBuffer[35][4] = readParameter >> 8;
			sendBuffer[35][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[35],8);
		}
		else if(address == 32023)
		{//慢速剥料速度
			readParameter = PARA_readParameter(0x5019);
			sendBuffer[36][4] = readParameter >> 8;
			sendBuffer[36][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[36],8);
		}
		else if(address == 32024)
		{//放料速度/下收料
			readParameter = PARA_readParameter(0x5002);
			sendBuffer[37][4] = readParameter >> 8;
			sendBuffer[37][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[37],8);
		}
		else if(address == 32025)
		{//放料行程/下收料
			readParameter = PARA_readParameter(0x5003);
			sendBuffer[38][4] = readParameter >> 8;
			sendBuffer[38][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[38],8);
		}
		else if(address == 32026)
		{//收料速度
			readParameter = PARA_readParameter(0x5006);
			sendBuffer[39][4] = readParameter >> 8;
			sendBuffer[39][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[39],8);
		}
		else if(address == 32027)
		{//收料行程
			readParameter = PARA_readParameter(0x5007);
			sendBuffer[40][4] = readParameter >> 8;
			sendBuffer[40][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[40],8);
		}
		else if(address == 32028)
		{//送料检测模式
			sendBuffer[41][5] = systemPara.sensorChosen;
			USART1_sendBuf(sendBuffer[41],8);
			sendBuffer[41][5] = 1;
		}
		else if(address == 32029)
		{//送料触发模式
			if(systemPara.givenMode == 1)
				USART1_sendBuf(sendBuffer[43],8);//有料不可送
			else if(systemPara.givenMode == 2)
				USART1_sendBuf(sendBuffer[44],8);//有料可送
		}
		else if(address == 32030)
		{//换料后退行程
			readParameter = PARA_readParameter(0x5014);
			sendBuffer[45][4] = readParameter >> 8;
			sendBuffer[45][5] = readParameter & 0xff;
			USART1_sendBuf(sendBuffer[45],8);
		}
		break;
		}
	}
}

void calculate_CRC(unsigned char *message, int length, unsigned char *CRC0)
{
	unsigned char CRCHi, CRCLo, TempHi, TempLo;
	static const unsigned char table[512] = {
		0x00, 0x00, 0xC0, 0xC1, 0xC1, 0x81, 0x01, 0x40, 0xC3, 0x01, 0x03, 0xC0, 0x02, 0x80, 0xC2, 0x41,
		0xC6, 0x01, 0x06, 0xC0, 0x07, 0x80, 0xC7, 0x41, 0x05, 0x00, 0xC5, 0xC1, 0xC4, 0x81, 0x04, 0x40,
		0xCC, 0x01, 0x0C, 0xC0, 0x0D, 0x80, 0xCD, 0x41, 0x0F, 0x00, 0xCF, 0xC1, 0xCE, 0x81, 0x0E, 0x40,
		0x0A, 0x00, 0xCA, 0xC1, 0xCB, 0x81, 0x0B, 0x40, 0xC9, 0x01, 0x09, 0xC0, 0x08, 0x80, 0xC8, 0x41,
		0xD8, 0x01, 0x18, 0xC0, 0x19, 0x80, 0xD9, 0x41, 0x1B, 0x00, 0xDB, 0xC1, 0xDA, 0x81, 0x1A, 0x40,
		0x1E, 0x00, 0xDE, 0xC1, 0xDF, 0x81, 0x1F, 0x40, 0xDD, 0x01, 0x1D, 0xC0, 0x1C, 0x80, 0xDC, 0x41,
		0x14, 0x00, 0xD4, 0xC1, 0xD5, 0x81, 0x15, 0x40, 0xD7, 0x01, 0x17, 0xC0, 0x16, 0x80, 0xD6, 0x41,
		0xD2, 0x01, 0x12, 0xC0, 0x13, 0x80, 0xD3, 0x41, 0x11, 0x00, 0xD1, 0xC1, 0xD0, 0x81, 0x10, 0x40,
		0xF0, 0x01, 0x30, 0xC0, 0x31, 0x80, 0xF1, 0x41, 0x33, 0x00, 0xF3, 0xC1, 0xF2, 0x81, 0x32, 0x40,
		0x36, 0x00, 0xF6, 0xC1, 0xF7, 0x81, 0x37, 0x40, 0xF5, 0x01, 0x35, 0xC0, 0x34, 0x80, 0xF4, 0x41,
		0x3C, 0x00, 0xFC, 0xC1, 0xFD, 0x81, 0x3D, 0x40, 0xFF, 0x01, 0x3F, 0xC0, 0x3E, 0x80, 0xFE, 0x41,
		0xFA, 0x01, 0x3A, 0xC0, 0x3B, 0x80, 0xFB, 0x41, 0x39, 0x00, 0xF9, 0xC1, 0xF8, 0x81, 0x38, 0x40,
		0x28, 0x00, 0xE8, 0xC1, 0xE9, 0x81, 0x29, 0x40, 0xEB, 0x01, 0x2B, 0xC0, 0x2A, 0x80, 0xEA, 0x41,
		0xEE, 0x01, 0x2E, 0xC0, 0x2F, 0x80, 0xEF, 0x41, 0x2D, 0x00, 0xED, 0xC1, 0xEC, 0x81, 0x2C, 0x40,
		0xE4, 0x01, 0x24, 0xC0, 0x25, 0x80, 0xE5, 0x41, 0x27, 0x00, 0xE7, 0xC1, 0xE6, 0x81, 0x26, 0x40,
		0x22, 0x00, 0xE2, 0xC1, 0xE3, 0x81, 0x23, 0x40, 0xE1, 0x01, 0x21, 0xC0, 0x20, 0x80, 0xE0, 0x41,
		0xA0, 0x01, 0x60, 0xC0, 0x61, 0x80, 0xA1, 0x41, 0x63, 0x00, 0xA3, 0xC1, 0xA2, 0x81, 0x62, 0x40,
		0x66, 0x00, 0xA6, 0xC1, 0xA7, 0x81, 0x67, 0x40, 0xA5, 0x01, 0x65, 0xC0, 0x64, 0x80, 0xA4, 0x41,
		0x6C, 0x00, 0xAC, 0xC1, 0xAD, 0x81, 0x6D, 0x40, 0xAF, 0x01, 0x6F, 0xC0, 0x6E, 0x80, 0xAE, 0x41,
		0xAA, 0x01, 0x6A, 0xC0, 0x6B, 0x80, 0xAB, 0x41, 0x69, 0x00, 0xA9, 0xC1, 0xA8, 0x81, 0x68, 0x40,
		0x78, 0x00, 0xB8, 0xC1, 0xB9, 0x81, 0x79, 0x40, 0xBB, 0x01, 0x7B, 0xC0, 0x7A, 0x80, 0xBA, 0x41,
		0xBE, 0x01, 0x7E, 0xC0, 0x7F, 0x80, 0xBF, 0x41, 0x7D, 0x00, 0xBD, 0xC1, 0xBC, 0x81, 0x7C, 0x40,
		0xB4, 0x01, 0x74, 0xC0, 0x75, 0x80, 0xB5, 0x41, 0x77, 0x00, 0xB7, 0xC1, 0xB6, 0x81, 0x76, 0x40,
		0x72, 0x00, 0xB2, 0xC1, 0xB3, 0x81, 0x73, 0x40, 0xB1, 0x01, 0x71, 0xC0, 0x70, 0x80, 0xB0, 0x41,
		0x50, 0x00, 0x90, 0xC1, 0x91, 0x81, 0x51, 0x40, 0x93, 0x01, 0x53, 0xC0, 0x52, 0x80, 0x92, 0x41,
		0x96, 0x01, 0x56, 0xC0, 0x57, 0x80, 0x97, 0x41, 0x55, 0x00, 0x95, 0xC1, 0x94, 0x81, 0x54, 0x40,
		0x9C, 0x01, 0x5C, 0xC0, 0x5D, 0x80, 0x9D, 0x41, 0x5F, 0x00, 0x9F, 0xC1, 0x9E, 0x81, 0x5E, 0x40,
		0x5A, 0x00, 0x9A, 0xC1, 0x9B, 0x81, 0x5B, 0x40, 0x99, 0x01, 0x59, 0xC0, 0x58, 0x80, 0x98, 0x41,
		0x88, 0x01, 0x48, 0xC0, 0x49, 0x80, 0x89, 0x41, 0x4B, 0x00, 0x8B, 0xC1, 0x8A, 0x81, 0x4A, 0x40,
		0x4E, 0x00, 0x8E, 0xC1, 0x8F, 0x81, 0x4F, 0x40, 0x8D, 0x01, 0x4D, 0xC0, 0x4C, 0x80, 0x8C, 0x41,
		0x44, 0x00, 0x84, 0xC1, 0x85, 0x81, 0x45, 0x40, 0x87, 0x01, 0x47, 0xC0, 0x46, 0x80, 0x86, 0x41,
		0x82, 0x01, 0x42, 0xC0, 0x43, 0x80, 0x83, 0x41, 0x41, 0x00, 0x81, 0xC1, 0x80, 0x81, 0x40, 0x40,
	};

	CRCHi = 0xff;
	CRCLo = 0xff;
	while(length)
	{
		TempHi = CRCHi;
		TempLo = CRCLo;
		CRCHi = table[2 * (*message ^ TempLo)];
		CRCLo = TempHi ^ table[(2 * (*message ^ TempLo)) + 1];
		message++;
		length--;
	};
	CRC0 [0] = CRCLo;
	CRC0 [1] = CRCHi;
}

