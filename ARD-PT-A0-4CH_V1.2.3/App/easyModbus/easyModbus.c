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
char recvBuffer[20];

//extern uint16_t recvOffsetPulseCount;
//extern uint16_t recvDistance, recvDistance1;
//extern uint8_t recvOffsetdir;

//例如：0x00 0x06 0x7919 0x0000 0xffff

char sendBuffer[33][8] ={
		{0x00, 0x03, 0x7d, 0x00, 0x00, 0x00, 0xff, 0xff  },//0清除停止状态
		{0x00, 0x03, 0x7d, 0x00, 0x00, 0x01, 0xff, 0xff  },//1停止状态

		{0x00, 0x03, 0x7d, 0x01, 0x00, 0x00, 0xff, 0xff  },//2
		{0x00, 0x03, 0x7d, 0x01, 0x00, 0x01, 0xff, 0xff  },//3初始化状态

		{0x00, 0x03, 0x7d, 0x02, 0x00, 0x00, 0xff, 0xff  },//4
		{0x00, 0x03, 0x7d, 0x02, 0x00, 0x01, 0xff, 0xff  },//5回原点

		{0x00, 0x03, 0x7d, 0x03, 0x00, 0x00, 0xff, 0xff  },//6
		{0x00, 0x03, 0x7d, 0x03, 0x00, 0x01, 0xff, 0xff  },//7取标

		{0x00, 0x03, 0x7d, 0x04, 0x00, 0xff, 0xff, 0xff  },//8
		{0x00, 0x03, 0x7d, 0x04, 0x00, 0x01, 0xff, 0xff  },//9贴标

		{0x00, 0x03, 0x7d, 0x05, 0x00, 0x00, 0xff, 0xff  },//10
		{0x00, 0x03, 0x7d, 0x05, 0x00, 0x01, 0xff, 0xff  },//11打印请求

		{0x00, 0x03, 0x7d, 0x06, 0x00, 0x00, 0xff, 0xff  },//12
		{0x00, 0x03, 0x7d, 0x06, 0x00, 0x01, 0xff, 0xff  },//13

		{0x00, 0x03, 0x7d, 0x07, 0x00, 0x01, 0xff, 0xff  },//14
		{0x00, 0x03, 0x7d, 0x07, 0x00, 0x02, 0xff, 0xff  },//15

		{0x00, 0x03, 0x7d, 0x08, 0x00, 0x01, 0xff, 0xff  },//16
		{0x00, 0x03, 0x7d, 0x08, 0x00, 0x02, 0xff, 0xff  },//17

		{0x00, 0x03, 0x7d, 0x09, 0x00, 0x00, 0xff, 0xff  },//18
		{0x00, 0x03, 0x7d, 0x09, 0x00, 0x01, 0xff, 0xff  },//19

		{0x00, 0x03, 0x7d, 0x0a, 0x00, 0x00, 0xff, 0xff  },//20调试模式
		{0x00, 0x03, 0x7d, 0x0a, 0x00, 0x01, 0xff, 0xff  },//21联机模式

		{0x00, 0x03, 0x7d, 0x0b, 0x00, 0x00, 0xff, 0xff  },//22无报警
		{0x00, 0x03, 0x7d, 0x0b, 0x00, 0x01, 0xff, 0xff  },//23异常报警状态

		{0x00, 0x03, 0x7d, 0x0c, 0x00, 0x01, 0xff, 0xff  },//24取标次数
		{0x00, 0x03, 0x7d, 0x0d, 0x00, 0x01, 0xff, 0xff  },//25取标间距
		{0x00, 0x03, 0x7d, 0x0e, 0x00, 0x01, 0xff, 0xff  },//26贴标起始位
		{0x00, 0x03, 0x7d, 0x0f, 0x00, 0x01, 0xff, 0xff  },//27贴标间距
		{0x00, 0x03, 0x7d, 0x10, 0x00, 0x01, 0xff, 0xff  },//28贴标次数
		{0x00, 0x03, 0x7d, 0x11, 0x00, 0x01, 0xff, 0xff  },//29X轴速度
		{0x00, 0x03, 0x7d, 0x12, 0x00, 0x01, 0xff, 0xff  },//30回原点速度
		{0x00, 0x03, 0x7d, 0x13, 0x00, 0x01, 0xff, 0xff  },//31回原点偏移
		{0x00, 0x03, 0x7d, 0x14, 0x00, 0x01, 0xff, 0xff  },//32复位速度

};

void easymodbus_handler(void)
{
	uint16_t address;
	uint16_t value;
	uint16_t readParameter;
	if(gNewComRecv)
	{
		gNewComRecv = 0;
//		USART1_sendBuf(recvBuffer, 8);

		address = (recvBuffer[2] << 8) | recvBuffer[3];
		value = (recvBuffer[4] << 8) | recvBuffer[5];
		switch(recvBuffer[1])
		{
		case 0x06:
			if(address == 31000)
			{
				if(systemPara.Initstatus)
				{
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
					systemPara.STOPOnceTriggerByUIorIO = 1;
				}
			}
			else if(address == 31001)
			{//初始化
				if(systemPara.RunStatus == 0 && systemPara.status == STATUS_ONLINE)
				{
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
					systemPara.doInit = 1;
				}
			}
			else if(address == 31002)
			{//回原点
				if(systemPara.RunStatus == 0 && systemPara.status == STATUS_ONLINE)
				{
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
					systemPara.doGohome = 1;
				}
			}
			else if(address == 31003)
			{//取标
				if(systemPara.RunStatus == 0 && systemPara.status == STATUS_ONLINE)
				{
					if(systemPara.Initstatus)
					{
						USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
						systemPara.pickOnceTriggerByUIorIO = 1;
					}
				}
			}
			else if(address == 31004)
			{//贴标
				if(systemPara.RunStatus == 0 && systemPara.status == STATUS_ONLINE)
				{
					if(systemPara.Initstatus)
					{
						USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
						systemPara.stickOnceTriggerByUIorIO = 2;
						systemPara.stickOnceOK = 0;
					}
				}
			}
			//31005-31009预留
			else if(address == 31010)
			{//模式切换
				USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
				switchMode(value);
			}
			/*
			 * 参数设置
			 */
			else if(address == 31011)
			{//贴标位置
				if((0 < value) &&  (value <= ((ADDR_5000_H[5] << 8) | ADDR_5000_L[5])))
				{
					Label_Position = value;
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
				}
				else
				{
					sendBuffer[8][5] = 0xff;
					USART1_sendBuf(sendBuffer[8],8);
				}
			}
			else if(address == 31012)
			{//取标次数
				recvDistance = value;
				if(recvDistance<100 || recvDistance>20000)
				{
					sendBuffer[24][5] = 0xff;
					USART1_sendBuf(sendBuffer[24],8);
				}
				else
				{
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
					PARA_writeParameter(0x5000, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_GIVENSPEED_ADDR, recvDistance);
					GUI_sendWord(0x5000, recvDistance >> 8, recvDistance & 0xff);
				}
			}
			else if(address == 31013)
			{//取标间距
				recvDistance = value;
				if(recvDistance<100 || recvDistance>20000)
				{
					sendBuffer[25][5] = 0xff;
					USART1_sendBuf(sendBuffer[25],8);
				}
				else
				{
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
					PARA_writeParameter(0x5001, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_GIVENOFFSET_ADDR, recvDistance);
					GUI_sendWord(0x5001, recvDistance >> 8, recvDistance & 0xff);
				}
			}
			else if(address == 31014)
			{//贴标起始位
				recvDistance = value;
				if(recvDistance<0 || recvDistance>50000)
				{
					sendBuffer[26][5] = 0xff;
					USART1_sendBuf(sendBuffer[26],8);
				}
				else
				{
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
					PARA_writeParameter(0x5002, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_LETSPEED_ADDR, recvDistance);
					GUI_sendWord(0x5002, recvDistance >> 8, recvDistance & 0xff);
				}
			}
			else if(address == 31015)
			{//贴标间距
				recvDistance = value;
				if(recvDistance<0 || recvDistance>3000)
				{
					sendBuffer[27][5] = 0xff;
					USART1_sendBuf(sendBuffer[227],8);
				}
				else
				{
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
					PARA_writeParameter(0x5012, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_GMDISTANCE_ADDR, recvDistance);
					GUI_sendWord(0x5012, recvDistance >> 8, recvDistance & 0xff);
				}
			}
			else if(address == 31016)
			{//贴标次数
				recvDistance = value;
				if(recvDistance<100 || recvDistance>10000)
				{
					sendBuffer[28][5] = 0xff;
					USART1_sendBuf(sendBuffer[28],8);
				}
				else
				{
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
					PARA_writeParameter(0x5005, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_BODISTANCE_ADDR, recvDistance);
					GUI_sendWord(0x5005, recvDistance >> 8, recvDistance & 0xff);
				}
			}
			else if(address == 31017)
			{//X轴速度
				recvDistance = value;
				if(recvDistance<100 || recvDistance>20000)
				{
					sendBuffer[29][5] = 0xff;
					USART1_sendBuf(sendBuffer[29],8);
				}
				else
				{
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
					PARA_writeParameter(0x5004, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_BOSPEED_ADDR, recvDistance);
					GUI_sendWord(0x5004, recvDistance >> 8, recvDistance & 0xff);
				}
			}
			else if(address == 31018)
			{//回原点速度
				recvDistance = value;
				if(recvDistance<100 || recvDistance>20000)
				{
					sendBuffer[32][5] = 0xff;
					USART1_sendBuf(sendBuffer[32],8);
				}
				else
				{
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
					PARA_writeParameter(0x5003, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_LETOFFSET_ADDR, recvDistance);
					GUI_sendWord(0x5003, recvDistance >> 8, recvDistance & 0xff);
				}
			}
			else if(address == 31019)
			{//回原点偏移
				recvDistance = value;
				if(recvDistance<0 || recvDistance>50000)
				{
					sendBuffer[31][5] = 0xff;
					USART1_sendBuf(sendBuffer[31],8);
				}
				else
				{
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
					PARA_writeParameter(0x5006, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_SHOUSPEED_ADDR, recvDistance);
					GUI_sendWord(0x5006, recvDistance >> 8, recvDistance & 0xff);
				}
			}
			else if(address == 31020)
			{//复位速度
				recvDistance = value;
				if(recvDistance<0 || recvDistance>50000)
				{
					sendBuffer[32][5] = 0xff;
					USART1_sendBuf(sendBuffer[32],8);
				}
				else
				{
					USART1_sendBuf(recvBuffer, 8);//成功应答返回原数据
					PARA_writeParameter(0x5015, recvDistance);
					STORAGE_saveSingalParameter(STORAGE_RSTSPEED_ADDR, recvDistance);
					GUI_sendWord(0x5015, recvDistance >> 8, recvDistance & 0xff);
				}
			}
			break;
		/*
		 * 输出
		 */
		case 0x03:
			if(address == 32000)
			{

			}
			else if(address == 32001)
			{
				if(systemPara.Initstatus == 0)
				{
					USART1_sendBuf(sendBuffer[2],8);
				}
				else
				{
					USART1_sendBuf(sendBuffer[3],8);
				}
			}
			else if(address == 32002)
			{
				if(IBIO_getInput(1) == 0 && systemPara.doGohome == 0)
				{
					USART1_sendBuf(sendBuffer[5],8);
				}
				else
				{
					USART1_sendBuf(sendBuffer[4],8);
				}
			}
			else if(address == 32003)
			{
				if(systemPara.pickOnceOK == 1 && systemPara.pickOnceTriggerByUIorIO == 0)
				{
					USART1_sendBuf(sendBuffer[7],8);
				}
				else
				{
					USART1_sendBuf(sendBuffer[6],8);
				}
			}
			else if(address == 32004)
			{
				if(systemPara.stickOnceOK == 1 && systemPara.stickOnceTriggerByUIorIO == 0)
				{//贴标完成
					sendBuffer[9][4] = Label_Position >> 8;
					sendBuffer[9][5] = Label_Position & 0xff;
					USART1_sendBuf(sendBuffer[9],8);
				}
				else
				{
					if((0 < Label_Position) &&  (Label_Position <= ((ADDR_5000_H[5] << 8) | ADDR_5000_L[5])))
					{
						sendBuffer[8][5] = 0x00;
						USART1_sendBuf(sendBuffer[8],8);
					}
					else
					{
						sendBuffer[8][5] = 0xff;
						USART1_sendBuf(sendBuffer[8],8);
					}

				}
			}
			else if(address == 32006)
			{
				if(systemPara.Initstatus == 0)
				{
					USART1_sendBuf(sendBuffer[2],8);
				}
				else
				{
					if(IBIO_getInput(3) == 1 && systemPara.pickOnceOK == 0)
					{
						USART1_sendBuf(sendBuffer[11],8);
					}
					else
					{
						if(systemPara.pickOnceOK == 1 && systemPara.pickOnceTriggerByUIorIO == 0)
						{
							USART1_sendBuf(sendBuffer[7],8);
						}
						else
						{
							if(systemPara.stickOnceOK == 1 && systemPara.stickOnceTriggerByUIorIO == 0)
							{
								sendBuffer[9][4] = Label_Position >> 8;
								sendBuffer[9][5] = Label_Position & 0xff;
								USART1_sendBuf(sendBuffer[9],8);
							}
							else
							{
								if((0 < Label_Position) &&  (Label_Position <= ((ADDR_5000_H[5] << 8) | ADDR_5000_L[5])))
								{
									sendBuffer[8][5] = 0x00;
									USART1_sendBuf(sendBuffer[8],8);
								}
								else
								{
									sendBuffer[8][5] = 0xff;
									USART1_sendBuf(sendBuffer[8],8);
								}
							}
						}
					}
				}
			}
			else if(address == 32010)
			{
				if(systemPara.status == STATUS_ONLINE)
				{
					USART1_sendBuf(sendBuffer[21],8);
				}
				else if(systemPara.status == STATUS_AUTO)
				{
					USART1_sendBuf(sendBuffer[20],8);
				}
			}
			else if(address == 32011)
			{
				switch(systemPara.AlarmFlag)
				{
					case 0:sendBuffer[23][5] = 0x00;break;		//无报警
					case 1:sendBuffer[23][5] = 0x01;break;		//X轴触发限位
					case 2:sendBuffer[23][5] = 0x02;break;		//X轴回原失败
					case 3:sendBuffer[23][5] = 0x03;break;		//标签取完，光纤检测异常
					case 4:sendBuffer[23][5] = 0x04;break;		//打印完成，光纤检测异常
					case 5:sendBuffer[23][5] = 0x05;break;		//贴标完成真空复检异常
					case 6:sendBuffer[23][5] = 0x06;break;		//升降气缸动作异常
					case 7:sendBuffer[23][5] = 0x07;break;		//贴标气缸动作异常
					case 8:sendBuffer[23][5] = 0x08;break;		//翻转气缸动作异常
					case 9:sendBuffer[23][5] = 0x09;break;		//平台气缸动作异常
					case 10:sendBuffer[23][5] = 0x0a;break;		//真空电磁阀动作异常

					case 20:sendBuffer[23][5] = 0x10;break;		//送料驱动报警
					case 21:sendBuffer[23][5] = 0x11;break;		//剥料驱动报警
				}
				USART1_sendBuf(sendBuffer[23],8);
			}
			else if(address == 32012)
			{
				readParameter = PARA_readParameter(0x5000);
				sendBuffer[24][4] = readParameter >> 8;
				sendBuffer[24][5] = readParameter & 0xff;
				USART1_sendBuf(sendBuffer[24],8);
			}
			else if(address == 32013)
			{
				readParameter = PARA_readParameter(0x5001);
				sendBuffer[25][4] = readParameter >> 8;
				sendBuffer[25][5] = readParameter & 0xff;
				USART1_sendBuf(sendBuffer[25],8);
			}
			else if(address == 32014)
			{
				readParameter = PARA_readParameter(0x5002);
				sendBuffer[26][4] = readParameter >> 8;
				sendBuffer[26][5] = readParameter & 0xff;
				USART1_sendBuf(sendBuffer[26],8);
			}
			else if(address == 32015)
			{
				readParameter = PARA_readParameter(0x5012);
				sendBuffer[27][4] = readParameter >> 8;
				sendBuffer[27][5] = readParameter & 0xff;
				USART1_sendBuf(sendBuffer[27],8);
			}
			else if(address == 32016)
			{
				readParameter = PARA_readParameter(0x5005);
				sendBuffer[28][4] = readParameter >> 8;
				sendBuffer[28][5] = readParameter & 0xff;
				USART1_sendBuf(sendBuffer[28],8);
			}
			else if(address == 32017)
			{
				readParameter = PARA_readParameter(0x5004);
				sendBuffer[29][4] = readParameter >> 8;
				sendBuffer[29][5] = readParameter & 0xff;
				USART1_sendBuf(sendBuffer[29],8);
			}
			else if(address == 32018)
			{
				readParameter = PARA_readParameter(0x5003);
				sendBuffer[30][4] = readParameter >> 8;
				sendBuffer[30][5] = readParameter & 0xff;
				USART1_sendBuf(sendBuffer[30],8);
			}
			else if(address == 32019)
			{
				readParameter = PARA_readParameter(0x5006);
				sendBuffer[31][4] = readParameter >> 8;
				sendBuffer[31][5] = readParameter & 0xff;
				USART1_sendBuf(sendBuffer[31],8);
			}
			else if(address == 32020)
			{
				readParameter = PARA_readParameter(0x5015);
				sendBuffer[32][4] = readParameter >> 8;
				sendBuffer[32][5] = readParameter & 0xff;
				USART1_sendBuf(sendBuffer[32],8);
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

