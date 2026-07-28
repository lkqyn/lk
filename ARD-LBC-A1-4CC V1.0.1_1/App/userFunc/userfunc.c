#include "userfunc/userfunc.h"
#include "motorctrl/motorctrl.h"
#include "paramanager/paramanager.h"
#include "inboardio/inboardio.h"
#include "exio/exio.h"
#include "dwin/gui.h"
#include "pwm1/pwm1.h"
#include "delay/delay.h"
//#include "tim6/tim6.h"
#include "eventHandler/eventHandler.h"
//#include "iwdg/iwdg.h"

//static uint8_t CylinderIn(void);
//static uint8_t CylinderOut(void);

extern void selectAutoMode(void *pv);
extern void selectManualMode(void *pv);
extern void selectFreeRunMode(void *pv);

uint64_t KP_count = 0; //空跑计数
uint64_t songliao_count = 0; // 送料计数

uint32_t FLdistance = 0;//送料累计报警距离
uint16_t KP_step = 0;  // 空跑步骤
uint16_t OLM_SL_step = 0;
uint16_t OLM_step = 0;
uint16_t OLM_RST_step = 0;
uint16_t CUT_step = 0;
uint16_t CUT_RST_step = 0;
uint16_t Init_step = 0;
uint16_t AM_step = 0;
uint16_t ms_count = 0;
uint16_t Shut_vacuum_step = 0;

uint16_t Gome_step = 0;
uint16_t Gome_CutLable = 0;
//uint32_t offsetSpeed[8]={230, 215, 190, 170, 150, 130, 115, 90};//初始化补偿,
uint32_t offsetSpeed[8]={115, 110, 105, 90, 75, 65, 40, 35};//初始化补偿,
//uint32_t offsetSpeed1[8]={200, 165, 130, 105, 82, 48, 15, 0};//夹料复位补偿
uint32_t offsetSpeed1[8]={120, 115, 110, 95, 80, 60, 25, 0};//夹料复位补偿


static uint8_t Cut_flag = 0;//切料标志，防止重复切料后退
uint8_t replacenment_flag = 0;//换料标志
uint8_t FLflag = 0,FLflag1 = 0; //放料标志位
uint8_t USLflag = 0; //上收料料标志位
uint8_t DSLflag = 0; //下收料料标志位
uint8_t SsorCnt = 0;//送料次数，用于检测没有放料情况时输出缺料预报警

uint8_t RST_flag = 0 ; //1，复位传感模式

volatile uint8_t QQ_speed_h, QQ_speed_l;
volatile uint8_t QQ1_speed_h, QQ1_speed_l;
volatile uint8_t QQ2_speed_h, QQ2_speed_l;

#define 	IOUI_ALERT()\
		do{\
			EXIO_setOutput(1, 0); \
			GUI_setOutputSignalColorDisp(9,0);\
			systemPara.isWorkTaskRun = 0;\
		}while(0)

#define		IOUI_UNALERT()\
		do{\
			EXIO_setOutput(2, 1); \
			GUI_setOutputSignalColorDisp(9,1);\
		}while(0)

#define		IOUI_UNREADY()\
		do{\
			EXIO_setOutput(0, 1); \
			GUI_setOutputSignalColorDisp(8,1);\
		}while(0)

#define		IOUI_READY()\
		do{\
			EXIO_setOutput(0, 0); \
			GUI_setOutputSignalColorDisp(8,0);\
		}while(0)



/**
 @breif 空函数
 */
void konghanshu(void *pv)
{
	// 如果未执行初始化, 不能执行该动作
	if (systemPara.status == STATUS_UNINIT)
	{
		return;
	}
}

/**
 @brief 控制输出信号
 */
void controlOutputSignal(void *pv)
{
	static uint8_t flag[] =
	{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };


	if(systemPara.isInEmergencyStopEnable)
	{
		if(IBIO_getInput(5) == 1 )
		{
			if(systemPara.Language == 0)
				POP_UP_INFO_RES_E_SOTP();
			else if(systemPara.Language == 1)
				EN_POP_UP_INFO_RES_E_SOTP();
			return;
		}//
	}   
	
	if(systemPara.status == STATUS_ONLINE )
    {
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE1();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE1();
    	return;
    }

	if (flag[*((uint8_t*)pv)])
	{
		if (*((uint8_t*)pv) <= 8)
		{
			IBIO_setOutput(*((uint8_t*)pv), 0);//原来-0:输出信号夹紧气缸;修改为-1:关闭信号夹紧气缸
		}
		else
		{
			EXIO_setOutput(*((uint8_t*)pv) - 9, 0);
		}

		flag[*((uint8_t*)pv)] = 0;

		if((*((uint8_t*)pv)) <= 12)
		{
			GUI_sendWord(0x7660 + *((uint8_t*)pv) - 1, 0, 0);
		}
		else
		{
			GUI_sendWord(0x78d0 + *((uint8_t*)pv) - 13, 0, 0);
		}
	}
	else
	{
		if (*((uint8_t*)pv) <= 8)
		{
			IBIO_setOutput(*((uint8_t*)pv), 1);//原来-1:关闭信号松开气缸;修改为-0:关闭信号松开气缸
		}
		else
		{
			EXIO_setOutput(*((uint8_t*)pv) - 9, 1);
		}

		flag[*((uint8_t*)pv)] = 1;

		if((*((uint8_t*)pv)) <= 12)
		{
			GUI_sendWord(0x7660 + *((uint8_t*)pv) - 1, 0, 1);
		}
		else
		{
			GUI_sendWord(0x78d0 + *((uint8_t*)pv) - 13, 0, 1);
		}

	}
}

void huiyuandian_UI(void *pv)
{
	if(systemPara.RunStatus == 0)
	{
//		systemPara.doGome = 1;
	}
	else
	{
		CANT_SWITCH_PAGE_MESSAGE2();
	}
}

/**
 @brief 联机模式函数调用，分功能
 */
void GivenMatieralOnline(void *pv)
{
	if(systemPara.sensorChosen == 1)
	{
		SL_Step_task(0);
	}
}

#if 0
/**
 * @brief 联机动作连续动作
 */
void OLM_Step_task(void *pv)
{
	char str[8]="0";
	uint16_t len;
	uint32_t limitPulseCount; //拉胶长度
	uint32_t limitPulseCount1; //上收料行程
	uint32_t offsetPulseCount;//原点偏移
	uint32_t distance1;		//	原点补偿加速度补偿
	uint32_t distance;		//	后退距离
	uint32_t RunSpeed;		//运行速度

	switch(OLM_step)
	{
		case 0://等待拉胶信号

			if(EXIO_getInput(0) == 1)
			{
				//GUI_mainMessageDispIsolate("等待设备拉胶信号.", 17);
				return;
			}
			else
			{
				if(EXIO_getInput(1) == 0  || EXIO_getInput(2) == 0)//
				{
					GUI_mainMessageDispIsolate("等待设备关闭初始化和切胶信号.", 29);
					//systemPara.givenOnceTriggerByUIorIO = 0;
					return;
				}
				systemPara.RunStatus = 2;
				GUI_mainMessageDispIsolate("开始拉胶动作.", 13);
				OLM_step ++;
			}
			break;
		case 1://清除交互信号
			EXO_sigOFF(3);
			EXO_sigOFF(5);
			OLM_step ++;
			break;
		case 2://后气缸松开
			GUI_mainMessageDispIsolate("压料气缸缩回动作.", 17);
			IBIO_setOutput(3, 1);
			delay_ms(150);
//			if(GetIOLevel(7, 1, 2000) == 0)
//			{
//				GUI_mainMessageDisp("报警信息：压料气缸缩回异常！", 28);
//				return;
//			}
			OLM_step ++;
			break;
		case 3:
			flTask_handler(0);
			OLM_step ++;
			break;
		case 4://上收料

			OLM_step ++;
			break;
		case 5://下收料

			OLM_step ++;
			break;
		case 6://拉胶
			GUI_mainMessageDispIsolate("拉胶中......", 12);
			distance = (ADDR_5000_H[1] << 8) | ADDR_5000_L[1];
			//offsetPulseCount = MC_mm2pulse(GIVEN_MOTOR,((ADDR_5000_H[18] << 8) | ADDR_5000_L[18]));
			//distance = limitPulseCount + offsetPulseCount;
			MC_motorMoveDistance(GIVEN_MOTOR, 0, distance);
			OLM_step ++;
			break;
		case 7://等待拉胶完成
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				OLM_step ++;
			}
			break;
		case 8://后夹料夹紧
			GUI_mainMessageDispIsolate("压料气缸伸出动作.", 17);
			IBIO_setOutput(3, 0);
			delay_ms(150);
//			if(GetIOLevel(7, 0, 2000) == 0)
//			{
//				GUI_mainMessageDisp("报警信息：压料气缸伸出异常！", 28);
//				return;
//			}
			OLM_step ++;
			break;
		case 9://后升降气缸伸出
//			GUI_mainMessageDispIsolate("后升降气缸伸出动作.", 19);
//			IBIO_setOutput(4, 0);
//			if(GetIOLevel(8, 1, 2000) == 0)
//			{
//				GUI_mainMessageDisp("报警信息：升降气缸伸出异常！", 30);
//				return;
//			}
			OLM_step ++;
			break;
		case 10:
			if(IBIO_getInput(2) == 0)
			{
				songliao_count ++;
				len = sprintf(str,"%d",songliao_count);
				GUI_showText(0x8000, "        ", 8);
				GUI_showText(0x8000, str, len);
				systemPara.givenOnceTriggerByUIorIO = 0;
				systemPara.RunStatus = 0;
				systemPara.givenOK = 1;
				systemPara.RSTOK = 0;
				systemPara.CUTOK = 0;
				GUI_mainMessageDisp("送料完成！", 9);
				EXO_sigON(2);
			}
			else
			{
				GUI_mainMessageDisp("报警信息：送料失败！", 30);
				Alarm(0);
				return;
			}
			OLM_step ++;
			break;
		case 11:
			if(EXIO_getInput(1) == 1 )//
			{
				GUI_mainMessageDispIsolate("等待切胶信号.", 13);
				return;
			}
			else
			{
				systemPara.RunStatus = 3;
				systemPara.CUTOK = 0;
				GUI_mainMessageDispIsolate("开始切胶动作.", 13);
				OLM_step ++;
			}
			break;
		case 12:
			if(IBIO_getInput(7) != 0)
			{
				GUI_mainMessageDispIsolate("压料气缸伸出动作.", 17);
				IBIO_setOutput(3, 0);
				delay_ms(150);
//				if(GetIOLevel(7, 0, 2000) == 0)
//				{
//					GUI_mainMessageDisp("报警信息：压料气缸伸出异常！", 28);
//					Alarm(0);
//					return;
//				}
			}
			OLM_step ++;
			break;
		case 13:
			GUI_mainMessageDispIsolate("开始裁切动作.", 13);
			if(CutLable(0) == 1)
			{
				OLM_step ++;
			}
			else
			{
				GUI_mainMessageDispIsolate("裁切失败.", 9);
				Alarm(0);
				return;
			}
			break;
		case 14://
			if(Cut_flag == 0)
			{
				Cut_flag = 1;
				// 暂存送料电机速度
				QQ1_speed_l = ADDR_5000_L[0];
				QQ1_speed_h = ADDR_5000_H[0];
				ADDR_5000_L[0] = ADDR_5000_L[19];
				ADDR_5000_H[0] = ADDR_5000_H[19];
				distance = (ADDR_5000_H[18] << 8) | ADDR_5000_L[18];
				MC_motorMoveDistance(GIVEN_MOTOR, 0, distance);
			}
			OLM_step ++;
			break;
		case 15:
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				ADDR_5000_L[0] = QQ1_speed_l;
				ADDR_5000_H[0] = QQ1_speed_h;
				OLM_step ++;
			}
			break;
		case 16:
			GUI_mainMessageDispIsolate("前夹料气缸松开动作.", 19);
			IBIO_setOutput(1, 1);
			if(GetIOLevel(5, 0, 2000) == 0)
			{
				GUI_mainMessageDisp("报警信息：前夹料气缸松开异常！", 30);
				Alarm(0);
				return;
			}
			OLM_step ++;
			break;
		case 17:
//			GUI_mainMessageDispIsolate("后升降气缸缩回动作.", 19);
//			IBIO_setOutput(4, 1);
//			if(GetIOLevel(8, 0, 2000) == 0)
//			{
//				GUI_mainMessageDisp("报警信息：升降气缸缩回异常！", 30);
//				Alarm(0);
//				return;
//			}
			OLM_step ++;
			break;
		case 18:
			GUI_mainMessageDispIsolate("切料完成.", 8);
			EXO_sigOFF(2);
			EXO_sigON(1);
			systemPara.RunStatus = 0;
			systemPara.RSTOK = 0;
			systemPara.CUTOK = 1;
			systemPara.CUTOnceTriggerByUIorIO = 0;
			OLM_step ++;
			break;
		case 19://等待复位信号

			if(EXIO_getInput(0) == 0 || EXIO_getInput(1) == 0 )
			{
				GUI_mainMessageDispIsolate("等待设备关闭拉胶和切胶信号.", 27);
				systemPara.RSTOnceTriggerByUIorIO = 0;
				return;
			}
			else
			{
				GUI_mainMessageDispIsolate("开始复位动作.", 13);
				systemPara.RunStatus = 4;
				systemPara.givenOK = 0;
				systemPara.CUTOK = 0;
				systemPara.RSTOK = 0;
				OLM_step ++;
			}
			break;
		case 20://清除交互信号
			EXO_sigOFF(1);
			OLM_step ++;
			break;
		case 21://后气缸夹紧
			GUI_mainMessageDispIsolate("压料气缸伸出动作.", 17);
			IBIO_setOutput(3, 0);
			delay_ms(150);
//			if(GetIOLevel(7, 0, 2000) == 0)
//			{
//				GUI_mainMessageDisp("报警信息：压料气缸伸出异常！", 28);
//				return;
//			}
			OLM_step ++;
			break;
		case 22:
//			GUI_mainMessageDispIsolate("后升降气缸缩回动作.", 19);
//			IBIO_setOutput(4, 1);
//			if(GetIOLevel(8, 0, 2000) == 0)
//			{
//				GUI_mainMessageDisp("报警信息：升降气缸缩回异常！", 30);
//				return;
//			}
			OLM_step ++;
			break;
		case 23:
			GUI_mainMessageDispIsolate("前夹料气缸松开动作.", 19);
			IBIO_setOutput(1, 1);
			if(GetIOLevel(5, 0, 2000) == 0)
			{
				GUI_mainMessageDisp("报警信息：前夹料气缸松开异常！", 30);
				return;
			}
			OLM_step ++;
			break;
		case 24:

			OLM_step ++;
			break;
		case 25://复位
			GUI_mainMessageDispIsolate("复位中......", 12);

			//传感模式加补偿
			offsetPulseCount = MC_mm2pulse(GIVEN_MOTOR,((ADDR_5000_H[9] << 8) | ADDR_5000_L[9]));
			RunSpeed = ((ADDR_5000_H[0] << 8) | ADDR_5000_L[0]) * 0.01f;//读取运行速度

			//goHomeSpeed = ((ADDR_5000_H[8] << 8) | ADDR_5000_L[8]) / 100;

			if( (RunSpeed <= 20) )
				distance1 = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[0]);
			else if( (RunSpeed > 20) &&  ( RunSpeed <= 40))
				distance1 = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[1]);
			else if( (RunSpeed > 40) &&  ( RunSpeed <= 60))
				distance1 = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[2]);
			else if( (RunSpeed > 60) &&  ( RunSpeed <= 80))
				distance1 = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[3]);
			else if( (RunSpeed > 80) &&  ( RunSpeed <= 100))
				distance1 = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[4]);
			else if( (RunSpeed > 100) &&  ( RunSpeed <= 120))
				distance1 = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[5]);
			else if( (RunSpeed > 120) &&  ( RunSpeed <= 140))
				distance1 = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[6]);
			else
				distance1 = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[7]);

			systemPara.SLmode = 1;//传感模式
			MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, 1, distance1,20000);


			//定长模式
//			offsetPulseCount = MC_mm2pulse(GIVEN_MOTOR,((ADDR_5000_H[1] << 8) | ADDR_5000_L[1]));
//			//切料后退距离
//			limitPulseCount = MC_mm2pulse(GIVEN_MOTOR,((ADDR_5000_H[18] << 8) | ADDR_5000_L[18]));
//			distance = limitPulseCount + offsetPulseCount;
//			MC_motorMoveDistance(GIVEN_MOTOR, 1, distance);

			OLM_step ++;
			break;
		case 26://等待电机停止

			OLM_step ++;
			break;
		case 27://等待复位完成
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				OLM_step ++;
			}
			break;
		case 28:
			if (motor_ch[GIVEN_MOTOR].stopEvent == EVENT_exceedPulseCountLimit)
			{
				GUI_mainMessageDisp("报警信息：未检测到原点信号！", 28);
				Alarm(0);
				return;
			}
			OLM_step ++;
			break;
		case 29:
			GUI_mainMessageDispIsolate("前夹料气伸出回动作.", 19);
			IBIO_setOutput(1, 0);
			if(GetIOLevel(5, 0, 2000) == 0)
			{
				GUI_mainMessageDisp("报警信息：前夹料气缸伸出异常！", 30);
				Alarm(0);
				return;
			}
			OLM_step ++;
			break;
		case 30:

			OLM_step ++;
			break;
		case 31:
			GUI_mainMessageDisp("复位完成！", 9);
			EXO_sigON(3);
			EXO_sigON(5);
			systemPara.RunStatus = 0;
			systemPara.givenOK = 0;
			systemPara.CUTOK = 0;
			systemPara.RSTOK = 1;
			Cut_flag = 0;
			systemPara.RSTOnceTriggerByUIorIO = 0;
			OLM_step = 0;
			break;
	}
}


#endif
/**
 * @brief 拉胶动作
 */
void SL_Step_task(void *pv)
{
	char str[8]="0";
	uint16_t len;
	uint32_t limitPulseCount;
	uint32_t offsetPulseCount;
	uint32_t distance;

	switch(OLM_SL_step)
	{
		case 0://等待拉胶信号
			if(systemPara.status != STATUS_FREERUN)
			{
				if (!systemPara.givenOnceTriggerByUIorIO || systemPara.RunStatus != 0)
				{
					systemPara.givenOnceTriggerByUIorIO = 0;
					return;
				}
				if(IBIO_getInput(1) == 1)
				{
					if(systemPara.givenOnceTriggerByUIorIO == 1)
					{
						systemPara.RSTOnceTriggerByUIorIO = 2;
					}
					else if(systemPara.givenOnceTriggerByUIorIO == 2)
					{
						GUI_mainMessageDisp("请先复位动作.", 13);
					}
					systemPara.givenOnceTriggerByUIorIO = 0;
					return;
				}
				if(replacenment_flag == 1 && systemPara.givenOnceTriggerByUIorIO == 3)
				{
					systemPara.givenOnceTriggerByUIorIO = 0;
					GUI_mainMessageDisp("换料动作完成.", 13);
					return;
				}
				if(systemPara.givenOK == 1)
				{
					GUI_mainMessageDisp("请先裁切动作！", 14);
					systemPara.givenOnceTriggerByUIorIO = 0;
					return;
				}
//				if(IBIO_getInput(6) == 0 && systemPara.givenOnceTriggerByUIorIO != 3)//缺料报警停止工作
//				{
//					delay_ms(5);
//					if(IBIO_getInput(6) == 0)
//					{
//						WLYJ_Alert();
//						return;
//					}
//				}
			}
			systemPara.RunStatus = 2;
			systemPara.SLOK = 0;
			systemPara.givenOK = 0;
			GUI_mainMessageDisp("开始送料动作.", 13);
			OLM_SL_step ++;
			break;
		case 1://清除交互信号
			EXO_sigOFF(5);
			EXO_sigOFF(1);
			EXO_sigOFF(2);
			EXO_sigOFF(4);
			OLM_SL_step ++;
			break;
		case 2://
			if(systemPara.givenOnceTriggerByUIorIO != 3)
			{
				if(IBIO_getInput(1) == 0)
				{
					if(JL_Action(0) == 0)
						return;
				}
			}
			else
			{
				if(JL_Action(1) == 0)
					return;
			}
			OLM_SL_step ++;
			break;
		case 3://
			if(systemPara.givenOnceTriggerByUIorIO != 3)
			{
				if(IBIO_getInput(1) == 0)
				{
					if(PTZK_Action(1) == 0)
						return;
				}
			}
			else
			{
				if(PTZK_Action(1) == 0)
					return;
			}
			OLM_SL_step ++;
			break;
		case 4:
			if(systemPara.givenOnceTriggerByUIorIO != 3)
			{
				if(IBIO_getInput(1) == 0)
				{
					if(PTSS_Action(1) == 0)
						return;
				}
			}
			else
			{
				if(PTSS_Action(1) == 0)
					return;
			}
			OLM_SL_step ++;
			break;
		case 5:
			if(systemPara.givenOnceTriggerByUIorIO != 3)
			{
				if(IBIO_getInput(2) == 0)
				{
						return;
				}
			}
			OLM_SL_step ++;
			break;
		case 6:
			if(IBIO_getInput(4) == 1)
			{
				if(CutLable(0) == 0)
					return;
				OLM_SL_step ++;
			}
			else
			{
				OLM_SL_step +=2;
			}
			break;
		case 7:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				OLM_SL_step ++;
			}
			break;
		case 8://拉胶
			GUI_mainMessageDisp("送料中......", 12);
			distance = (ADDR_5000_H[1] << 8) | ADDR_5000_L[1];
			systemPara.SLmode = 0;//位置模式
			if(systemPara.givenOnceTriggerByUIorIO == 3)
				OLM_SL_step +=2;
			else
				MC_motorMoveDistance(GIVEN_MOTOR, 1, distance);//设置100，实际走了98.65
			OLM_SL_step ++;
			break;
		case 9://等待拉胶完成
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				if(IBIO_getInput(1) == 0)
				{
					GUI_mainMessageDisp("报警信息：送料失败！", 30);
					Alarm(0);
					return;
				}
				OLM_SL_step ++;
			}
			break;
		case 10:
			if(systemPara.givenOnceTriggerByUIorIO != 3)
			{
				if((systemPara.isFeedInPlaceEnable == 1 && IBIO_getInput(2) == 0) || (systemPara.isFeedInPlaceEnable == 0 && IBIO_getInput(2) == 1))
				{
					songliao_count ++;
					len = sprintf(str,"%d",songliao_count);
					GUI_showText(0x8000, "        ", 8);
					GUI_showText(0x8000, str, len);
					systemPara.givenOK = 1;
					systemPara.RSTOK = 0;
					systemPara.CUTOK = 0;
					GUI_mainMessageDisp("送料完成！", 9);
//					EXO_sigON(2);
				}
				else
				{
					GUI_mainMessageDisp("报警信息：送料失败！", 30);
					Alarm(0);
				}

				if(systemPara.LackMaterral == 0 && systemPara.status == STATUS_ONLINE)
				{
					systemPara.LackMaterral = 1;
					ERR_INFO_OVRE_MATERIAL();
					EXO_sigON(4);
				}
			}
			else
			{
				replacenment_flag = 1;
				GUI_runStatusDisp(0);
				systemPara.Initstatus = 0;
				GUI_mainMessageDisp("换料动作完成.", 13);
			}

			systemPara.givenOnceTriggerByUIorIO = 0;
			systemPara.RunStatus = 0;
			if(systemPara.status == STATUS_ONLINE)
			{
				systemPara.CUTOnceTriggerByUIorIO = 1;
			}
			OLM_SL_step = 0;
			break;

	}
}

/**
 * @brief 复位动作
 */
void RST_Step_task(void *pv)
{
	uint32_t limitPulseCount;//后退距离
	uint32_t offsetPulseCount;//原点补偿
	uint32_t limitPulseCount1;//收料行程
	uint32_t distance;
	uint32_t RunSpeed;
	uint32_t goHomeSpeed;

	switch(OLM_RST_step)
	{
		case 0://等待复位信号
			if(systemPara.status != STATUS_FREERUN)
			{
				if (!systemPara.RSTOnceTriggerByUIorIO || systemPara.RunStatus != 0)
				{
					systemPara.RSTOnceTriggerByUIorIO = 0;
					return;
				}
			}
			GUI_mainMessageDisp("开始复位动作.", 13);
			systemPara.RunStatus = 4;
			systemPara.givenOK = 0;
			systemPara.CUTOK = 0;
			systemPara.RSTOK = 0;
			OLM_RST_step ++;
			break;
		case 1://清除交互信号
			EXO_sigOFF(1);
			EXO_sigOFF(2);
			OLM_RST_step ++;
			break;
		case 2://后气缸夹紧
			if(IBIO_getInput(1) == 0)
			{
				OLM_RST_step= 11;
			}
			else
			{
				OLM_RST_step ++;
			}
			break;
		case 3:
			if(JL_Action(1) == 0)
				return;
			OLM_RST_step ++;
			break;
		case 4:
			OLM_RST_step ++;
			break;
		case 5:

			OLM_RST_step ++;
			break;
		case 6://复位、
			GUI_mainMessageDisp("复位中......", 12);
			//传感模式加补偿
			offsetPulseCount = MC_mm2pulse(GIVEN_MOTOR,((ADDR_5000_H[9] << 8) | ADDR_5000_L[9]));
			RunSpeed = ((ADDR_5000_H[0] << 8) | ADDR_5000_L[0]) * 0.01f;//读取运行速度

			if( (RunSpeed <= 20) )
				distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[0]);
			else if( (RunSpeed > 20) &&  ( RunSpeed <= 40))
				distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[1]);
			else if( (RunSpeed > 40) &&  ( RunSpeed <= 60))
				distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[2]);
			else if( (RunSpeed > 60) &&  ( RunSpeed <= 80))
				distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[3]);
			else if( (RunSpeed > 80) &&  ( RunSpeed <= 100))
				distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[4]);
			else if( (RunSpeed > 100) &&  ( RunSpeed <= 120))
				distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[5]);
			else if( (RunSpeed > 120) &&  ( RunSpeed <= 140))
				distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[6]);
			else
				distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed1[7]);

			systemPara.SLmode = 1;//传感模式
			MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, 0, distance,1600);//1600表示40MM

			//定长模式
//			offsetPulseCount = MC_mm2pulse(GIVEN_MOTOR,((ADDR_5000_H[1] << 8) | ADDR_5000_L[1]));
//			//切料后退距离
//			limitPulseCount = MC_mm2pulse(GIVEN_MOTOR,((ADDR_5000_H[18] << 8) | ADDR_5000_L[18]));
//			distance = limitPulseCount + offsetPulseCount;
//			MC_motorMoveDistance(GIVEN_MOTOR, 1, distance);
			OLM_RST_step ++;
			break;
		case 7://等待电机停止

			OLM_RST_step ++;
			break;
		case 8://等待复位完成
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				OLM_RST_step ++;
			}
			break;
		case 9:
			if(JL_Action(0) == 0)
				return;
			OLM_RST_step ++;
			break;
		case 10:
			delay_ms(600);
			OLM_RST_step ++;
			break;
		case 11:
			if(IBIO_getInput(1) == 0)
			{
				OLM_RST_step ++;
			}
			else
				OLM_RST_step = 3;
			break;
		case 12:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("复位完成！", 9);
			else
				GUI_mainMessageDisp("Reset complete!", 15);
			EXO_sigON(5);
			systemPara.RunStatus = 0;
			systemPara.givenOK = 0;
			systemPara.CUTOK = 0;
			if(systemPara.RSTOnceTriggerByUIorIO == 1)
				systemPara.RSTOK = 1;
			if(systemPara.RSTOnceTriggerByUIorIO == 2)
				systemPara.RSTOK = 2;
			Cut_flag = 0;
			if(systemPara.RSTOnceTriggerByUIorIO != 3)
				systemPara.givenOnceTriggerByUIorIO = 1;
			systemPara.RSTOnceTriggerByUIorIO = 0;
			OLM_RST_step = 0;
			break;
	}
}

/**
 @brief 自动模式函数调用，分功能
 */
void GivenMaterialAuto(void *pv)
{
	if(systemPara.sensorChosen == 1)
	{
		//autoMode_Step_task(pv);
	}
}

/**
@breif 裁切动作
 */
void CutLable_Step_level(void *pv)
{
	uint32_t distance;
	switch(CUT_step)
	{
		case 0:
			if(systemPara.status != STATUS_FREERUN)
			{
//				if(EXIO_getInput(0) == 0 )//
//				{
//					GUI_mainMessageDispIsolate("等待设备关闭拉胶信号.", 23);
//					systemPara.givenOnceTriggerByUIorIO = 0;
//					return;
//				}
				if (!systemPara.CUTOnceTriggerByUIorIO || systemPara.RunStatus != 0)
				{
					systemPara.CUTOnceTriggerByUIorIO = 0;
					return;
				}
				if(IBIO_getInput(4) == 1)
				{
					GUI_showMessage("提示信息：不在原点位置禁止切胶！", 32);
					systemPara.CUTOnceTriggerByUIorIO = 0;
					return;
				}
			}
			systemPara.RunStatus = 3;
			systemPara.CUTOK = 0;
			GUI_mainMessageDisp("开始裁切动作.", 13);
			CUT_step ++;
			break;
		case 1:
			if(PTZK_Action(0) == 0)
				return;
			CUT_step ++;
			break;
		case 2:
			if(CutLable(1) == 0)
				return;
			CUT_step ++;
			break;
		case 3:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				if(IBIO_getInput(4) == 0)
				{
					GUI_mainMessageDisp("裁切失败！.", 9);
					Alarm(0);
					return;
				}
				GUI_mainMessageDisp("裁切完成.", 8);
				CUT_step ++;
			}
			break;
		case 4:
			delay_ms(200);
			if(JL_Action(1) == 0)
				return;
			CUT_step ++;
			break;
		case 5:
			delay_ms(200);
			CUT_step ++;
			break;
		case 6:
			if(IBIO_getInput(6) == 0)
			{
				if(PTSS_Action(0) == 0)
					return;
				CUT_step ++;
			}
			else
			{
				GUI_mainMessageDisp("报警信息：真空不达标，物料掉落", 9);
				Alarm(0);
				return;
			}
			break;
		case 7:
//			EXO_sigOFF(2);
//			EXO_sigON(1);
			CUT_step ++;
			break;
		case 8:
//			if(systemPara.status == STATUS_ONLINE)
//			{
//				GUI_mainMessageDisp("等待断开裁切信号.", 17);
//				if(EXIO_getInput(1) == 1)
//				{
//					delay_ms(2);
//					if(EXIO_getInput(1) == 1)
//					{
//						EXO_sigOFF(1);
//					}
//				}
//				else
//				{
//					return;
//				}
//			}
			GUI_mainMessageDisp("送料完成.", 8);
			EXO_sigON(2);
			systemPara.RunStatus = 0;
			systemPara.givenOK = 0;
			systemPara.RSTOK = 0;
			systemPara.CUTOK = 1;
			systemPara.CUTOnceTriggerByUIorIO = 0;
			CUT_step = 0;
			break;

	}
}
//切刀复位
void CutLable_Step_RST(void *pv)
{
	uint32_t distance;
	switch(CUT_RST_step)
	{
		case 0:
			if(systemPara.status != STATUS_FREERUN)
			{
//				if(EXIO_getInput(0) == 0 )//
//				{
//					GUI_mainMessageDispIsolate("等待设备关闭拉胶信号.", 23);
//					systemPara.givenOnceTriggerByUIorIO = 0;
//					return;
//				}
				if (!systemPara.RSTCUTOnceTriggerByUIorIO || systemPara.RunStatus != 0)
				{
					systemPara.RSTCUTOnceTriggerByUIorIO = 0;
					return;
				}
				if(IBIO_getInput(4) == 0)
				{
					GUI_showMessage("提示信息：裁切电机在原点位置！", 32);
					systemPara.RSTCUTOnceTriggerByUIorIO = 0;
					return;
				}
			}
			systemPara.RunStatus = 3;
			GUI_mainMessageDisp("开始裁切复位动作.", 13);
			CUT_RST_step ++;
			break;
		case 1:
			if(PTZK_Action(1) == 0)
				return;
			CUT_RST_step ++;
			break;
		case 2:
			if(CutLable(0) == 0)
				return;
			CUT_RST_step ++;
			break;
		case 3:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				if(IBIO_getInput(4) == 1)
				{
					GUI_mainMessageDisp("裁切复位失败！.", 9);
					Alarm(0);
					return;
				}
				GUI_mainMessageDisp("裁切复位完成.", 8);
				CUT_RST_step ++;
			}
			break;
		case 4:
			if(PTSS_Action(1) == 0)
				return;
			CUT_RST_step ++;
			break;
		case 5:
			GUI_mainMessageDisp("复位完成.", 8);
			EXO_sigON(2);
			systemPara.RunStatus = 0;
			systemPara.RSTCUTOnceTriggerByUIorIO = 0;
			CUT_RST_step = 0;
			break;

	}
}
/**
 @breif 裁切动作
 */
uint8_t CutLable(uint8_t Action)
{
	uint32_t distance1;

	// 如果未执行初始化, 不能执行该动作
//	if (systemPara.Initstatus == STATUS_UNINIT)
//	{
//		return 0;
//	}
	distance1 = (ADDR_5000_H[18] << 8) | ADDR_5000_L[18];
	if(Action == 1 && IBIO_getInput(4) == 0)
	{
		GUI_mainMessageDisp("裁切电机裁切动作.", 17);
		systemPara.UPSLmode = 0;
		MC_motorMoveDistance(BO_MOTOR, 0, distance1);
	}
	else if(Action == 0 && IBIO_getInput(4) == 1)
	{
		GUI_mainMessageDisp("裁切电机复位动作.", 17);
		QQ_speed_l = ADDR_5000_L[19];
		QQ_speed_h = ADDR_5000_H[19];
		// 将回原点速度替换裁切电机速度
		ADDR_5000_L[19] = ADDR_5000_L[8];
		ADDR_5000_H[19] = ADDR_5000_H[8];
		systemPara.UPSLmode = 1;
		MC_motorMoveDistance(BO_MOTOR, 1, distance1*2);
		//电机速度还原
		ADDR_5000_L[19] = QQ_speed_l;
		ADDR_5000_H[19] = QQ_speed_h;
		QQ_speed_l = 0;
		QQ_speed_h = 0;

	}
	return 1;
}
/**
 * @brief 裁切电机回原点动作
 */
uint8_t GoHome_CutLable(void *pv)
{
	uint32_t limitPulseCount;
	uint32_t offsetPulseCount;
	uint32_t distance;
	uint32_t goHomeSpeed;

	switch(Gome_CutLable)
	{
		case 0://等待复位信号
			if(!systemPara.CQdoGome )
			{
				systemPara.CQdoGome = 0;
				return;
			}
			Gome_CutLable ++;
			break;
		case 1:
			Gome_CutLable ++;
			break;
		case 2:
			if(IBIO_getInput(4) == 0)
			{
				GUI_mainMessageDisp("避开原点...", 11);
				// 暂存裁切电机速度
				QQ_speed_l = ADDR_5000_L[4];
				QQ_speed_h = ADDR_5000_H[4];
				// 将回原点速度替换裁切电机速度
				ADDR_5000_L[4] = ADDR_5000_L[8];
				ADDR_5000_H[4] = ADDR_5000_H[8];
				systemPara.UPSLmode = 2;//传感模式
				MC_motorMoveDistance(BO_MOTOR, 1, 1000);
				//电机速度还原
				ADDR_5000_L[4] = QQ_speed_l;
				ADDR_5000_H[4] = QQ_speed_h;
				QQ_speed_l = 0;
				QQ_speed_h = 0;
			}
			else
			{
				limitPulseCount = MC_mm2pulse(BO_MOTOR,(ADDR_5000_H[1] << 8) | ADDR_5000_L[1]);
				offsetPulseCount = MC_mm2pulse(BO_MOTOR,((ADDR_5000_H[9] << 8) | ADDR_5000_L[9]));
				goHomeSpeed = ((ADDR_5000_H[8] << 8) | ADDR_5000_L[8]) * 0.01f;
				if( (goHomeSpeed <= 10) )
					distance = offsetPulseCount + MC_mm2pulse(BO_MOTOR,offsetSpeed[0]);
				else if( (goHomeSpeed > 10) &&  ( goHomeSpeed <= 25))
					distance = offsetPulseCount + MC_mm2pulse(BO_MOTOR,offsetSpeed[1]);
				else if( (goHomeSpeed > 25) &&  ( goHomeSpeed <= 40))
					distance = offsetPulseCount + MC_mm2pulse(BO_MOTOR,offsetSpeed[2]);
				else if( (goHomeSpeed > 40) &&  ( goHomeSpeed <= 55))
					distance = offsetPulseCount + MC_mm2pulse(BO_MOTOR,offsetSpeed[3]);
				else if( (goHomeSpeed > 55) &&  ( goHomeSpeed <= 65))
					distance = offsetPulseCount + MC_mm2pulse(BO_MOTOR,offsetSpeed[4]);
				else if( (goHomeSpeed > 65) &&  ( goHomeSpeed <= 75))
					distance = offsetPulseCount + MC_mm2pulse(BO_MOTOR,offsetSpeed[5]);
				else if( (goHomeSpeed > 75) &&  ( goHomeSpeed <= 85))
					distance = offsetPulseCount + MC_mm2pulse(BO_MOTOR,offsetSpeed[6]);
				else
					distance = offsetPulseCount + MC_mm2pulse(BO_MOTOR,offsetSpeed[7]);

				GUI_mainMessageDisp("检测原点...", 11);
				// 暂存裁切电机速度
				QQ_speed_l = ADDR_5000_L[6];
				QQ_speed_h = ADDR_5000_H[6];
				// 将回原点速度替换裁切电机速度
				ADDR_5000_L[6] = ADDR_5000_L[8];
				ADDR_5000_H[6] = ADDR_5000_H[8];
				systemPara.UPSLmode = 1;//传感模式
				MC_motorRunAndOffsetByEXTI(BO_MOTOR, 0, offsetPulseCount, 200000);
				//电机速度还原
				ADDR_5000_L[6] = QQ_speed_l;
				ADDR_5000_H[6] = QQ_speed_h;
				QQ_speed_l = 0;
				QQ_speed_h = 0;
			}
			Gome_CutLable ++;
			break;
		case 3:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				if(IBIO_getInput(4) == 1 && systemPara.UPSLmode == 1)
				{
					systemPara.UPSLmode = 0;
					ERR_INFO_GO_ORG_TIMEOUT();
					Alarm(0);
					return 0;
				}
				else if(IBIO_getInput(4) == 0 && systemPara.UPSLmode == 2)
				{
					systemPara.UPSLmode = 0;
					ERR_INFO_AVOID_ORG_TIMEOUT();
					Alarm(0);
					return 0;
				}
				else if(IBIO_getInput(4))
				{
					Gome_CutLable = 2;
				}
				else
				{
					Gome_CutLable ++;
				}
			}

			break;
		case 4:
			systemPara.CQdoGome = 0;
			GUI_mainMessageDisp("回原点完成.", 11);
			Gome_CutLable = 0;
			break;

	}
}

/**
 * @brief 回原点动作
 */
uint8_t GoHome_step(void *pv)
{
	uint32_t limitPulseCount;
	uint32_t offsetPulseCount;
	uint32_t distance;
	uint32_t goHomeSpeed;
//	uint8_t cw;
//	volatile uint8_t backupMicroStep;
	switch(Gome_step)
	{
		case 0://等待复位信号
			if(!systemPara.doGome )
			{
				systemPara.doGome = 0;
				return;
			}
			if(systemPara.RunStatus != 1)//不是初始化
				systemPara.RunStatus = 4;
			Gome_step ++;
			break;
		case 1:

			Gome_step ++;
			break;
		case 2:
			if(IBIO_getInput(1) == 0)
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("避开原点...", 11);
				else if(systemPara.Language == 1)
					GUI_mainMessageDispIsolate("Avoid the origin...", 19);
				// 暂存剥料电机速度
				QQ_speed_l = ADDR_5000_L[0];
				QQ_speed_h = ADDR_5000_H[0];
				// 将回原点速度替换剥料电机速度
				ADDR_5000_L[0] = ADDR_5000_L[8];
				ADDR_5000_H[0] = ADDR_5000_H[8];
				systemPara.SLmode = 2;//传感模式
				MC_motorMoveDistance(GIVEN_MOTOR, 1, 1000);
				//电机速度还原
				ADDR_5000_L[0] = QQ_speed_l;
				ADDR_5000_H[0] = QQ_speed_h;
				QQ_speed_l = 0;
				QQ_speed_h = 0;
			}
			else
			{
				limitPulseCount = MC_mm2pulse(GIVEN_MOTOR,(ADDR_5000_H[1] << 8) | ADDR_5000_L[1]);
				offsetPulseCount = MC_mm2pulse(GIVEN_MOTOR,((ADDR_5000_H[9] << 8) | ADDR_5000_L[9]));
				goHomeSpeed = ((ADDR_5000_H[8] << 8) | ADDR_5000_L[8]) * 0.01f;
				if( (goHomeSpeed <= 10) )
					distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[0]);
				else if( (goHomeSpeed > 10) &&  ( goHomeSpeed <= 25))
					distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[1]);
				else if( (goHomeSpeed > 25) &&  ( goHomeSpeed <= 40))
					distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[2]);
				else if( (goHomeSpeed > 40) &&  ( goHomeSpeed <= 55))
					distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[3]);
				else if( (goHomeSpeed > 55) &&  ( goHomeSpeed <= 65))
					distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[4]);
				else if( (goHomeSpeed > 65) &&  ( goHomeSpeed <= 75))
					distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[5]);
				else if( (goHomeSpeed > 75) &&  ( goHomeSpeed <= 85))
					distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[6]);
				else
					distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[7]);

				if(systemPara.Language == 0)
					GUI_mainMessageDisp("检测原点...", 11);
				else if(systemPara.Language == 1)
					GUI_mainMessageDispIsolate("Back to origin.", 15);
				// 暂存剥料电机速度
				QQ_speed_l = ADDR_5000_L[0];
				QQ_speed_h = ADDR_5000_H[0];
				// 将回原点速度替换剥料电机速度
				ADDR_5000_L[0] = ADDR_5000_L[8];
				ADDR_5000_H[0] = ADDR_5000_H[8];
				systemPara.SLmode = 1;//传感模式
				MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, 0, offsetPulseCount, 200000);
				//电机速度还原
				ADDR_5000_L[0] = QQ_speed_l;
				ADDR_5000_H[0] = QQ_speed_h;
				QQ_speed_l = 0;
				QQ_speed_h = 0;
			}
			Gome_step ++;
			break;
		case 3:
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				if(IBIO_getInput(1) == 1 && systemPara.SLmode == 1)
				{
					systemPara.SLmode = 0;

					if(systemPara.Language == 0)
						ERR_INFO_GO_ORG_TIMEOUT();
					else if(systemPara.Language == 1)
						EN_ERR_INFO_GO_ORG_TIMEOUT();
					//systemPara.AlarmFlag = 4;
					Alarm(0);
					return 0;
				}
				else if(IBIO_getInput(1) == 0 && systemPara.SLmode == 2)
				{
					systemPara.SLmode = 0;

					if(systemPara.Language == 0)
						ERR_INFO_AVOID_ORG_TIMEOUT();
					else if(systemPara.Language == 1)
						EN_ERR_INFO_AVOID_ORG_TIMEOUT();
					//systemPara.AlarmFlag = 3;
					Alarm(0);
					return 0;
				}
				else if(IBIO_getInput(1))
				{
					Gome_step = 2;
				}
				else
				{
					Gome_step ++;
				}
			}

			break;
		case 4:
			systemPara.doGome = 0;

			if(systemPara.Language == 0)
				GUI_mainMessageDisp("回原点完成.", 11);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("Back to origin complete.", 24);
			if(systemPara.RunStatus != 1 && systemPara.status != STATUS_FREERUN)//不是初始化和空跑模式
				systemPara.RunStatus = 0;
			Gome_step = 0;
			break;
	}
}

/**
 * @brief 回原点动作
 */
void GoHome(void *pv)
{
	uint32_t offsetPulseCount;
	uint32_t distance;
	uint32_t goHomeSpeed;
	uint8_t cw;
	volatile uint8_t backupMicroStep;

	if(motor_ch[GIVEN_MOTOR].status != Motor_Stop)
		return;
	// 暂存拉料电机速度
	QQ2_speed_l = ADDR_5000_L[0];
	QQ2_speed_h = ADDR_5000_H[0];
	// 将回原点速度替换剥料电机速度
	ADDR_5000_L[0] = ADDR_5000_L[8];
	ADDR_5000_H[0] = ADDR_5000_H[8];

	systemPara.isWorkTaskRun = 1;
	while (IBIO_getInput(1) == 0)
	{
//		MC_motorMoveDistance(GIVEN_MOTOR, 0, 1000);
		systemPara.SLmode = 2;//传感模式
		MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, 0, 40, 1000);
		while (motor_ch[GIVEN_MOTOR].status != Motor_Stop)
		{
			if(systemPara.isWorkTaskRun == 0)
			{
				MC_motorStop(GIVEN_MOTOR);
				return;
			}
//			if(IBIO_getInput(1) == 1)
//			{
//				delay_ms(100);
//				if(IBIO_getInput(1) == 1)
//				{
//					MC_motorStop(GIVEN_MOTOR);
//					break;
//				}
//			}
		}

		if (motor_ch[GIVEN_MOTOR].stopEvent == EVENT_exceedPulseCountLimit)
		{
			// 电机速度还原
			ADDR_5000_L[0] = QQ2_speed_l;
			ADDR_5000_H[0] = QQ2_speed_h;

			QQ2_speed_l = 0;
			QQ2_speed_h = 0;
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("报警信息：避开原点信号失败！", 30);
			else
				GUI_mainMessageDisp("Alarm message: Failed to avoid the origin signal!", 49);
			//Alarm();
			return;
		}
		if(systemPara.isWorkTaskRun == 0)
		{
			MC_motorStop(GIVEN_MOTOR);
			return;
		}
	}

	offsetPulseCount = MC_mm2pulse(GIVEN_MOTOR,((ADDR_5000_H[9] << 8) | ADDR_5000_L[9]));
	goHomeSpeed = ((ADDR_5000_H[8] << 8) | ADDR_5000_L[8]) * 0.01f;
	if( (goHomeSpeed <= 10) )
		distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[0]);
	else if( (goHomeSpeed > 10) &&  ( goHomeSpeed <= 25))
		distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[1]);
	else if( (goHomeSpeed > 25) &&  ( goHomeSpeed <= 40))
		distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[2]);
	else if( (goHomeSpeed > 40) &&  ( goHomeSpeed <= 55))
		distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[3]);
	else if( (goHomeSpeed > 55) &&  ( goHomeSpeed <= 65))
		distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[4]);
	else if( (goHomeSpeed > 65) &&  ( goHomeSpeed <= 75))
		distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[5]);
	else if( (goHomeSpeed > 85) &&  ( goHomeSpeed <= 95))
		distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[6]);
	else
		distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[7]);

	systemPara.SLmode = 1;//传感模式
	MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, 1, distance, 200000);

	while (motor_ch[GIVEN_MOTOR].status != Motor_Stop)
	{
		if(systemPara.isWorkTaskRun == 0)
		{
			return;
		}
	}
	// 剥料电机速度还原
	ADDR_5000_L[0] = QQ2_speed_l;
	ADDR_5000_H[0] = QQ2_speed_h;

	QQ2_speed_l = 0;
	QQ2_speed_h = 0;

	if (motor_ch[GIVEN_MOTOR].stopEvent == EVENT_exceedPulseCountLimit)
	{
		if(systemPara.Language == 0)
			GUI_mainMessageDisp("报警信息：未检测到原点信号！", 28);
		else
			GUI_mainMessageDisp("Alarm message: No origin signal detected!", 41);
		//Alarm();
		return;
	}

	if(systemPara.Initstatus == 0)
		systemPara.isWorkTaskRun = 0;
}


/**
 空跑模式
 */
void KongPaoMoShi(void *pv)
{
	char str[20];
	uint16_t len;

	switch(KP_step)
	{
		case 0:
			KP_step ++;
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("提示信息：即将运行空跑动作，请注意！", 36);
			else
				GUI_mainMessageDisp("Tips:Aging mode is running!", 27);
			break;
		case 1:
			KP_count ++;
			len = sprintf(str,"%d",KP_count);
			GUI_showText(0x8000, "        ", 8);
			GUI_showText(0x8000, str, len);
			KP_step ++;
			break;
		case 2:
			delay_ms(500);
			KP_step ++;
			break;
		case 3:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("执行送料动作.", 13);
			else
				GUI_mainMessageDisp("Execute the feeding action.", 27);
			SL_Step_task(0);

			if(systemPara.RunStatus ==0)
			{
				delay_ms(500);
				KP_step ++;
			}

			break;
		case 4:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("执行裁切动作.", 13);
			else
				GUI_mainMessageDisp("Perform a cut action.", 21);
			CutLable_Step_level(0);
			if(systemPara.RunStatus ==0)
			{
				delay_ms(500);
				KP_step ++;
			}
			break;
		case 5:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("执行复位动作.", 13);
			else
				GUI_mainMessageDisp("Perform reset action.", 21);
			RST_Step_task(0);
			if(systemPara.RunStatus ==0)
			{
				delay_ms(500);
				KP_step ++;
			}
			break;
		case 6:
			KP_step ++;
			break;
		case 7:
			KP_step = 0;
			break;
	}
}

/**
 @brief 初始化
 */
void ChuShiHua(void *pv)
{
	switch(Init_step)
	{
		case 0://等待初始化信号
			if(!systemPara.doInit || systemPara.RunStatus != 0)
			{
				if(systemPara.status == STATUS_AUTO)
					systemPara.doInit = 0;
				return;
			}

			MC_motorStopAll();
			Given_EN(0);                //关闭使能，清除驱动器报警信号
			Reset_Given_Alarm();
			delay_ms(50);               //延时打开使能
			Given_EN(1);

			LV8731V_cmd(GIVEN_MOTOR, 1);
			LV8731V_cmd(BO_MOTOR, 1);

			if(systemPara.isUpShouEnable)
				LV8731V_cmd(UP_MOTOR, 1);
			if(systemPara.isDownShouEnable)
				LV8731V_cmd(DOWN_MOTOR, 1);
			if(systemPara.isAutoLetMetalEnable)
				LV8731V_cmd(LET_MOTOR, 1);

			GUI_runStatusDisp(0);
			systemPara.RunStatus = 1;
			systemPara.RSTOK = 0;

			Init_step ++;
			break;
		case 1://清除交互信号
			EXO_sigOFF(1);
			EXO_sigOFF(2);
			EXO_sigOFF(3);
			EXO_sigOFF(4);
			EXO_sigOFF(5);
			systemPara.LackMaterral = 1;
			GUI_mainMessageDisp("清除交互信号.", 13);
			delay_ms(100);
			Init_step ++;
			break;
		case 2:
			if(PTZK_Action(1) == 0)
				return;
			Init_step ++;
			break;
		case 3:
			if(PTSS_Action(1) == 0)
				return;
			Init_step ++;
			break;
		case 4:
			if(JL_Action(0) == 0)
				return;
			Init_step ++;
			break;
		case 5:
			if(ZP_Action(0)== 0)
				return ;
			Init_step ++;
			break;
		case 6:
			GUI_mainMessageDispIsolate("裁切回原点中...", 11);
			systemPara.CQdoGome = 1;
			Init_step ++;
			break;
		case 7:
			if(systemPara.CQdoGome != 0)
			{
				if(GoHome_CutLable(0) == 0)
				{
					return;
				}
			}
			else
			{
				Init_step ++;
			}
			break;
		case 8:
			if(CutLable(1) == 0)
				return;
			Init_step ++;
			break;
		case 9:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				Init_step ++;
			}
			break;
		case 10:
			if(JL_Action(1) == 0)
				return;
			Init_step ++;
			break;
		case 11://回原点
			GUI_mainMessageDispIsolate("送料电机回原点中...", 11);
			systemPara.doGome = 1;
			Init_step ++;
			break;
		case 12:
			if(systemPara.doGome != 0)
			{
				if(GoHome_step(0)== 0)
				{
					return;
				}
			}
			else
				Init_step ++;
			break;
		case 13:
			if(JL_Action(0) == 0)
				return;
			Init_step ++;
			break;
		case 14:
			if(CutLable(0) == 0)
				return;
			Init_step ++;
			break;
		case 15:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				Init_step ++;
			}
			break;
		case 16:
			GUI_mainMessageDisp("初始化完成！", 11);
			EXO_sigON(3);
			GUI_runStatusDisp(1);
			systemPara.doInit = 0;
			Cut_flag = 0;
			systemPara.givenOK = 0;
			systemPara.CUTOK = 0;
			systemPara.RSTOK = 0;
			systemPara.isWorkTaskRun = 1;
			systemPara.RunStatus = 0;
			systemPara.Initstatus = 1;
			Init_step = 0;
			break;
	}
}


/**
 收料动作
 */

void slTask_handler(void *pv)
{
	uint32_t limitPulseCount;
	uint32_t limitPulseCount1;


	if (systemPara.isUpShouEnable)
	{
		if(motor_ch[UP_MOTOR].status == Motor_Stop)
		{
			limitPulseCount = (ADDR_5000_H[7]<<8)|ADDR_5000_L[7];
			MC_motorMoveDistance(UP_MOTOR, 0,limitPulseCount);
		}
	}
//	if (systemPara.isDownShouEnable)
//	{
//		if(IBIO_getInput(6) == 1)
//		{
//			limitPulseCount1 = (ADDR_5000_H[5]<<8)|ADDR_5000_L[5];
//			MC_motorMoveDistance(DOWN_MOTOR, 0,limitPulseCount1);
//		}
//	}
}
void slUpTask_handler(void *pv)
{
	uint32_t limitPulseCount;

	if (systemPara.isUpShouEnable)
	{
		if(systemPara.SLmode == 1)//放松料带
		{
			if(IBIO_getInput(8) == 0 && USLflag == 0)
			{
				USLflag = 1;
				if(motor_ch[UP_MOTOR].status != Motor_Stop)
					MC_motorStop(UP_MOTOR);
				limitPulseCount = (ADDR_5000_H[7]<<8)|ADDR_5000_L[7];
				systemPara.UPSLmode = 2;
				MC_motorMoveDistance(UP_MOTOR, 1,limitPulseCount);
			}
			if(IBIO_getInput(8) == 1)
			{
				USLflag = 0;
			}
		}
		else// if(systemPara.RunStatus != 4)//收紧料带
		{
			if(IBIO_getInput(8) == 1 && USLflag == 0)
			{
				USLflag = 1;
				if(motor_ch[UP_MOTOR].status != Motor_Stop)
					MC_motorStop(UP_MOTOR);
				systemPara.UPSLmode = 1;
				limitPulseCount = (ADDR_5000_H[7]<<8)|ADDR_5000_L[7];
				MC_motorMoveDistance(UP_MOTOR, 0,limitPulseCount);
			}
			if(IBIO_getInput(8) == 0)
			{
				USLflag = 0;
			}
		}
	}
}
//void slUpTask_handler(void *pv)
//{
//	uint32_t limitPulseCount;
//	if (systemPara.isUpShouEnable)
//	{
//		{
//			if(IBIO_getInput(7) == 0 && DSLflag == 0)
//			{
//				DSLflag = 1;
//				systemPara.isUpShouEnable = 1;
//				limitPulseCount = (ADDR_5000_H[7]<<8)|ADDR_5000_L[7];
//				MC_motorMoveDistance(UP_MOTOR, 0,limitPulseCount);
//			}
//
//			if(IBIO_getInput(7) == 1)
//			{
//				DSLflag = 0;
//			}
//		}
//	}
//}
void flTask_handler(void *pv)
{
    uint32_t limitPulseCount;

	if(systemPara.isAutoLetMetalEnable)
	{
		if(IBIO_getInput(9) == 0 && FLflag == 0)
		{
			FLflag = 1;
			FLflag1 = 0;
			systemPara.FLmode = 1;
			limitPulseCount = (ADDR_5000_H[3]<<8)|ADDR_5000_L[3];
			MC_motorMoveDistance(LET_MOTOR, 0,limitPulseCount);
		}
		if(IBIO_getInput(9) == 1)
		{
			FLflag = 0;
			FLflag1 = 1;
		}
		if(IBIO_getInput(10) == 0)
		{
			FLflag1 = 1;
		}
	}
}

void GivenMotorExceedPulseLimitAlert(void)
{
	IOUI_ALERT();
	IOUI_UNREADY();
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("报警信息：送料行程走完，未检测到有料！", 38);
	else
		GUI_mainMessageDisp("Alarm info:feeding ovre,no material is detected!", 48);
	GUI_runStatusDisp(0);
	systemPara.isWorkTaskRun = 0;
	systemPara.letOnceTrigger = 0;

//	IBIO_INPUT3_EXTI_DISABLE();
//	IBIO_INPUT4_EXTI_DISABLE();
}



void ShouMotorExceedPulseLimitAlert(void)
{
//	IOUI_ALERT();
//	IOUI_UNREADY();

//	GUI_mainMessageDisp("报警信息：上收料行程走完，上收料异常！", 38);

//	EXIO_setOutput(2, 0);
//	GUI_setOutputSignalColorDisp(9, 0);

//	GUI_runStatusDisp(0);
//	systemPara.isWorkTaskRun = 0;
//	systemPara.letOnceTrigger = 0;
//
//	motor_ch[GIVEN_MOTOR].motorHadRun = 0;
//	IBIO_INPUT5_EXTI_DISABLE();
//	IBIO_INPUT6_EXTI_DISABLE();
}

void LetMotorExceedPulseLimitAlert(void)
{
	MC_motorStop(LET_MOTOR);
	MC_motorStop(GIVEN_MOTOR);
	//IOUI_ALERT();
	IOUI_UNREADY();
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("报警信息：放料行程走完，未检测到料带！", 38);
	else
		GUI_mainMessageDisp("后夹料气缸夹紧动作.", 19);
	GUI_runStatusDisp(0);

	systemPara.isWorkTaskRun = 0;
	Alarm(0);
}
/**
 @brief 物料用尽检测报警处理
 */
void WLYJ_Alert(void)
{
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("报警信息：缺料预警，请增补物料！", 32);
	else
		GUI_mainMessageDisp("Alarm message:pre-alarm for lack of material!", 45);
	Alarm(0);
}

/**
 @brief UI控制切换紧急停止状态
 */
void EnterStopModeUI(void *pv)
{
	MC_motorStopAllEmergency();
	MC_motorStopAll();
    GUI_runStatusDisp(2);
    if(systemPara.Language == 0)
    	GUI_mainMessageDispIsolate("UI紧急停止动作！", 16);
	else
		GUI_mainMessageDispIsolate("UI E-stop action!", 17);

	__disable_irq();

	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

/**
 @brief UI停止动作
 */
void StopModeIO(void *pv)
{
	static uint8_t flag = 0;

	if(systemPara.STOPOnceTriggerByUIorIO == 0)
	{
		return;
	}

    MC_motorStopAll();
	MC_cmd(0,0);
	MC_cmd(1,0);
	MC_cmd(2,0);
	MC_cmd(3,0);
    GUI_runStatusDisp(0);

    if(systemPara.STOPOnceTriggerByUIorIO == 1)
    {
    	if(systemPara.Language == 0)
    		GUI_mainMessageDispIsolate("外部触发急停，停止所以动作！", 28);
    	else if(systemPara.Language == 1)
    		GUI_mainMessageDispIsolate("Trigger emergency stop, stop all actions!", 41);
    }
    else
    {
		if(systemPara.isInEmergencyStopEnable)
		{
			if(IBIO_getInput(5) == 1)
			{
				flag = 1;
				if(systemPara.Language == 0)
					GUI_mainMessageDispIsolate("内部触发急停，停止所以动作！", 24);
				else if(systemPara.Language == 1)
					GUI_mainMessageDispIsolate("Trigger emergency stop, stop all actions!", 41);

			}
		}
    }

	EXO_sigOFF(1);
	EXO_sigOFF(2);
	EXO_sigOFF(3);
	EXO_sigOFF(4);
	EXO_sigOFF(5);
	if(PTZK_Action(1) == 0)
		return;
	if(QQ_speed_l > 0 )
	{
		ADDR_5000_L[4] = QQ_speed_l;
		ADDR_5000_H[4] = QQ_speed_h;
	}

	if(QQ1_speed_l > 0 )
	{
		ADDR_5000_L[0] = QQ1_speed_l;
		ADDR_5000_H[0] = QQ1_speed_h;
	}

	Clear();
}

/**
 @brief UI停止动作
 */
void StopModeUI(void *pv)
{
	MC_motorStopAll();
	if(systemPara.Language == 0)
		GUI_mainMessageDispIsolate("UI停止动作！", 16);
	else
		GUI_mainMessageDisp("UI stop action.", 15);
	MC_motorStopAll();
	GUI_runStatusDisp(0);
	EXO_sigOFF(1);
	EXO_sigOFF(2);
	EXO_sigOFF(3);
	EXO_sigOFF(4);
	EXO_sigOFF(5);
	if(PTZK_Action(1) == 0)
		return;
	if(QQ_speed_l > 0 )
	{
		ADDR_5000_L[4] = QQ_speed_l;
		ADDR_5000_H[4] = QQ_speed_h;
	}
	if(QQ1_speed_l > 0 )
	{
		ADDR_5000_L[0] = QQ1_speed_l;
		ADDR_5000_H[0] = QQ1_speed_h;
	}
	Clear();
}

/**
 * 平台有料
 * * @param -no- 1 切料完成, 2 拉料完成, 3 初始化, 4 异常报警, 5复位完成
 */

void EXO_sigON(uint8_t no)
{
	if(systemPara.status != STATUS_ONLINE)
	{
		return;
	}
	switch(no)
	{
	case 1:
		EXIO_setOutput(2,0);
		GUI_setOutputSignalColorDisp(10,0);
		break;
	case 2:
		EXIO_setOutput(3, 0);
		GUI_setOutputSignalColorDisp(11,0);
		break;
	case 3:
		EXIO_setOutput(0,0);
		GUI_setOutputSignalColorDisp(8,0);
		break;
	case 4:
		EXIO_setOutput(1,0);
		GUI_setOutputSignalColorDisp(9,0);
		break;
	case 5:
//		EXIO_setOutput(4,0);
//		GUI_setOutputSignalColorDisp(12,0);
		break;
	}
}

/**
 * 关闭输出信号
 * * @param -no- 1 切料完成, 2 拉料完成, 3 初始化完成, 4 异常报警, 5 复位完成
 */
void EXO_sigOFF(uint8_t no)
{
	if(systemPara.status != STATUS_ONLINE)
	{
		return;
	}
	switch(no)
	{
	case 1:
		EXIO_setOutput(2, 1);
		GUI_setOutputSignalColorDisp(10,1);
		break;
	case 2:
		EXIO_setOutput(3, 1);
		GUI_setOutputSignalColorDisp(11,1);
		break;
	case 3:
		EXIO_setOutput(0,1);
		GUI_setOutputSignalColorDisp(8,1);
		break;
	case 4:
		EXIO_setOutput(1,1);
		GUI_setOutputSignalColorDisp(9,1);
		break;
	case 5:
//		EXIO_setOutput(4,1);
//		GUI_setOutputSignalColorDisp(12,1);
		break;
	}
}

/**
 *
 * * @param -pin-1.送料原点信号 2.送料到位信号 3.压料气缸动点信号 4.切刀电机原点信号 5.内部急停信号 6.平台真空信号 7.平台气缸原点信号 8.平台气缸动点信号

 */
//检测输入电平，并超时报警
uint8_t GetIOLevel(uint8_t pin, uint8_t level, uint16_t time)
{
	uint16_t count = 0;
	uint8_t IOlevel;
	delay_ms(50);
	for(; count <= time; count ++)
	{
		delay_ms(1);
		IOlevel = IBIO_getInput(pin);
		if(IOlevel == level)
		{
			return 1;
		}
		if(count >= time)
		{
			Alarm(0);
			return 0;
		}
	}
}

//* * @param -pin- 1 原点感应, 2 拉胶完成感应, 3 放料启动, 4 放料停止, 5 前夹料原点, 6 切刀动点, 7 压料动点, 8 备用
uint8_t GetEXIOLevel(uint8_t pin, uint8_t level, uint16_t time)
{
	uint16_t count = 0;
	uint8_t IOlevel;
	delay_ms(100);
	for(; count <= time; count ++)
	{
		delay_ms(1);
		IOlevel = EXIO_getInput(pin);
		if(IOlevel == level)
		{
			return 1;
		}
		if(count >= time)
		{
			Alarm(0);
			return 0;
		}
	}
}

//夹料气缸
uint8_t JL_Action(uint8_t Action)
{
	if(Action == 1)
	{
		GUI_mainMessageDisp("夹料气缸松开动作.", 20);
		IBIO_setOutput(1, 1);
		delay_ms(50);
		if(GetIOLevel(3, 1, 2000) == 0)
		{
			GUI_mainMessageDisp("报警信息：夹料气缸松开异常！", 31);
			return 0;
		}
	}
	else
	{
		GUI_mainMessageDisp("夹料气缸夹紧动作.", 20);
		IBIO_setOutput(1, 0);
		delay_ms(50);
		if(GetIOLevel(3, 0, 2000) == 0)
		{
			GUI_mainMessageDisp("报警信息：夹料气缸夹紧异常！", 31);
			return 0;
		}
	}
	return 1;
}
uint8_t ZP_Action(uint8_t Action)
{
	if(Action == 1)
	{
		GUI_mainMessageDisp("整平气缸松开动作.", 20);
		IBIO_setOutput(4, 1);
		GUI_setOutputSignalColorDisp(3,1);
		delay_ms(150);

	}
	else
	{
		GUI_mainMessageDisp("整平气缸夹紧动作.", 20);
		IBIO_setOutput(4, 0);
		GUI_setOutputSignalColorDisp(3,0);
		delay_ms(150);
	}
	return 1;
}

//平台伸缩气缸
uint8_t PTSS_Action(uint8_t Action)
{
	if(Action == 1)
	{
		GUI_mainMessageDisp("平台缩回动作.", 20);
		IBIO_setOutput(2, 1);
		delay_ms(200);
		if(GetIOLevel(7, 0, 2000) == 0)
		{
			GUI_mainMessageDisp("报警信息：平台缩回异常！", 31);
			return 0;
		}
	}
	else
	{
		GUI_mainMessageDisp("平台伸出动作.", 20);
		IBIO_setOutput(2, 0);
		delay_ms(200);
		if(GetIOLevel(8, 0, 2000) == 0)
		{
			GUI_mainMessageDisp("报警信息：平台伸出异常！", 31);
			return 0;
		}
	}
	return 1;
}

//平台真空控制
uint8_t PTZK_Action(uint8_t Action)
{
	if(Action == 1)
	{
	    GUI_mainMessageDisp("真空关闭.", 17);
		IBIO_setOutput(3, 1);
		delay_ms(50);
		if(GetIOLevel(6, 1, 2000) == 0)
		{
				GUI_mainMessageDisp("报警信息：真空关闭异常！", 28);
			return 0;
		}
	}
	else
	{
			GUI_mainMessageDisp("真空打开.", 17);
		IBIO_setOutput(3, 0);
		delay_ms(50);
		if(GetIOLevel(6, 0, 2000) == 0)
		{
				GUI_mainMessageDisp("报警信息：真空打开异常！", 28);
			return 0;
		}
	}
	return 1;
}



void Alarm(void *pv)
{
	if(systemPara.status == STATUS_ONLINE)
	{
		EXO_sigON(4);
		EXO_sigOFF(1);
		EXO_sigOFF(2);
		EXO_sigOFF(3);
	}
	MC_motorStopAll();
	GUI_runStatusDisp(0);
	Clear();
}
void Clear(void)
{
	systemPara.Err = 1;
	systemPara.SLOK = 0;
	systemPara.CUTOK = 0;
	systemPara.RSTOK = 0;
	systemPara.doInit = 0;
	systemPara.doGome = 0;
	systemPara.givenOK = 0;
	systemPara.RunStatus = 0;
	systemPara.LackMaterral = 1;
	systemPara.LackMaterral = 1;
	systemPara.isWorkTaskRun = 0;
	systemPara.Initstatus = STATUS_UNINIT;
	systemPara.RSTOnceTriggerByUIorIO = 0;
	systemPara.CUTOnceTriggerByUIorIO = 0;
	systemPara.STOPOnceTriggerByUIorIO = 0;
	systemPara.givenOnceTriggerByUIorIO = 0;

	if(QQ_speed_l != 0)
	{
		ADDR_5000_L[0] = QQ_speed_l;
		ADDR_5000_H[0] = QQ_speed_h;
	}
	else if(QQ1_speed_l != 0)
	{
		ADDR_5000_L[0] = QQ1_speed_l;
		ADDR_5000_H[0] = QQ1_speed_h;
	}
	else if(QQ2_speed_l != 0)
	{
		ADDR_5000_L[0] = QQ2_speed_l;
		ADDR_5000_H[0] = QQ2_speed_h;
	}
	CUT_RST_step = 0;
	KP_step = 0;
	AM_step = 0;
	FLflag1 = 0;
	OLM_step = 0;
	CUT_step = 0;
	Cut_flag = 0;
	ms_count = 0;
	Cut_flag = 0;
	Init_step = 0;
	Gome_step = 0;
	FLdistance = 0;
	OLM_SL_step = 0;
	OLM_RST_step = 0;
	Gome_CutLable = 0;
	Shut_vacuum_step = 0;
	replacenment_flag = 0;
}

void MdriveALM(void)
{
	Alarm(0);
	Given_EN(0);
	GUI_mainMessageDisp("报警信息：裁切驱动器报警.", 25);
}

uint8_t EmergencyStop(void)
{
	if(systemPara.isInEmergencyStopEnable)
	{
		if(IBIO_getInput(5) == 1 )
		{
			POP_UP_INFO_RES_E_SOTP();
			return 1;
		}
	}
	return 0;
}
//清除送料伺服报警
void Reset_Given_Alarm(void)
{
	IBIO_setOutput(6, 0);
	GUI_setOutputSignalColorDisp(5,0);
	delay_ms(50);
	IBIO_setOutput(6, 1);
	GUI_setOutputSignalColorDisp(5,1);
}

//送料电机使能
void Given_EN(uint8_t state)
{
	if(state)
	{
		IBIO_setOutput(5, 0);
		GUI_setOutputSignalColorDisp(4,0);
	}
	else
	{
		IBIO_setOutput(5, 1);
		GUI_setOutputSignalColorDisp(4,1);
	}
}
//关闭真空
void Shut_vacuum(void)
{
	switch(Shut_vacuum_step)
	{
		case 0:
			if(systemPara.status != STATUS_FREERUN)
			{
				if (!systemPara.Shutvacuum)
				{
					systemPara.Shutvacuum = 0;
					return;
				}
			}
			systemPara.RunStatus = 5;
			GUI_mainMessageDisp("开始关闭真空动作.", 13);
			Shut_vacuum_step ++;
			break;
		case 1:
			if(PTZK_Action(1) == 0)
				return;
			Shut_vacuum_step ++;
			break;
		case 2:
			if(IBIO_getInput(6) == 1)
			{
				EXO_sigON(1);
				EXO_sigOFF(2);
				GUI_mainMessageDisp("关闭真空完成。", 16);
				Shut_vacuum_step ++;
			}
			else
			{
				GUI_mainMessageDisp("关闭真空失败！", 16);
				Alarm(0);
			}
			break;
		case 3:
			if(systemPara.status == STATUS_ONLINE)
			{
				GUI_mainMessageDisp("等待断开关闭真空信号.", 17);
				if(EXIO_getInput(1) == 1)
				{
					delay_ms(2);
					if(EXIO_getInput(1) == 1)
					{
						EXO_sigOFF(1);
					}
				}
				else
				{
					return;
				}
			}
			GUI_mainMessageDisp("复位完成.", 8);
			systemPara.RunStatus = 0;
			systemPara.CUTOK = 0;
			systemPara.Shutvacuum = 0;
			Shut_vacuum_step = 0;
			break;
	}
}
//真空检测
void Detect_vacuum(void)
{
	if(IBIO_getInput(6) == 1 && systemPara.CUTOK == 1)
	{
		GUI_mainMessageDisp("提示信息：物料被取走", 20);
		EXO_sigOFF(2);
		return;
	}
	else if(IBIO_getInput(6) == 0 && systemPara.CUTOK == 1)
	{
		EXO_sigON(2);
	}
}

