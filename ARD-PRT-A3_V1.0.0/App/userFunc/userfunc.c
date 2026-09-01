#include "userfunc/userfunc.h"
#include "motorctrl/motorctrl.h"
#include "paramanager/paramanager.h"
#include "inboardio/inboardio.h"
#include <usart1/USART1.h>
#include "exio/exio.h"
#include "dwin/gui.h"
#include "pwm1/pwm1.h"
#include "delay/delay.h"
//#include "tim6/tim6.h"
#include "eventHandler/eventHandler.h"
//#include "iwdg/iwdg.h"
#include "storage/storage.h"
#include "at24cxx/at24cxx.h"
//static uint8_t CylinderIn(void);
//static uint8_t CylinderOut(void);

extern void selectAutoMode(void *pv);
extern void selectManualMode(void *pv);
extern void selectFreeRunMode(void *pv);

uint64_t KP_count = 0; //空跑计数
uint64_t songliao_count = 0; // 送料计数

uint16_t KP_step = 0;  // 空跑步骤

uint16_t Init_step = 0;
uint16_t AM_step = 0;
uint16_t ms_count = 0;

uint16_t Gohome_step = 0;
uint16_t Labeling_step = 0;
uint16_t Taking_step = 0;
uint16_t GoToLableposi_step = 0;
uint16_t GoTotakeposi_step = 0;

//uint32_t offsetSpeed[8]={230, 215, 190, 170, 150, 130, 115, 90};//初始化补偿,
uint32_t offsetSpeed[8]={115, 110, 105, 90, 75, 65, 40, 35};//初始化补偿,
//uint32_t offsetSpeed1[8]={200, 165, 130, 105, 82, 48, 15, 0};//夹料复位补偿
uint32_t offsetSpeed1[8]={120, 115, 110, 95, 80, 60, 25, 0};//夹料复位补偿


uint8_t USLflag = 0; //放松-上收料料标志位
static uint8_t SL_count = 0;


/**
 @brief 控制输出信号
 */
void controlOutputSignal(void *pv)
{
	static uint8_t flag[] =
	{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };


	if(systemPara.isInEmergencyStopEnable)
	{
		if(IBIO_getInput(4) == 1 )
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
		systemPara.doGohome = 1;
	}
	else
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE2();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE2();
	}
}



/**
 空跑模式
 */
void KongPaoMoShi(void *pv)
{
	char str[20];
	uint16_t len;
	static uint16_t delayMsCount= 0;
	static uint32_t Hold_msCount= 0;
	switch(KP_step)
	{
		case 0:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("提示信息：即将运行空跑动作，请注意！", 36);
			else
				GUI_mainMessageDisp("Tips:Aging mode is running!", 27);
			KP_step ++;
			break;
		case 1:
			KP_step ++;
			break;
		case 2:
			delay_ms(500);
			KP_step ++;
			break;
		case 3:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("执行取标动作.", 13);
			else
				GUI_mainMessageDisp("Execute the feeding action.", 27);
//			Taking(0);
//			if(!IBIO_getInput(1))
//			{//原点位置
////				systemPara.doLabelPosi  = 1;
//				systemPara.doTakePosi  = 1;
//				KP_step  ++;
//			}
//			else
//			{
				KP_step = 5;
//			}
			break;
		case 4:
			if(systemPara.doTakePosi != 0)
			{
				GotoTakePosi(0);
			}
			else
			{
				KP_step  ++;
			}
			break;
		case 5:
			Vacuum_Action(0, 0);
			Blow_Action(0);
			KP_step  ++;
			break;
		case 6://延迟一秒
			delay_ms(1);
			if(++delayMsCount < 1000)
			{
				return;
			}
			else
			{
				delayMsCount = 0;
				KP_step  ++;
			}
			break;
		case 7:
			Blow_Action(1);
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("取标完成.", 32);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Taking completion.", 18);
			KP_step  ++;
			break;
		case 8:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("执行贴标动作.", 13);
			else
				GUI_mainMessageDisp("Perform labeling actions.", 25);
			Cylinder_Action(0, 0);
			if (!IBIO_getInput(1) || !IBIO_getInput(3)) {
				KP_step = 10;
			}
//			if(systemPara.RunStatus ==0)
//			{
//				delay_ms(500);
//				KP_step ++;
//			}
//			systemPara.doLabelPosi  = 1;

			break;
		case 9:
			if(systemPara.doLabelPosi != 0)
			{
				GotoLabelPosi(0);
			}
			else
			{
				KP_step ++;
			}
			break;
		case 10://关闭真空
			Vacuum_Action(1, 0);
			KP_step  ++;
			break;
		case 11://保压
			delay_ms(1);
			if(++ Hold_msCount >= ((ADDR_5000_H[20] << 8) | ADDR_5000_L[20]))
			{
				Hold_msCount = 0;
				KP_step  ++;
			}
			break;
		case 12://回取标位
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("贴标完成.", 11);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Labeling is complete.", 21);
			KP_step  ++;
			break;
		case 13:
			Cylinder_Action(1, 1);
			KP_step  ++;
			break;
		case 14:
			KP_count ++;
			len = sprintf(str,"%d",KP_count);
			GUI_showText(0x8000, "        ", 8);
			GUI_showText(0x8000, str, len);
			KP_step ++;
			break;
		case 15:
			KP_step = 0;
			break;
	}
}

/**
 @brief 初始化
 */
void ChuShiHua(void *pv)
{
	static uint16_t ctime;
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

			LV8731V_cmd(GIVEN_MOTOR, 1);
		//	motor_ch[0].motorHadRun = 0;
			systemPara.isUpShouEnable = 1;
			if(systemPara.isUpShouEnable)
				LV8731V_cmd(UP_MOTOR, 1);
//			if(systemPara.isDownShouEnable)
//				LV8731V_cmd(DOWN_MOTOR, 1);
//			if(systemPara.isAutoLetMetalEnable)
//				LV8731V_cmd(LET_MOTOR, 1);

			GUI_runStatusDisp(0);
			systemPara.RunStatus = 1;
			systemPara.RSTOK = 0;
			systemPara.doTakePosiOK = 0;
			Init_step = 10;
			break;
		case 10://清除交互信号
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("清除交互信号.", 13);
			else
				GUI_mainMessageDisp("Clear interactive signal.", 25);
			EXO_sigOFF(1);
			EXO_sigOFF(2);
			EXO_sigOFF(3);
			EXO_sigOFF(4);
			USLflag = 0;//
			systemPara.LackMaterral = 1;
			delay_ms(50);
			Init_step = 20;
			break;
		case 20://真空关闭
			Vacuum_Action(1, 0);
			Init_step = 30;
			break;
		case 30://吹气关闭
//			Blow_Action(1);
			Init_step = 42;
			break;
//		case 40://回原点
//			systemPara.doGohome = 1;
//			Init_step = 41;
//			break;
//		case 41://回原点
//			if(systemPara.doGohome != 0)
//			{
//				if(GoHome_step(0) == 0)
//				{
//					return;
//				}
//			}
//			else
//			{
//				Init_step = 42;
//			}
//			break;
		case 42: // 缩回气缸
			if (!Cylinder_Action(1, 1)) {
				return;
			}
			Init_step = 43;
			break;

		case 43: // 打开真空检测是否有标
			//Vacuum_Action(0, 0);
			if (!IBIO_getInput(5)) {
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("报警：平台检测有物料！", 20);
				else
					GUI_mainMessageDisp("Material Detected", 17);
				Alarm(0);
				return;
			}
//			Vacuum_Action(1, 1);
			Init_step = 50;
			break;

		case 50:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("初始化完成！", 11);
			else
				GUI_mainMessageDisp("Initialization is complete!", 27);
			EXO_sigON(1);
			if(systemPara.isUpShouEnable)
			{
				slUpTask_handler(0);
			}

			GUI_runStatusDisp(1);
			systemPara.doInit = 0;
			SL_count = 0;
			USLflag = 0;
			systemPara.doTakePosiOK = 0;
			systemPara.doLabelposiOK = 0;
			systemPara.RunStatus = 0;
			systemPara.Initstatus = 1;
			systemPara.AlarmFlag = 0;
			EXO_sigON(2);
			Init_step = 0;
			break;
	}
}

/**
 * @brief Y轴回原点动作
 */
uint8_t GoHome_step(void *pv)
{

	uint32_t offsetPulseCount;
	uint32_t distance;
	uint32_t goHomeSpeed;

	switch(Gohome_step)
	{
		case 0://等待复位信号
			if(!systemPara.doGohome)
			{
				systemPara.doGohome = 0;
//				break;
				return;
			}
			if(systemPara.RunStatus != 1 && systemPara.RunStatus != 11)//不是初始化
				systemPara.RunStatus = 13;
			systemPara.doGohomeOK = 0;
			systemPara.RSTAlarm = 0;
			Gohome_step ++;
			break;
		case 1:
			Gohome_step ++;
			break;
		case 2:
			if(IBIO_getInput(1) == 0)
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("避开原点......", 14);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("turns away from the origin...", 29);

				systemPara.SLmode = 2;//传感模式
				MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, systemPara.MotorGivenDir, 0, 5000);
			}
			else
			{
				offsetPulseCount = 0;
				goHomeSpeed = ((ADDR_5000_H[8] << 8) | ADDR_5000_L[8]) * 0.001f;
				if(goHomeSpeed <= 0)
					goHomeSpeed = 1;
				distance = offsetPulseCount + MC_mm2pulse(GIVEN_MOTOR,offsetSpeed[goHomeSpeed- 1]);

				if(systemPara.Language == 0)
					GUI_mainMessageDisp("检测原点......", 14);
				else if(systemPara.Language == 	1)
					GUI_mainMessageDisp("detection origin...", 19);
				systemPara.SLmode = 1;//传感模式
				if(systemPara.RSTAlarm == 1)
					MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, systemPara.MotorGivenDir, distance, 200000);
				else if(systemPara.RSTAlarm == 0 || IBIO_getInput(2) == 0)
					MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, !systemPara.MotorGivenDir, distance, 20000);

			}
			Gohome_step  ++;
			break;
		case 3:
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				if(IBIO_getInput(1) == 1 && systemPara.SLmode == 1)
				{
					systemPara.SLmode = 0;
					if(systemPara.Language == 0)
						GUI_mainMessageDisp("报警信息：未检测到原点信号！", 32);
					else if(systemPara.Language == 1)
						GUI_mainMessageDisp("Alarm message: Origin signal not detected!", 42);
					systemPara.AlarmFlag = 4;
					Alarm(0);
					return 0;
				}
				else if(IBIO_getInput(1) == 0 && systemPara.SLmode == 2)
				{
					systemPara.SLmode = 0;
					if(systemPara.Language == 0)
						GUI_mainMessageDisp("报警信息：避开原点失败！", 24);
					else if(systemPara.Language == 1)
						GUI_mainMessageDisp("Alarm message:failed to avoid the origin!", 41);
					systemPara.AlarmFlag = 5;
					Alarm(0);
					return 0;
				}
				else if(IBIO_getInput(1) || (IBIO_getInput(1) != 0 && systemPara.RSTAlarm == 1))
				{//避开原点完成
					Gohome_step = 2;
				}
				else if(IBIO_getInput(1) == 0 && systemPara.RSTAlarm == 1)
				{//从1负限位回原点完成
					systemPara.RSTAlarm = 0;
					Gohome_step = 2;
				}
				else
				{//正常回原点完成
					systemPara.RSTAlarm = 0;
					Gohome_step ++;
				}
			}
			break;
		case 4:

			Gohome_step ++;
			break;
		case 5:
			systemPara.doGohome = 0;
			systemPara.doGohomeOK = 1;
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("回原点完成.", 11);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("return to origin complete.", 26);
			if((systemPara.RunStatus != 1 && systemPara.RunStatus != 11) && systemPara.status != STATUS_FREERUN)//不是初始化和空跑模式
				systemPara.RunStatus = 0;
			Gohome_step = 0;
			systemPara.doTakePosiOK = 0;
			systemPara.doTakePosi =0;
			systemPara.doLabelposiOK = 0;
			systemPara.doLabelPosi = 0;
			systemPara.doTaking = 0;
			systemPara.doTakingOK = 0;
			systemPara.doLabeling = 0;
			systemPara.doLabelingOK = 0;
			systemPara.GoBackTakePosi = 0;
			systemPara.AlarmFlag = 0;
			return 1;
			break;
	}
}

/***
 * 取标动作
 */
void Taking(void *pv)
{
	static uint16_t delayMsCount= 0;

	switch(Taking_step)
	{
		case 0://等待复位信号
			if(!systemPara.doTaking)
			{
				systemPara.doTaking = 0;
//				break;
				return;
			}

			systemPara.doTakingOK = 0;
			systemPara.RunStatus = 3;

			EXO_sigOFF(3); // 关闭贴标完成
			EXO_sigOFF(4); // 关闭取标完成
			Taking_step  = 20;
			break;
//		case 10://贴标
//			if(!IBIO_getInput(1))
//			{//原点位置
////				systemPara.doLabelPosi  = 1;
//				systemPara.doTakePosi  = 1;
//				Taking_step  = 11;
//			}
//			else
//			{
//				Taking_step = 20;
//			}
//			break;
//		case 11:
//			if(systemPara.doTakePosi != 0)
//			{
//				GotoTakePosi(0);
//			}
//			else
//			{
//				Taking_step  = 20;
//			}
//			break;
		case 20://打开真空
			Vacuum_Action(0, 0);
//			delay_ms(100);
			Taking_step  = 21;
			break;
		case 21://复检真空-判断是否有标签
//			if(IBIO_getInput(4) == 0)
//			{
//				delay_ms(5);
//				if(IBIO_getInput(4) == 0)
//				{
//					if(systemPara.Language == 0)
//						GUI_mainMessageDisp("报警信息：吸板检测有料，请确认！", 32);
//					else if(systemPara.Language == 1)
//						GUI_mainMessageDisp("Alarm:Suction plate detection of material, please confirm!", 58);
//					EXO_sigON(4);
//
//					IBIO_setOutput(1, 1);//关闭真空
//					GUI_setOutputSignalColorDisp(0,1);
//
//					Taking_step = 0;
//					systemPara.RunStatus = 0;
//					systemPara.doTaking = 0;
//					return;
//				}
//			}
			Taking_step  = 30;
			break;
		case 30:
//			Blow_Action(0);
			Taking_step  = 40;
			break;
		case 40:
			if(systemPara.status == STATUS_ONLINE)
			{//联机模式，等待完成打印状态
				if(!EXIO_getInput(0))
				{
					delay_ms(2);
					if(!EXIO_getInput(0))
					{
						if(systemPara.Language == 0)
							GUI_mainMessageDisp("正在打印中！", 12);
						else if(systemPara.Language == 1)
							GUI_mainMessageDisp("Printing in progress!", 21);
						return;
					}
				}
				else
				{
					Taking_step  = 50;
				}
			}
			else
			{//非联机模式延时2S
				delay_ms(1);
				if(++delayMsCount < 2000)
				{
					return;
				}
				else
				{
					delayMsCount = 0;
					Taking_step  = 50;
				}
			}
			break;
		case 50://检测真空-判断是否有料
//			Blow_Action(1);
			if(IBIO_getInput(5))
			{
				delay_ms(5);
				if(IBIO_getInput(5))
				{
					if(systemPara.Language == 0)
						GUI_mainMessageDisp("报警信息：平台检测无料，请确认！", 32);
					else if(systemPara.Language == 1)
						GUI_mainMessageDisp("Alarm:The suction plate is not materialized, please confirm!", 60);

					EXO_sigOFF(1);
					EXO_sigOFF(2);
					EXO_sigOFF(3);
					EXO_sigOFF(4);
					systemPara.AlarmFlag = 1;
					IBIO_setOutput(1, 1);
					GUI_setOutputSignalColorDisp(0,1);

					Init_step = 0;
					Taking_step = 0;
					GoTotakeposi_step =0;
					systemPara.doInit = 0;
					systemPara.RunStatus = 0;
					systemPara.doTakePosi = 0;
					systemPara.doLabelposiOK = 0;
					systemPara.doTaking = 0;
					systemPara.doLabelingOK = 0;
					return;
				}
			}
			else
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("取标完成.", 32);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Taking completion.", 18);
				if(systemPara.AlarmFlag != 2)
					EXO_sigON(4);
				EXO_sigOFF(2);
				systemPara.doTakingOK = 1;
			}
			Taking_step = 0;
			systemPara.RunStatus = 0;
			systemPara.doTaking = 0;
			break;
	}
}

/***
 * 贴标动作
 */
void Labeling(void *pv)
{
	static uint32_t Hold_msCount= 0;
	static uint32_t delay = 0;

	switch(Labeling_step)
	{
		case 0://等待复位信号
			if(!systemPara.doLabeling)
			{
				systemPara.doLabeling = 0;
//				break;
				return;
			}

			if(IBIO_getInput(5))
			{
				delay_ms(2);
				if(IBIO_getInput(5))
				{
					if(systemPara.Language == 0)
						GUI_mainMessageDisp("报警信息：请先取标！", 20);
					else if(systemPara.Language == 1)
						GUI_mainMessageDisp("Please take the label first!", 28);
					systemPara.doLabeling = 0;
					Alarm(0);
					return;
				}
			}
			systemPara.doLabelingOK = 0;
			systemPara.RunStatus = 2;

			EXO_sigOFF(3);
			EXO_sigOFF(4);
			Labeling_step  = 10;
			break;

		case 10:// 伸出气缸
			Cylinder_Action(0, 0);
			delay = 0;
			Labeling_step  = 11;
			break;

		case 11:
			if (!IBIO_getInput(3))
			{//关闭真空
				Vacuum_Action(1, 0);
				EXO_sigON(3);
				Labeling_step  = 12;
			}
			break;
		case 12:
			Labeling_step  = 20;
			break;
		case 20://保压
//			delay_ms(1);
//			if(++ Hold_msCount >= ((ADDR_5000_H[20] << 8) | ADDR_5000_L[20]))
//			{
//				Hold_msCount = 0;
				Labeling_step  = 30;
//			}
			break;
		case 30://回取标位
//			EXO_sigOFF(4);//关闭取标完成信号
//			if (!Cylinder_Action(1, 1)) {
//				return;
//			}
			Labeling_step  = 40;
			break;
		case 40://复检真空-判断是否贴上
			if(IBIO_getInput(5) == 1)
			{
				delay_ms(1);
				if(++ Hold_msCount >= ((ADDR_5000_H[20] << 8) | ADDR_5000_L[20]))
				{
					Hold_msCount = 0;
					Labeling_step  = 41;
				}
			}
			else {
				Hold_msCount = 0;
			}
			break;
		case 41:
			EXO_sigOFF(3);
			if (!Cylinder_Action(1, 1)) {
				return;
			}
			Labeling_step  = 42;
			break;
		case 42:
			EXO_sigON(2);
			Labeling_step  = 0;
//			systemPara.doLabelingOK = 1;
			systemPara.doLabeling = 0;
			systemPara.RunStatus = 0;
			systemPara.GoBackTakePosi = 0;
			break;
	}
}

/***
 * 去贴标位
 */
void GotoLabelPosi(void *pv)
{
	uint16_t distance,distance1;

	switch(GoToLableposi_step)
	{
		case 0://等待复位信号
			if(!systemPara.doLabelPosi)
			{
				systemPara.doLabelPosi = 0;
				return;
			}

			if(systemPara.doLabelposiOK)
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("当前位置为贴标位.", 9);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("The current position is the labeling.", 37);
				systemPara.doLabelPosi = 0;
				return;
			}
			systemPara.doLabelposiOK = 0;
			GoToLableposi_step ++;
			break;
		case 1:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("去贴标位.", 9);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Go to the labeling position.", 28);

			if(systemPara.Lableposition == 1) // 贴标位置1
				distance = ((ADDR_5000_H[1] << 8) | ADDR_5000_L[1]) - ((ADDR_5000_H[19] << 8) | ADDR_5000_L[19]);
			else if(systemPara.Lableposition == 2) // 贴标位置2
				distance = ((ADDR_5000_H[18] << 8) | ADDR_5000_L[18]) - ((ADDR_5000_H[19] << 8) | ADDR_5000_L[19]);

			if(IBIO_getInput(1))
			{
				MC_motorMoveDistance(GIVEN_MOTOR, systemPara.MotorGivenDir, distance);
			}else
			{//在原点位置需要加上取标位置
				distance1 = (ADDR_5000_H[19] << 8) | ADDR_5000_L[19];
				MC_motorMoveDistance(GIVEN_MOTOR, systemPara.MotorGivenDir, distance + distance1);
			}
			GoToLableposi_step ++;
			break;
		case 2:
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				GoToLableposi_step ++;
			}
			break;
		case 3:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("到达贴标位.", 11);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Arrive at the labeling position.", 32);
			if(systemPara.Lableposition == 1) // 贴标位置1
				systemPara.doLabelposiOK = 1;
			else if(systemPara.Lableposition == 2)
				systemPara.doLabelposiOK = 2;
			systemPara.doLabelPosi = 0;
			systemPara.doTakePosiOK = 0;
			GoToLableposi_step = 0;
			break;
	}
}


/***
 * 去取标位
 */
void GotoTakePosi(void *pv)
{
	uint16_t distance;
	char str[20];
	uint16_t len;
	static uint16_t delayMsCount= 0;

	switch(GoTotakeposi_step)
	{
		case 0:
			if(!systemPara.doTakePosi)
			{
				systemPara.doTakePosi = 0;
//				break;
				return;
			}

			if(systemPara.doTakePosiOK)
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("当前位置为取标位.", 9);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("The current position is the take.", 23);
				systemPara.doTakePosi = 0;
//				break;
				return;
			}
			systemPara.doTakePosiOK = 0;
			GoTotakeposi_step ++;
			break;
		case 1:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("去取标位.", 9);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Go to the Taking position.", 26);
			Cylinder_Action(0, 1);
			GoTotakeposi_step ++;
			break;
		case 2:
			if(systemPara.GoBackTakePosi == 1)
			{
				delay_ms(1);
				if(++delayMsCount < 100)
				{
					return;
				}
				else
				{
					delayMsCount = 0;
					GoTotakeposi_step ++;
				}
			}
			else
			{
				GoTotakeposi_step ++;
			}
			break;
		case 3:
			if(systemPara.GoBackTakePosi == 1)
			{
				IBIO_setOutput(1, 0);
				GUI_setOutputSignalColorDisp(0,0);
				delay_ms(1);
				if(++delayMsCount < 70)
				{
					return;
				}
				else
				{
					delayMsCount = 0;
					if(IBIO_getInput(4) == 0)
					{
						delay_ms(5);
						if(IBIO_getInput(4) == 0)
						{
							if(systemPara.Language == 0)
								GUI_mainMessageDisp("报警信息：检测未贴上，请确认！", 32);
							else if(systemPara.Language == 1)
								GUI_mainMessageDisp("Alarm:The test is not attached, please confirm!", 47);
							EXO_sigOFF(1);
							EXO_sigOFF(2);
							EXO_sigOFF(3);
							EXO_sigOFF(4);
							systemPara.AlarmFlag = 2;

							IBIO_setOutput(1, 1);
							GUI_setOutputSignalColorDisp(0,1);
						}
					}
					else
					{
						EXO_sigON(3);
						if(systemPara.Language == 0)
							GUI_mainMessageDisp("贴标完成.", 11);
						else if(systemPara.Language == 1)
							GUI_mainMessageDisp("Labeling is complete.", 21);
						systemPara.doLabelingOK = 1;
						len = sprintf(str,"%d",++songliao_count);
						GUI_showText(0x8000, "        ", 8);
						GUI_showText(0x8000, str, len);
						IBIO_setOutput(1, 1);
						GUI_setOutputSignalColorDisp(0,1);
					}
				}

			}
			GoTotakeposi_step ++;
			break;
		case 4:
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				GoTotakeposi_step ++;
			}
			break;
		case 5:
			if(systemPara.status == STATUS_FREERUN || systemPara.GoBackTakePosi == 1)
			{
				GoTotakeposi_step ++;
			}
			else
			{//打开真空复检
				Vacuum_Action(0, 0);
				delay_ms(100);
				GoTotakeposi_step ++;
			}
			break;
		case 6://复检真空-判断是否有标签
			if(systemPara.status == STATUS_FREERUN || systemPara.GoBackTakePosi == 1)
			{
				GoTotakeposi_step ++;
			}
			else
			{
				if(IBIO_getInput(4) == 0)
				{
					delay_ms(5);
					if(IBIO_getInput(4) == 0)
					{
						if(systemPara.Language == 0)
							GUI_mainMessageDisp("报警信息：吸盘检测有料，请确认！", 32);
						else if(systemPara.Language == 1)
							GUI_mainMessageDisp("Alarm:Suction plate detection of material, please confirm!", 58);
						//EXO_sigON(4);
						EXO_sigOFF(1);
						EXO_sigOFF(2);
						EXO_sigOFF(3);
						EXO_sigOFF(4);
						systemPara.AlarmFlag = 7;

						IBIO_setOutput(1, 1);//关闭真空
						GUI_setOutputSignalColorDisp(0,1);

						Taking_step = 0;
						GoTotakeposi_step =0;
						systemPara.doInit = 0;
						Init_step = 0;
						systemPara.RunStatus = 0;
						systemPara.doTakePosi = 0;
						systemPara.doLabelposiOK = 0;
						systemPara.doTaking = 0;
						return;
					}
				}
				if(systemPara.RunStatus == 1)
					Vacuum_Action(1, 0);
				GoTotakeposi_step ++;
			}
			break;
		case 7:
			if(systemPara.GoBackTakePosi != 1)
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("到达取标位.", 11);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Arrive at the Taking position.", 30);
			}
			IBIO_setOutput(1, 1);//关闭真空
			GUI_setOutputSignalColorDisp(0,1);

			if(systemPara.AlarmFlag != 2)
				EXO_sigON(2);
			systemPara.doTakePosiOK = 1;
			systemPara.doTakePosi = 0;
			systemPara.doLabelposiOK = 0;
			GoTotakeposi_step = 0;
			break;
	}
}



/**
 * 上收料动作
 *  状态 放松还是收紧料带
 */
void slUpTask_handler(void *pv)
{
	uint32_t limitPulseCount;
	static uint16_t delay_count = 0;


	if(motor_ch[UP_MOTOR].status != Motor_Stop)
		return;

	if (systemPara.isUpShouEnable)
	{
		if(IBIO_getInput(6) == 1 && USLflag == 0)
		{
			USLflag = 1;
			systemPara.UPSLmode = 1;
			limitPulseCount = (ADDR_5000_H[7]<<8)|ADDR_5000_L[7];
			MC_motorMoveDistance(UP_MOTOR, systemPara.MotorUpShouDir,limitPulseCount);

			if(++SL_count >= 5)
			{
				SL_count = 0;
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("提示信息：收料报警！", 36);
				else
					GUI_mainMessageDisp("Tips:Recving Alarm!", 27);
//				systemPara.AlarmFlag = 3;
				Alarm(0);
//				sendBuffer[21][5] = 0x03;
//				USART1_sendBuf(sendBuffer[21], 8);
			}
		}
		if(IBIO_getInput(6) == 0)
		{
			SL_count = 0;
			USLflag = 0;
		}
		if(IBIO_getInput(6) == 1)
		{
			if(++delay_count <= 100)
				delay_ms(1);
			else
			{
				delay_count = 0;
				USLflag = 0;
			}
		}
	}
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
	systemPara.AlarmFlag = 6;
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

    MC_motorStopAll();
	MC_cmd(0,0);
	MC_cmd(1,0);
	MC_cmd(2,0);
	MC_cmd(3,0);
	GUI_runStatusDisp(0);

	EXO_sigOFF(1);
	EXO_sigOFF(2);
	EXO_sigOFF(3);
	EXO_sigOFF(4);

	Clear();

//    USART1_sendBuf(sendBuffer[1],8);
//	__disable_irq();
//
//	__set_FAULTMASK(1);
//	NVIC_SystemReset();
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
	USART1_sendBuf(sendBuffer[1],8);

    MC_motorStopAll();

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
			if(IBIO_getInput(4) == 1)
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


	USART1_sendBuf(sendBuffer[1],8);
	MC_motorStopAll();
	GUI_runStatusDisp(0);

	IBIO_setOutput(6, 1);
	GUI_setOutputSignalColorDisp(5,1);
	IBIO_setOutput(7, 1);
	GUI_setOutputSignalColorDisp(6,1);

	EXO_sigOFF(1);
	EXO_sigOFF(2);
	EXO_sigOFF(3);
	EXO_sigOFF(4);

	Clear();
}

/**
 * 平台有料
 * * @param -no- 1 初始化完成信号, 2 请求打印信号, 3 送标完成, 4 接标完成,
 *
 * *
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
		EXIO_setOutput(0,0);
		GUI_setOutputSignalColorDisp(8,0);
		USART1_sendBuf(sendBuffer[5],8);
		break;
	case 2:
		EXIO_setOutput(1,0);
		GUI_setOutputSignalColorDisp(9,0);
//		switch(systemPara.AlarmFlag)
//		{
//			case 1:sendBuffer[21][5] = 0x01;break;
//			case 2:sendBuffer[21][5] = 0x02;break;
//			case 3:sendBuffer[21][5] = 0x03;break;
//			case 4:sendBuffer[21][5] = 0x04;break;
//			case 5:sendBuffer[21][5] = 0x05;break;
//			case 6:sendBuffer[21][5] = 0x06;break;
//			case 7:sendBuffer[21][5] = 0x07;break;
//			case 8:sendBuffer[21][5] = 0x08;break;
//			case 9:sendBuffer[21][5] = 0x09;break;
//			case 10:sendBuffer[21][5] = 0x0a;break;
//			case 11:sendBuffer[21][5] = 0x0b;break;
//			case 12:sendBuffer[21][5] = 0x0c;break;
//			default:	break;
//		}
//		systemPara.AlarmFlag = 0;
		USART1_sendBuf(sendBuffer[19],8);
		break;
	case 3:
		EXIO_setOutput(2,0);
		GUI_setOutputSignalColorDisp(10,0);
		USART1_sendBuf(sendBuffer[11],8);
		break;
	case 4:
		EXIO_setOutput(3, 0);
		GUI_setOutputSignalColorDisp(11,0);
		USART1_sendBuf(sendBuffer[9],8);
		break;
	}
}

/**
 * 关闭输出信号
 * * @param -no- 1 初始化, 2 请求打印信号, 3 送标完成, 4 接标完成,
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
		if(!EXIO_getOutput(0))
		{
			EXIO_setOutput(0,1);
			GUI_setOutputSignalColorDisp(8,1);
			USART1_sendBuf(sendBuffer[4],8);
		}
		break;
	case 2:
		if(!EXIO_getOutput(1))
		{
			EXIO_setOutput(1,1);
			GUI_setOutputSignalColorDisp(9,1);
			USART1_sendBuf(sendBuffer[18],8);
		}
		break;
	case 3:
		if(!EXIO_getOutput(2))
		{
			EXIO_setOutput(2, 1);
			GUI_setOutputSignalColorDisp(10,1);
			USART1_sendBuf(sendBuffer[10],8);
		}
		break;
	case 4:
		if(!EXIO_getOutput(3))
		{
			EXIO_setOutput(3, 1);
			GUI_setOutputSignalColorDisp(11,1);
			USART1_sendBuf(sendBuffer[8],8);
		}
		break;

	}
}

/**
 *
 * * @param -
 *
 **/
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
	return 0;
}

/**
 * @param -pin- 1 送料信号, 2 裁切信号, 3 初始化信号, 4 停止信号,
 *              5 放料启动, 6 放料停止, 7 备用, 8 备用
 */
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
	return 0;
}


/**
 * 真空控制
 * Action:			1-关闭 0-打开
 * En_outtime:		1-检测传感，使用超时检测，0不使用传感器
 */
uint8_t Vacuum_Action(uint8_t Action, uint8_t En_outtime)
{
	if(Action == 1)
	{
		if(systemPara.Language == 0)
			GUI_mainMessageDisp("关闭真空.", 9);
		else
			GUI_mainMessageDisp("Close the vacuum.", 17);
		IBIO_setOutput(1, 1);
		GUI_setOutputSignalColorDisp(0,1);
//		if(En_outtime)
//		{
//			if(GetIOLevel(4, 1, 2000) == 0)
//			{
//				if(systemPara.Language == 0)
//					GUI_mainMessageDisp("报警信息：关闭真空异常！", 32);
//				else
//					GUI_mainMessageDisp("Alarm:close the vacuum exception!", 3);
//				systemPara.AlarmFlag = 6;
//
//				Alarm(0);
//				return 0;
//			}
//		}
//		else
		{
			delay_ms(20);
		}

	}
	else
	{
		if(systemPara.Language == 0)
			GUI_mainMessageDisp("打开真空.", 20);
		else
			GUI_mainMessageDisp("Open the vacuum.", 16);
		IBIO_setOutput(1, 0);
		GUI_setOutputSignalColorDisp(0,0);
//		if(En_outtime)
//		{
//			if(GetIOLevel(4, 0, 2000) == 0)
//			{
//				if(systemPara.Language == 0)
//					GUI_mainMessageDisp("报警信息：打开真空异常！", 32);
//				else
//					GUI_mainMessageDisp("Alarm:Open the vacuum exception!", 32);
//				systemPara.AlarmFlag = 6;
//				Alarm(0);
//				return 0;
//			}
//		}
//		else
		{
			delay_ms(20);
		}
	}
	return 1;
}


void Blow_Action(uint8_t Action)
{
	if(Action == 1)
	{
		if(systemPara.Language == 0)
			GUI_mainMessageDisp("关闭吹气.", 9);
		else
			GUI_mainMessageDisp("Blow closing.", 13);
		IBIO_setOutput(2, 1);
		GUI_setOutputSignalColorDisp(1,1);
	}
	else
	{
		if(systemPara.Language == 0)
			GUI_mainMessageDisp("打开吹气.", 9);
		else
			GUI_mainMessageDisp("Blow opening.", 13);
		IBIO_setOutput(2, 0);
		GUI_setOutputSignalColorDisp(1,0);
	}
}

/**
 * 气缸控制
 * Action:			1-缩回 0-伸出
 * En_outtime:		1-检测传感，使用超时检测，0不使用传感器
 */
uint8_t Cylinder_Action(uint8_t Action, uint8_t En_outtime)
{
	if(Action == 1)
	{
		if(systemPara.Language == 0)
			GUI_mainMessageDisp("缩回气缸", 8);
		else
			GUI_mainMessageDisp("Retracted cylinder", 18);
		IBIO_setOutput(3, 0);
		IBIO_setOutput(4, 1);
		GUI_setOutputSignalColorDisp(2,0);
		GUI_setOutputSignalColorDisp(3,1);
		if(En_outtime)
		{
			if(GetIOLevel(2, 0, 2000) == 0)
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("报警信息：气缸缩回异常", 22);
				else
					GUI_mainMessageDisp("Alarm:cylinder retracted alarm", 30);
				systemPara.AlarmFlag = 6;
				Alarm(0);
				return 0;
			}
		}
		else
		{
			delay_ms(20);
		}

	}
	else
	{
		if(systemPara.Language == 0)
			GUI_mainMessageDisp("伸出气缸", 8);
		else
			GUI_mainMessageDisp("Extend cylinder", 15);
		IBIO_setOutput(3, 1);
		IBIO_setOutput(4, 0);
		GUI_setOutputSignalColorDisp(2,1);
		GUI_setOutputSignalColorDisp(3,0);
		if(En_outtime)
		{
			if(GetIOLevel(3, 0, 2000) == 0)
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("报警信息：气缸伸出异常", 22);
				else
					GUI_mainMessageDisp("Alarm:cylinder extend alarm", 27);
				systemPara.AlarmFlag = 6;
				Alarm(0);
				return 0;
			}
		}
		else
		{
			delay_ms(20);
		}
	}
	return 1;
}



void switchMode(uint8_t mode)
{
	if(mode == 1)
	{
		GUI_switchPage(32);

	    systemPara.status = STATUS_ONLINE;
	    AT24CXX_WriteOneByte(STORAGE_WORKMODE_ADDR, systemPara.status);
	    GUI_switchModeDisp(systemPara.status);

	    if(systemPara.Initstatus == 1)
	    {
			EXO_sigON(1);
			if(IBIO_getInput(4) == 0 && systemPara.doTakingOK == 1)
				EXO_sigON(4);
	    }
	    else
	    {
	    	systemPara.Initstatus = 0;
	    	GUI_runStatusDisp(0);
			if(systemPara.Language == 0)
				MAIN_TIPS_INNI_FIRST();
			else
				EN_MAIN_TIPS_INNI_FIRST();
	    }

		GUI_switchPage(32);
		USART1_sendBuf(sendBuffer[22],8);
		if(systemPara.Language == 0)
			MAIN_INFO_SW2ONLINE();
		else
			EN_MAIN_INFO_SW2ONLINE();
	}
	else if(mode == 0)
	{
	    if(systemPara.logginStatus < 10)
		{//未登录
			GUI_switchPage(32);
		}
		else
		{//已登录
			GUI_switchPage(0);
		}
	    //	调试模式关闭交互信号
		EXO_sigOFF(1);
		EXO_sigOFF(2);
		EXO_sigOFF(3);
		EXO_sigOFF(4);
		USART1_sendBuf(sendBuffer[23],8);
		if(systemPara.Language == 0)
			MAIN_INFO_SW2DEBUG();
		else
			EN_MAIN_INFO_SW2DEBUG();
	    systemPara.status = STATUS_AUTO;
	    AT24CXX_WriteOneByte(STORAGE_WORKMODE_ADDR, systemPara.status);
	    GUI_switchModeDisp(systemPara.status);
	}
}

void Alarm(void *pv)
{
//	EXO_sigON(2);

	EXO_sigOFF(1);
	EXO_sigOFF(2);
	EXO_sigOFF(3);
	EXO_sigOFF(4);

	IBIO_setOutput(1, 1);
	GUI_setOutputSignalColorDisp(0,1);
	IBIO_setOutput(2, 1);
	GUI_setOutputSignalColorDisp(1,1);

	MC_motorStopAll();
	GUI_runStatusDisp(0);
	Clear();
}
void Clear(void)
{
	systemPara.isWorkTaskRun = 0;
	systemPara.RunStatus = 0;
	systemPara.doTaking = 0;
	systemPara.doTakingOK = 0;
	systemPara.doLabeling = 0;
	systemPara.doLabelingOK = 0;
	systemPara.doLabelPosi = 0;
	systemPara.doLabelposiOK = 0;
	systemPara.doTakePosi = 0;
	systemPara.doTakePosiOK = 0;
	systemPara.doInit = 0;
	systemPara.Initstatus = STATUS_UNINIT;
	systemPara.doGohome = 0;
	systemPara.doGohomeOK = 0;
	systemPara.Err = 1;
	systemPara.LackMaterral = 1;
	systemPara.STOPOnceTriggerByUIorIO = 0;

	KP_step = 0;
	Init_step = 0;
	ms_count = 0;
	Gohome_step = 0;
	Gohome_step = 0;
	Labeling_step = 0;
	Taking_step = 0;
	GoToLableposi_step = 0;
	GoTotakeposi_step = 0;
	USLflag = 0;

}

void MdriveALM(void)
{
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("报警信息：送料驱动器报警.", 25);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("Alarm info: Feeding driver alarm.", 33);

	LV8731V_cmd(GIVEN_MOTOR, 0);
	Alarm(0);
}

uint8_t EmergencyStop(void)
{
	if(systemPara.isInEmergencyStopEnable)
	{
		if(IBIO_getInput(4) == 1 )
		{
			if(systemPara.Language == 0)
				POP_UP_INFO_RES_E_SOTP();
			else if(systemPara.Language == 1)
				EN_POP_UP_INFO_RES_E_SOTP();
			return 1;
		}
	}
	return 0;
}
