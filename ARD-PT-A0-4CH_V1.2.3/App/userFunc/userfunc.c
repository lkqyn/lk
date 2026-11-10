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
#include "easyModbus/easyModbus.h"
#include "storage/storage.h"
#include "at24cxx/at24cxx.h"
#include "MdriverAlarmIO/mdriveralarmio.h"

extern void selectAutoMode(void *pv);
extern void selectManualMode(void *pv);
extern void selectFreeRunMode(void *pv);

uint64_t songliao_count = 0; // 送料计数

uint16_t KP_step = 0;  // 空跑步骤
uint16_t OLM_step = 0;// 联机模式
uint16_t Gohome_step = 0;
uint16_t Pick_step = 0;
uint16_t PickRst_step = 0;
uint16_t ReceiLabel_step = 0;
uint16_t ReceiLabelRst_step = 0;
uint16_t Stick_step = 0;
uint16_t Init_step = 0;
uint16_t ms_count = 0;
uint16_t GQ_count = 0;

uint16_t Pick_Position_step = 0;
uint16_t Pick_Position_count = 0;
uint16_t Stick_Position_step = 0;
uint16_t Stick_Position_count = 0;

uint16_t Fetch_count = 0;
uint16_t Label_count = 0;
uint16_t Label_Position = 0;
uint16_t Fetch_time = 0;

uint32_t Fetch_distance = 0;
uint32_t Label_distance = 0;
uint32_t X_position = 0;

uint32_t offsetSpeed[10]={78, 75, 72, 70, 68, 65, 60, 55, 52,50};//初始化补偿,
//						  0-10 1-20 2-40 3-60 4-80 5-100 6-120 7-140

uint32_t offsetSpeed1[20]={78, 75, 72, 70, 68, 65, 60, 55, 52,50,
							48, 45,   43,   41,   39,   38,   36,  35,  34,  32};//夹料复位补偿


volatile uint8_t QQ_speed_h, QQ_speed_l;
volatile uint8_t QQ1_speed_h, QQ1_speed_l;
volatile uint8_t QQ2_speed_h, QQ2_speed_l;

uint8_t replacenment_flag = 0;//换料标志

uint16_t recvOffsetPulseCount = 0;//偏移距离
uint16_t recvDistance = 0; //送料长度
uint16_t recvBLDistance = 0; //换料后撤行程
uint16_t recvDistance1 = 0; //退料长度
uint8_t recvOffsetdir = 1; //偏移方向，默认正向

uint32_t recvOffsetPulseCount1 = 0, recvOffsetPulseCount2 = 0; //接收补偿暂存

/**
 @brief 控制输出信号
 */
void controlOutputSignal(void *pv)
{
	static uint8_t flag[] =
	{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

	if(systemPara.isInEmergencyStopEnable)
	{
		if(IBIO_getInput(12) == 1 )
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
			IBIO_setOutput(*((uint8_t*)pv), 0);
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
			IBIO_setOutput(*((uint8_t*)pv), 1);
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
	if(EmergencyStop() == 1)
		return;

	systemPara.doGohome = 1;
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

	switch(Gohome_step)
	{
		case 0://等待复位信号
			if(!systemPara.doGohome)
			{
				systemPara.doGohome = 0;
				break;
			}
			if(systemPara.RunStatus != 1 && systemPara.status != STATUS_FREERUN)//不是初始化和空跑模式
				systemPara.RunStatus = 7;
			Gohome_step ++;
			break;
		case 1:

			Gohome_step ++;
			break;
		case 2:
			if(IBIO_getInput(1) == 0)
			{

				if(systemPara.Language == 0)
					GUI_mainMessageDisp("避开原点...", 11);
				else if(systemPara.Language == 1)
					GUI_mainMessageDispIsolate("Avoid the origin...", 19);
				// 暂存剥料电机速度
				QQ_speed_l = ADDR_5000_L[4];
				QQ_speed_h = ADDR_5000_H[4];
				// 将回原点速度替换剥料电机速度
				ADDR_5000_L[4] = ADDR_5000_L[8];
				ADDR_5000_H[4] = ADDR_5000_H[8];
				systemPara.RSTmode = 2;//传感模式
				MC_motorRunAndOffsetByEXTI(BO_MOTOR, systemPara.MotorBoDir, 100, 2500);
				//电机速度还原
				ADDR_5000_L[4] = QQ_speed_l;
				ADDR_5000_H[4] = QQ_speed_h;
				QQ_speed_l = 0;
				QQ_speed_h = 0;
			}
			else
			{
				offsetPulseCount = MC_mm2pulse(BO_MOTOR,((ADDR_5000_H[6] << 8) | ADDR_5000_L[6]));
				goHomeSpeed = ((ADDR_5000_H[8] << 8) | ADDR_5000_L[8]) * 0.001f;
				if(goHomeSpeed <= 0) goHomeSpeed = 1;
				distance = offsetPulseCount + MC_mm2pulse(BO_MOTOR,offsetSpeed1[goHomeSpeed - 1]);

				if(systemPara.Language == 0)
					GUI_mainMessageDisp("检测原点...", 11);
				else if(systemPara.Language == 1)
					GUI_mainMessageDispIsolate("Back to origin.", 15);
				// 暂存剥料电机速度
				QQ_speed_l = ADDR_5000_L[4];
				QQ_speed_h = ADDR_5000_H[4];
				// 将回原点速度替换剥料电机速度
				ADDR_5000_L[4] = ADDR_5000_L[8];
				ADDR_5000_H[4] = ADDR_5000_H[8];
				systemPara.RSTmode = 1;//传感模式
				MC_motorRunAndOffsetByEXTI(BO_MOTOR, !systemPara.MotorBoDir, distance, RSTlimitPulseCount);
				//电机速度还原
				ADDR_5000_L[4] = QQ_speed_l;
				ADDR_5000_H[4] = QQ_speed_h;
				QQ_speed_l = 0;
				QQ_speed_h = 0;
			}
			Gohome_step  ++;
			break;
		case 3:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				if(IBIO_getInput(1) == 1 && systemPara.RSTmode == 1)
				{
					systemPara.RSTmode = 0;

					if(systemPara.Language == 0)
						ERR_INFO_GO_ORG_TIMEOUT();
					else if(systemPara.Language == 1)
						EN_ERR_INFO_GO_ORG_TIMEOUT();
					systemPara.AlarmFlag = 4;
					Alarm(0);
					return 0;
				}
				else if(IBIO_getInput(1) == 0 && systemPara.RSTmode == 2)
				{
					systemPara.RSTmode = 0;

					if(systemPara.Language == 0)
						ERR_INFO_AVOID_ORG_TIMEOUT();
					else if(systemPara.Language == 1)
						EN_ERR_INFO_AVOID_ORG_TIMEOUT();
					systemPara.AlarmFlag = 3;
					Alarm(0);
					return 0;
				}
				else if(IBIO_getInput(1))
				{
					Gohome_step = 2;
				}
				else
				{
					Gohome_step ++;
				}
			}
			break;
		case 4:

			Gohome_step ++;
			break;
		case 5:
			systemPara.doGohome = 0;

			if(systemPara.Language == 0)
				GUI_mainMessageDisp("回原点完成.", 11);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("Back to origin complete.", 24);
			if(systemPara.RunStatus != 1 && systemPara.status != STATUS_FREERUN)//不是初始化和空跑模式
				systemPara.RunStatus = 0;

			Pick_Position_count = 0;
			Stick_Position_count = 0;
			Gohome_step = 0;
			return 1;
			break;
	}
}

/**
 @brief 接标动作
 */
void ReceivingLabel(void *pv)
{

	switch(ReceiLabel_step)
	{
		case 0:
			if (!systemPara.ReceivingOnceTriggerByUIorIO)
			{
				return;
			}

			if(IBIO_getInput(3) == 0)
			{
				delay_ms(5);
				if(IBIO_getInput(3) == 0)
				{
					if(systemPara.Language == 0)
						GUI_mainMessageDisp("检测到标签，请确认是否有标签！", 30);
					else if(systemPara.Language == 1)
						GUI_mainMessageDisp("Label detected, please check if there’s a label!", 48);

					systemPara.ReceivingOnceTriggerByUIorIO = 0;
					return;
				}
			}

			systemPara.ReceivingOnceOK = 0;
			systemPara.RunStatus = 11;
			ReceiLabel_step ++;
			break;
		case 1:
			ReceiLabel_step ++;
			break;
		case 2:
			if(YL_Action(1) == 0)
				return;
			ReceiLabel_step ++;
			break;
		case 3:
			if(YLPY_Action(1) == 0)
				return;
			ReceiLabel_step ++;
			break;
		case 4:
			if(PT_Action(0) == 0)
				return;
			ReceiLabel_step ++;
			break;
		case 5:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("接标结束.", 9);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("Receiving lable completed.", 26);

			systemPara.RunStatus = 0;
			systemPara.ReceivRSTOnceOK = 0;
			systemPara.ReceivingOnceOK = 1;
			systemPara.ReceivingOnceTriggerByUIorIO = 0;
			ReceiLabel_step = 0;
			break;
	}
}


/**
 @brief 接标复位动作
 */
void ReceivingLabel_Rst(void *pv)
{

	switch(ReceiLabelRst_step)
	{
		case 0:
			if (!systemPara.ReceivRSTOnceTriggerByUIorIO)
			{
				return;
			}

//			if(IBIO_getInput(3) == 1)
//			{
//				delay_ms(5);
//				if(IBIO_getInput(3) == 1)
//				{
//					if(systemPara.Language == 0)
//						GUI_mainMessageDisp("未检测到标签，请先打印！", 23);
//					else if(systemPara.Language == 1)
//						GUI_mainMessageDisp("No label detected, please print first!", 38);
//
//					systemPara.ReceivRSTOnceTriggerByUIorIO = 0;
//					return;
//				}
//			}

			systemPara.ReceivRSTOnceOK = 0;
			systemPara.RunStatus = 10;
			ReceiLabelRst_step ++;
			break;
		case 1:

			ReceiLabelRst_step ++;
			break;
		case 2:
			if(YL_Action(0) == 0)
				return;
			ReceiLabelRst_step ++;
			break;
		case 3:
			if(PT_Action(1) == 0)
				return;
			ReceiLabelRst_step ++;
			break;
		case 4:
			if(YL_Action(1) == 0)
				return;
			ReceiLabelRst_step ++;
			break;
		case 5:
			if(YLPY_Action(0) == 0)
				return;
			ReceiLabelRst_step ++;
			break;
		case 6:
			if(YL_Action(0) == 0)
				return;
			ReceiLabelRst_step ++;
			break;
		case 7:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("接标复位结束.", 12);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("Receiving lable reset completed.", 32);

			systemPara.RunStatus = 0;
			systemPara.ReceivRSTOnceOK = 1;
			systemPara.ReceivingOnceOK = 0;
			systemPara.ReceivRSTOnceTriggerByUIorIO = 0;
			ReceiLabelRst_step = 0;
			break;
	}
}


/**
 @brief 取标复位动作
 */
void PickLabel_Rst(void *pv)
{
	uint32_t offsetPulseCount;//原点补偿
	uint32_t distance;			//
	uint32_t RunSpeed;

	switch(PickRst_step)
	{
		case 0:
			if (!systemPara.RSTOnceTriggerByUIorIO)
			{
				return;
			}

			if(IBIO_getInput(3) == 0)
			{
				if(systemPara.Language == 0)
				    GUI_mainMessageDisp("物料取尽，光纤检测异常.", 23);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Material exhausted, abnormal optical testing.", 45);

				systemPara.pickOnceTriggerByUIorIO = 0;
				Alarm(0);
				return;
			}

			systemPara.RSTOnceOK = 0;
			systemPara.RunStatus = 8;
			PickRst_step = 2;
			break;
		case 2:
			if(IBIO_getOutput(5) == 0 || IBIO_getInput(4) == 0)		//关闭真空
			{
				if(ZK_Action(1) == 0)
					return;
			}
			PickRst_step ++;
			break;
		case 3:
			if(IBIO_getOutput(2) == 0 || IBIO_getInput(7) != 0)		//缩回贴标气缸
			{
				if(TB_Action(1) == 0)
					return;
			}
			PickRst_step ++;
			break;
		case 4:
			if(IBIO_getOutput(1) == 0 || IBIO_getInput(5) != 0)		//缩回升降气缸
			{
				if(SJ_Action(1) == 0)
					return;
			}
			PickRst_step ++;
			break;
		case 5:
			if(IBIO_getOutput(3) == 0 || IBIO_getInput(9) != 0)		//旋转气缸回原
			{
				if(XZ_Action(1) == 0)
					return;
			}
			PickRst_step ++;
			break;
		case 6:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("X轴复位开始...", 14);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("X-axis reset starts...", 22);

			offsetPulseCount = MC_mm2pulse(BO_MOTOR,((ADDR_5000_H[6] << 8) | ADDR_5000_L[6]));
			RunSpeed = ((ADDR_5000_H[21] << 8) | ADDR_5000_L[21]) * 0.001f;//读取运行速度
			if(RunSpeed <= 0) RunSpeed = 1;
			distance = offsetPulseCount + MC_mm2pulse(BO_MOTOR,offsetSpeed1[RunSpeed - 1]);

			// 暂存剥料电机速度
			QQ_speed_l = ADDR_5000_L[4];
			QQ_speed_h = ADDR_5000_H[4];

			ADDR_5000_L[4] = ADDR_5000_L[21];
			ADDR_5000_H[4] = ADDR_5000_H[21];
			systemPara.RSTmode = 1;//传感模式
			MC_motorRunAndOffsetByEXTI(BO_MOTOR, !systemPara.MotorBoDir, distance,RSTlimitPulseCount);
			ADDR_5000_L[4] = QQ_speed_l;
			ADDR_5000_H[4] = QQ_speed_h;
			QQ_speed_l = 0;
			QQ_speed_h = 0;
			PickRst_step ++;
			break;
		case 7:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				PickRst_step ++;
			}
			break;
		case 8:
			if (motor_ch[BO_MOTOR].stopEvent == EVENT_exceedPulseCountLimit)
			{
				if(systemPara.Language == 0)
					ERR_INFO_GO_ORG_TIMEOUT();
				else if(systemPara.Language == 1)
					EN_ERR_INFO_GO_ORG_TIMEOUT();

				systemPara.AlarmFlag = 4;
				Alarm(0);
				return;
			}
			PickRst_step ++;
			break;
		case 9:
//			if((EXIO_getOutput(6) == 1 || EXIO_getInput(6) != 0))	//压料缩回
			{
				if(YL_Action(1) == 0)
					return;
			}
			PickRst_step ++;
			break;
		case 10:
//			if((EXIO_getOutput(4) == 1 || EXIO_getInput(4) != 0))	//压料平移缩回
			{

				if(YLPY_Action(1) == 0)
					return;
			}
			PickRst_step ++;
			break;

		case 11:
//			if(IBIO_getOutput(4) == 1 || IBIO_getInput(12) != 0)	//平台伸出
			{
				if(PT_Action(0) == 0)
					return;
			}
			PickRst_step ++;
			break;
		case 12:
//			if((EXIO_getOutput(6) == 1 || EXIO_getInput(6) != 0))	//压料缩回
			{
				if(YL_Action(1) == 0)
					return;
			}
			PickRst_step ++;
			break;
		case 13:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("X轴复位结束.", 12);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("X-aixs reset completed.", 23);

			X_position = 0;

			systemPara.RunStatus = 0;
			systemPara.RSTOnceOK = 1;
			systemPara.stickOnceOK1 = 1;
			systemPara.RSTOnceTriggerByUIorIO = 0;
			PickRst_step = 0;
			break;
	}
}


/**
 @brief 取标动作
 */
void PickLabel(void *pv)
{
	uint32_t distance,distance1;
	distance = ((ADDR_5000_H[1] << 8) | ADDR_5000_L[1]);
	distance1 = ((ADDR_5000_H[7] << 8) | ADDR_5000_L[7]);

	switch(Pick_step)
	{
		case 0:
			if(systemPara.stickOnceOK1 ==  1 && IBIO_getInput(12) == 0 && IBIO_getInput(3) == 0)
			{
				delay_ms(1);
				if(++GQ_count >= 350 && IBIO_getInput(3) == 0)
				{
					GQ_count = 0;
					if(systemPara.pickOnceTriggerByUIorIO == 0)
					{
						systemPara.pickOnceTriggerByUIorIO = 1;
					}
				}
			}
			else
			{
				GQ_count = 0;
			}

			if(!systemPara.pickOnceTriggerByUIorIO || systemPara.RunStatus != 0)
			{
				systemPara.pickOnceTriggerByUIorIO = 0;
				return;
			}

			if(EXIO_getInput(3) == 0)
			{
				systemPara.pickOnceTriggerByUIorIO = 0;
				return;
			}

			if(IBIO_getInput(4) == 0)
			{
				delay_ms(5);
				if(IBIO_getInput(4) == 0)
				{
					if(systemPara.Language == 0)
						GUI_mainMessageDisp("提示信息：吸头检测到料！", 24);
					else if(systemPara.Language == 1)
						GUI_mainMessageDisp("Notification: The suction head has material.", 44);
					systemPara.pickOnceTriggerByUIorIO = 0;
					return;
				}
			}

			if(IBIO_getInput(3) != 0 && Fetch_count == 0 && systemPara.status == STATUS_ONLINE)
			{
				if(systemPara.Language == 0)
				    GUI_mainMessageDisp("未检测到料.", 11);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("No material detected.", 21);

				systemPara.pickOnceTriggerByUIorIO = 0;
				systemPara.AlarmFlag = 4;
				Alarm(0);
				return;
			}

			EXO_sigOFF(2);
			EXO_sigOFF(4);

			systemPara.stickOnceOK1 = 0;

			systemPara.RunStatus = 5;
			Pick_step ++;
			break;
		case 1:
			IBIO_setOutput(5, 0);
			GUI_setOutputSignalColorDisp(4,0);
			Pick_step = 50;
			break;
		case 50:
			if(systemPara.ReceivRSTOnceOK == 0)
			{
				systemPara.ReceivRSTOnceTriggerByUIorIO = 1;
				Pick_step ++;
			}
			else
			{
				Pick_step ++;
			}

			break;
		case 51:
			if(systemPara.ReceivRSTOnceOK == 1)
			{
				Pick_step  = 2;
			}
			break;
		case 2:
			delay_ms(1);
			if(++ms_count > 150)
			{
				ms_count = 0;
				if(IBIO_getInput(4) == 1)
				{
					Pick_step ++;
				}
				else
				{
					Pick_step = 16;
				}
			}
			break;
		case 3:
			if(IBIO_getOutput(4) == 0 || IBIO_getInput(11) != 0)	//平台缩回
			{
				if(PT_Action(1) == 0)
					return;
			}
			Pick_step ++;
			break;
		case 4:
			if(IBIO_getOutput(5) == 0 || IBIO_getInput(4) == 0)		//关闭真空
			{
				if(ZK_Action(1) == 0)
					return;
			}
			Pick_step ++;
			break;
		case 5:
			if(IBIO_getOutput(2) == 0 || IBIO_getInput(7) != 0)		//缩回贴标气缸
			{
				if(TB_Action(1) == 0)
					return;
			}
			Pick_step ++;
			break;
		case 6:
			if(IBIO_getOutput(1) == 0 || IBIO_getInput(5) != 0)		//缩回升降气缸
			{
				if(SJ_Action(1) == 0)
					return;
			}
			Pick_step ++;
			break;
		case 7:
			if(IBIO_getOutput(3) == 0 || IBIO_getInput(9) != 0)		//旋转气缸回原
			{
				if(XZ_Action(1) == 0)
					return;
			}
			Pick_step ++;
			break;
		case 8:
			if(systemPara.Language == 0)
			    GUI_mainMessageDisp("去取标位.", 9);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Go to the fetch position", 24);
			Pick_step ++;
			break;
		case 9:
			if(Fetch_count < ((ADDR_5000_H[0] << 8) | ADDR_5000_L[0]) && IBIO_getInput(3) == 0)
			{
				Fetch_distance = distance1+ (Fetch_count * distance);
				Pick_step ++;
			}
			else
			{
				if(IBIO_getOutput(4) == 1 || IBIO_getInput(12) != 0)	//平台伸出
				{
					systemPara.ReceivRSTOnceOK = 0;
					YL_Action(1);
					YLPY_Action(1);
					if(PT_Action(0) == 0)
//					systemPara.ReceivingOnceTriggerByUIorIO == 1;
						return;
				}

				systemPara.RSTOnceTriggerByUIorIO = 1;
				systemPara.RSTOnceOK = 0;

				if(IBIO_getInput(3) == 0)
				{
					if(Fetch_count >= ((ADDR_5000_H[0] << 8) | ADDR_5000_L[0]))
					{
						if(systemPara.Language == 0)
							GUI_mainMessageDisp("物料取尽，光纤检测异常.", 23);
						else if(systemPara.Language == 1)
							GUI_mainMessageDisp("Material exhausted, abnormal optical testing.", 45);

						systemPara.AlarmFlag = 3;

						systemPara.stickOnceTriggerByUIorIO = 0;
						systemPara.stickOnceOK = 0;
						systemPara.stickOnceOK1 = 1;
						Stick_step = 0;

						Alarm(0);
					}
				}
				else
				{
					USART1_sendBuf(sendBuffer[11],8);
				}

				systemPara.pickOnceTriggerByUIorIO = 0;
				systemPara.pickOnceOK = 0;
				systemPara.RunStatus = 0;
				X_position = 0;
				Fetch_count = 0;
				Pick_step = 0;
				return;
			}
			break;
		case 10:
			if(Fetch_distance > X_position)
			{
				MC_motorMoveDistance(BO_MOTOR, systemPara.MotorBoDir, (Fetch_distance - X_position));

			}
			else if(Fetch_distance < X_position)
			{
				MC_motorMoveDistance(BO_MOTOR, !systemPara.MotorBoDir, (X_position - Fetch_distance));
			}
			Pick_step ++;
			break;
		case 11:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				X_position = distance1+ (Fetch_count * distance);
				Fetch_count++;
				Pick_step ++;
			}
			break;
//		case 40:
//			YLPY_Action(0);
//			YL_Action(0);
//			Pick_step = 41;
//			break;
//		case 41:
//			delay_ms(10);
//			if(++ms_count > 50)
//			{
//				ms_count = 0;
//				Pick_step = 12;
//			}
//			break;
		case 12:
			if(IBIO_getOutput(1) == 1 || IBIO_getInput(6) != 0)		//伸出升降气缸
			{
				if(SJ_Action(0) == 0)
					return;
			}
			Pick_step ++;
			break;
		case 13:
			if(systemPara.Language == 0)
				MAIN_INFO_ZK_OPEN();
			else if(systemPara.Language == 1)
				EN_MAIN_INFO_ZK_OPEN();

			IBIO_setOutput(5, 0);
			GUI_setOutputSignalColorDisp(4,0);			//打开真空

			Pick_step ++;
			break;
		case 14:
			delay_ms(1);
			if(++ ms_count >= 100)
			{
				ms_count = 0;
				Pick_step ++;
			}
			break;
		case 15:
			if(IBIO_getInput(4) == 0)
			{
				delay_ms(5);
				if(IBIO_getInput(4) == 0)
				{
					Pick_step ++;
				}
			}
			else
			{
				delay_ms(1);
				if(++ ms_count >= 1000)
				{
					if(++ Fetch_time >= 2)
					{
						Fetch_time = 0;
						ms_count = 0;
						Pick_step = 3;
					}
					else
					{
						if(IBIO_getOutput(1) == 0 || IBIO_getInput(5) != 0)		//缩回升降气缸
						{
							if(SJ_Action(1) == 0)
								return;
						}

						IBIO_setOutput(5, 1);
						GUI_setOutputSignalColorDisp(4,1);			//关闭真空
						Pick_step = 12;
					}
				}
			}
			break;
		case 16:
			if(IBIO_getOutput(1) == 0 || IBIO_getInput(5) != 0)		//缩回升降气缸
			{
				if(SJ_Action(1) == 0)
					return;
			}
			Pick_step ++;
			break;
		case 17:
			delay_ms(1);
			if(++ ms_count >= 100)
			{
				ms_count = 0;
				Pick_step ++;
			}
			break;
		case 18:
			if(IBIO_getInput(4) == 0)
			{
				delay_ms(5);
				if(IBIO_getInput(4) == 0)
				{
					ms_count = 0;
					Pick_step ++;
				}
			}
			else
			{
				delay_ms(1);
				if(++ ms_count >= 1000)
				{
					if(++ Fetch_time >= 2)
					{
						Fetch_time = 0;
						ms_count = 0;
						Pick_step = 3;
					}
					else
					{
						IBIO_setOutput(5, 1);
						GUI_setOutputSignalColorDisp(4,1);			//关闭真空
						Pick_step = 12;
					}
				}
			}
			break;
		case 19:
			if(IBIO_getOutput(3) == 1 || IBIO_getInput(10) != 0)		//旋转气缸动作
			{
				if(XZ_Action(0) == 0)
					return;
			}
			Pick_step ++;
			break;
		case 20:
			if(systemPara.Language == 0)
			    GUI_mainMessageDisp("取标完成.", 9);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Fetching action is complete.", 28);

			if(systemPara.status == STATUS_ONLINE)
			{
				EXO_sigON(2);
			}

			systemPara.pickOnceTriggerByUIorIO = 0;
			systemPara.PrintStatus = 0;
			systemPara.pickOnceOK = 1;
//			systemPara.stickOnceOK = 0;
			systemPara.stickOnceOK1 = 0;
			systemPara.RunStatus = 0;
			Pick_step = 0;

			delay_ms(200);

			if(IBIO_getInput(3) == 0)
			{
				if(Fetch_count >= ((ADDR_5000_H[0] << 8) | ADDR_5000_L[0]))
				{
					if(IBIO_getOutput(4) == 1 || IBIO_getInput(12) != 0)	//平台伸出
					{
						systemPara.ReceivRSTOnceOK = 0;
						YL_Action(1);
						YLPY_Action(1);
						if(PT_Action(0) == 0)
//						systemPara.ReceivingOnceTriggerByUIorIO == 1;
							return;
					}
					if(systemPara.Language == 0)
						GUI_mainMessageDisp("物料取尽，光纤检测异常.", 23);
					else if(systemPara.Language == 1)
						GUI_mainMessageDisp("Material exhausted, abnormal optical testing.", 45);

					systemPara.pickOnceTriggerByUIorIO = 0;
					systemPara.AlarmFlag = 3;

					systemPara.stickOnceTriggerByUIorIO = 0;
					systemPara.stickOnceOK = 0;
					systemPara.stickOnceOK1 = 1;
					Stick_step = 0;

					Alarm(0);
					return;
				}
			}
			else
			{
//				X_position = 0;
				Fetch_count = 0;			//qin0houdierlunquliaozouweizhi,buyinxiangjingdkehulue
//				Pick_step = 0;
				if(IBIO_getOutput(4) == 1 || IBIO_getInput(12) != 0)	//平台伸出
				{
//					systemPara.ReceivingOnceTriggerByUIorIO == 1;
					systemPara.ReceivRSTOnceOK = 0;
					YL_Action(1);
					YLPY_Action(1);
					if(PT_Action(0) == 0)

						return;
				}
//				USART1_sendBuf(sendBuffer[11],8);
			}

//			if(systemPara.pickOnceOK == 1 && systemPara.pickOnceTriggerByUIorIO == 0)
//			{
//				USART1_sendBuf(sendBuffer[7],8);
//			}
//			else
//			{
//				USART1_sendBuf(sendBuffer[6],8);
//			}

//			delay_ms(250);
			break;
	}
}


/**
 @brief 贴标动作
 */
void StickLabel(void *pv)
{
	char str[8]="0";
	uint16_t len;
	uint32_t distance, distance1;

	distance = ((ADDR_5000_H[18] << 8) | ADDR_5000_L[18]);		//贴标间距
	distance1 = ((ADDR_5000_H[2] << 8) | ADDR_5000_L[2]);		//贴标起始位

	switch(Stick_step)
	{

		case 0:
			if(!systemPara.stickOnceTriggerByUIorIO)
			{
				systemPara.stickOnceTriggerByUIorIO = 0;
				return;
			}

			if(EXIO_getInput(3) == 0)
			{
				systemPara.stickOnceTriggerByUIorIO = 0;
				return;
			}

//			if(systemPara.pickOnceOK != 1)
//			{
//				return;
//			}
			if(IBIO_getInput(3) == 1)
			{
//				X_position = 0;
				Fetch_count = 0;
//				Pick_step = 0;
				if(IBIO_getOutput(4) == 1 || IBIO_getInput(12) != 0)	//平台伸出
				{
					systemPara.ReceivRSTOnceOK = 0;
					YL_Action(1);
					YLPY_Action(1);
					if(PT_Action(0) == 0)
//					systemPara.ReceivingOnceTriggerByUIorIO == 1;
						return;
				}
				USART1_sendBuf(sendBuffer[11],8);
			}


			EXO_sigOFF(1);
			EXO_sigOFF(2);

			if(systemPara.stickOnceTriggerByUIorIO == 2)
			{
				if(Label_Position > 0)
				{
					Label_count = Label_Position - 1;
				}
				else
				{
					systemPara.stickOnceTriggerByUIorIO = 0;
					return;
				}
			}

			systemPara.RunStatus = 6;
			Stick_step ++;
			break;
		case 1:
			if(IBIO_getOutput(2) == 0 || IBIO_getInput(7) != 0)		//缩回贴标气缸
			{
				if(TB_Action(1) == 0)
					return;
			}

			IBIO_setOutput(5, 0);
			GUI_setOutputSignalColorDisp(4,0);		//打开真空复检
			Stick_step ++;
			break;
		case 2:
			if(IBIO_getOutput(1) == 0 || IBIO_getInput(5) != 0)		//缩回升降气缸
			{
				if(SJ_Action(1) == 0)
					return;
			}
			Stick_step ++;
			break;
		case 3:
			if(IBIO_getInput(4) == 0)
			{
				delay_ms(5);
				if(IBIO_getInput(4) == 0)
				{
					Stick_step += 2;
				}
			}
			else
			{
				if(IBIO_getInput(3) == 0)
				{
					systemPara.pickOnceTriggerByUIorIO = 1;
					systemPara.pickOnceOK = 0;
					systemPara.RunStatus = 0;
	//				Stick_step = 0;
					Stick_step ++;
				}
			}
			break;
		case 4:
			if(systemPara.pickOnceOK == 1)
			{
				if(IBIO_getInput(3) == 1)
				{
	//				X_position = 0;
					Fetch_count = 0;
	//				Pick_step = 0;
					if(IBIO_getOutput(4) == 1 || IBIO_getInput(12) != 0)	//平台伸出
					{
						systemPara.ReceivRSTOnceOK = 0;
						YL_Action(1);
						YLPY_Action(1);
						if(PT_Action(0) == 0)
//						systemPara.ReceivingOnceTriggerByUIorIO == 1;
							return;
					}
					USART1_sendBuf(sendBuffer[11],8);
				}
				Stick_step ++;
			}
			break;
		case 5:
			if(systemPara.Language == 0)
			    GUI_mainMessageDisp("去贴标位.", 9);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Go to the label position", 24);
			Stick_step ++;
			break;
		case 6:
			if(Label_count < ((ADDR_5000_H[5] << 8) | ADDR_5000_L[5]))
			{//计算当前贴位置
				Label_distance = distance1 + (Label_count * distance);
			}
			Stick_step ++;
			break;
		case 7:
			if(Label_distance > X_position)//目标位置大于当前位置
			{//当前位置与贴标位置比较，确定贴标位置和方向
				MC_motorMoveDistance(BO_MOTOR, systemPara.MotorBoDir, (Label_distance - X_position));
			}
			else if(Label_distance < X_position)
			{
				MC_motorMoveDistance(BO_MOTOR, !systemPara.MotorBoDir, (X_position - Label_distance));
			}
			Stick_step ++;
			break;
		case 8:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				X_position = distance1 + (Label_count * distance);
				Label_count ++;
				Stick_step ++;
			}
			break;
		case 9:
			if(IBIO_getOutput(3) == 1 || IBIO_getInput(10) != 0)		//旋转气缸动作
			{
				if(XZ_Action(0) == 0)
					return;
			}
			Stick_step ++;
			break;
		case 10:
			if(IBIO_getOutput(2) == 1 || IBIO_getInput(8) != 0)		//伸出贴标气缸
			{
				if(TB_Action(0) == 0)
					return;
			}
			Stick_step ++;
			break;
		case 11:
			if(IBIO_getOutput(5) == 0 || IBIO_getInput(4) == 0)		//关闭真空
			{
				if(ZK_Action(1) == 0)
					return;
			}
			Stick_step ++;
			break;
		case 12:
			delay_ms(1);
			if(++ ms_count >= 50)
			{
				ms_count = 0;
				Stick_step ++;
			}
			break;
		case 13:
			if(IBIO_getOutput(2) == 0 || IBIO_getInput(7) != 0)		//缩回贴标气缸
			{
				if(TB_Action(1) == 0)
					return;
			}
			Stick_step ++;
			break;
		case 14:
			IBIO_setOutput(5, 0);
			GUI_setOutputSignalColorDisp(4,0);		//打开真空复检
			Stick_step ++;
			break;
		case 15:
			delay_ms(1);
			if(++ ms_count >= 300)
			{
				ms_count = 0;
				Stick_step ++;
			}
			break;
		case 16:
			if(IBIO_getInput(4) == 1)
			{
				Stick_step ++;
			}
			else
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("报警信息：真空异常！", 30);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Alarm info:abnormal extension of sensor cylinder!", 49);
				systemPara.AlarmFlag = 5;
				Alarm(0);

				Label_count --;

				systemPara.stickOnceTriggerByUIorIO = 0;
				systemPara.stickOnceOK = 0;
				systemPara.stickOnceOK1 = 1;
				systemPara.pickOnceOK = 0;
				systemPara.RunStatus = 0;
				Stick_step = 0;
				return;
			}
			break;
		case 17:
			if(IBIO_getOutput(3) == 0 || IBIO_getInput(9) != 0)		//旋转气缸回原
			{
				if(XZ_Action(1) == 0)
					return;
			}

			songliao_count ++;
			len = sprintf(str,"%d",songliao_count);
			GUI_showText(0x8000, "        ", 8);
			GUI_showText(0x8000, str, len);
			Stick_step ++;
			break;
//		case 17:
////			if(Label_count < ((ADDR_5000_H[5] << 8) | ADDR_5000_L[5]) && systemPara.status == STATUS_ONLINE
////					&& systemPara.stickOnceTriggerByUIorIO == 1)
////			{
////				systemPara.pickOnceTriggerByUIorIO = 1;
////				systemPara.pickOnceOK = 0;
////				systemPara.RunStatus = 0;
////				Stick_step = 0;
////			}
////			else
//			{
//				Stick_step ++;
//			}
//			break;
		case 18:
			if(systemPara.Language == 0)
			    GUI_mainMessageDisp("贴标完成.", 9);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Labeling action is complete.", 28);

//			if(Label_count >= ((ADDR_5000_H[5] << 8) | ADDR_5000_L[5]))
			{
//				Label_count = 0;
				if(systemPara.status == STATUS_ONLINE)
				{
					EXO_sigON(1);
				}
			}
			Stick_step ++;
			break;
		case 19:
			if(EXIO_getInput(1) == 0 && systemPara.status == STATUS_ONLINE && systemPara.stickOnceTriggerByUIorIO == 1)
			{
				delay_ms(5);
				if(EXIO_getInput(1) == 0)
				{
					if(systemPara.Language == 0)
					    GUI_mainMessageDisp("等待断开贴标信号.", 17);
					else if(systemPara.Language == 1)
						GUI_mainMessageDisp("Wait for disconnect label signal.", 33);
					return;
				}
			}
			else
			{
				Stick_step ++;
			}
			break;
		case 20:
			delay_ms(1);
			if(++ ms_count >= 100)
			{
				ms_count = 0;
				Stick_step ++;
			}
			break;
		case 21:
			if(Label_count >= ((ADDR_5000_H[5] << 8) | ADDR_5000_L[5]))
			{
				Label_count = 0;
			}
			if(systemPara.stickOnceTriggerByUIorIO == 1)
			{
				EXO_sigOFF(1);
			}
			else
			{
//				Label_count = 0;
				EXO_sigON(1);
			}


			if(Fetch_count >= ((ADDR_5000_H[0] << 8) | ADDR_5000_L[0]) || IBIO_getInput(3) != 0)
			{
				systemPara.pickOnceTriggerByUIorIO = 0;
				systemPara.RSTOnceTriggerByUIorIO = 1;
				systemPara.RSTOnceOK = 0;
				systemPara.RunStatus = 0;
				X_position = 0;
				Fetch_count = 0;
				Pick_step = 0;
			}
			else
			{
				systemPara.stickOnceOK1 = 1;

				if(systemPara.status == STATUS_ONLINE)
				{
					systemPara.pickOnceTriggerByUIorIO = 1;
					systemPara.pickOnceOK = 0;
				}
			}

			systemPara.stickOnceTriggerByUIorIO = 0;
			systemPara.stickOnceOK = 1;
			systemPara.pickOnceOK = 0;
			systemPara.RunStatus = 0;
			Stick_step = 0;


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
				systemPara.doInit = 0;
				return;
			}

			MC_motorStopAll();

			//关闭使能，清除驱动器报警信号
			if(systemPara.AlarmFlag == 20)
			{
				LV8731V_cmd(GIVEN_MOTOR, 0);
				delay_ms(50);
				LV8731V_cmd(GIVEN_MOTOR, 1);
			}
			if(systemPara.AlarmFlag == 21)
			{
				LV8731V_cmd(BO_MOTOR, 0);
				delay_ms(50);
				LV8731V_cmd(BO_MOTOR, 1);
			}
			systemPara.AlarmFlag = 0;

			if(systemPara.isAutoLetMetalEnable)
				LV8731V_cmd(LET_MOTOR, 1);
			if(systemPara.isUpShouEnable)
				LV8731V_cmd(SHOU_MOTOR, 1);

			Clear();

			if(systemPara.Language == 0)
				GUI_mainMessageDisp("开始初始化.", 11);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Start initialization.", 21);
			systemPara.RunStatus = 1;
			systemPara.RSTOnceOK = 0;
			Init_step ++;
			break;
		case 1://清除交互信号

			if(systemPara.Language == 0)
				GUI_mainMessageDisp("清除交互信号.", 13);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("Clear interactive signal.", 25);
			EXO_sigOFF(1);
			EXO_sigOFF(2);
			EXO_sigOFF(3);
			EXO_sigOFF(4);
			EXO_sigOFF(5);
			if(replacenment_flag)
			{
				replacenment_flag = 0;
			}
			delay_ms(100);
			Init_step ++;
			break;
		case 2:
			if(IBIO_getInput(5) == 1)
			{
				if(SJ_Action(1) == 0)//缩回
					return;
			}
			Init_step ++;
			break;
		case 3:
			if(IBIO_getInput(7) == 1)
			{
				if(TB_Action(1) == 0)//缩回
					return;
			}
			Init_step ++;
			break;
		case 4:
			if(IBIO_getInput(9) == 1)
			{
				if(XZ_Action(1) == 0)//缩回
					return;
			}
			Init_step ++;
			break;
		case 5:
			if(YL_Action(1) == 0)//压料不在下面
				return;
			Init_step ++;
			break;
		case 6:
			if(YLPY_Action(1) == 0)//压料平移伸出
				return;
			Init_step ++;
			break;
		case 7:
			if((IBIO_getOutput(4) == 1 || IBIO_getInput(12) != 0) && IBIO_getInput(3) == 1)	//平台伸出
			{
				if(PT_Action(0) == 0)//平台伸出
					return;
			}
			Init_step ++;
			break;
		case 8:
			if(IBIO_getOutput(5) == 0 || IBIO_getInput(4) == 0)		//关闭真空
			{
				if(ZK_Action(1) == 0)//真空
					return;
			}
			Init_step ++;
			break;
		case 9://复位
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("复位中...", 9);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Back to origin...", 17);
			systemPara.doGohome = 1;
			Init_step ++;
			break;
		case 10://
			if(systemPara.doGohome != 0)
			{
				if(GoHome_step(0)== 0)
				{
					return;
				}
			}
			else
				Init_step ++;
			break;


		case 11:
			PT_Action(0);
			Init_step ++;
			break;
		case 12:
			if(ZK_Action(1) == 0)
				return;
			Init_step ++;
			break;
		case 13:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("初始化完成！", 11);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Initialization is complete!", 27);


			if(systemPara.status == STATUS_ONLINE)
			{
				EXO_sigON(3);
				if(IBIO_getInput(3) == 1)
				{
					EXO_sigON(4);
				}
				else
				{
					systemPara.pickOnceTriggerByUIorIO = 1;
					systemPara.pickOnceOK = 0;
				}
			}

			GUI_runStatusDisp(1);
			systemPara.doInit = 0;
			systemPara.pickOnceOK = 0;
			systemPara.stickOnceOK = 0;
			systemPara.stickOnceOK1 = 1;
			systemPara.RSTOnceOK = 1;
			systemPara.isWorkTaskRun = 1;
			systemPara.RunStatus = 0;
			systemPara.Initstatus = 1;

			if(systemPara.Initstatus == 0)
			{
				USART1_sendBuf(sendBuffer[2],8);
			}
			else
			{
				USART1_sendBuf(sendBuffer[3],8);
				delay_ms(100);
				if(IBIO_getInput(3) == 1)
				{
					USART1_sendBuf(sendBuffer[11],8);
				}
				else
				{
					systemPara.pickOnceTriggerByUIorIO = 1;
					systemPara.pickOnceOK = 0;
				}
			}

			Init_step = 0;
			break;
	}
}

void Pick_Position(void *pv)
{
	uint32_t distance;
	distance = ((ADDR_5000_H[1] << 8) | ADDR_5000_L[1]);

	switch(Pick_Position_step)
	{
		case 0:
			if(IBIO_getInput(1) != 0 && Pick_Position_count == 0)
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("请先复位.", 9);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Please reset first.", 19);
				return;
			}

			if(Pick_Position_count >= (((ADDR_5000_H[0] << 8) | ADDR_5000_L[0])))
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("请先复位.", 9);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Please reset first.", 19);
				return;
			}

			if(++ Pick_Position_count < (((ADDR_5000_H[0] << 8) | ADDR_5000_L[0])))
			{
				MC_motorMoveDistance(BO_MOTOR, systemPara.MotorBoDir, distance);
			}
			break;
	}
}

void Stick_Position(void *pv)
{
	uint32_t distance;
	distance = ((ADDR_5000_H[18] << 8) | ADDR_5000_L[18]);

	switch(Stick_Position_step)
	{
		case 0:
			if(IBIO_getInput(1) == 0 && Stick_Position_count == 0)
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("请先到贴标起始位.", 17);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Please go to the label start position first.", 44);
				return;
			}

			if(Stick_Position_count >= (((ADDR_5000_H[5] << 8) | ADDR_5000_L[5])))
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("请先复位.", 9);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Please reset first.", 19);
				return;
			}

			if(++ Stick_Position_count < (((ADDR_5000_H[5] << 8) | ADDR_5000_L[5])))
			{
				MC_motorMoveDistance(BO_MOTOR, systemPara.MotorBoDir, distance);
			}
			break;
	}
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
	    GUI_mainMessageDisp("UI紧急停止动作！", 16);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("UI E-stop action!", 17);

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
	if(systemPara.STOPOnceTriggerByUIorIO == 2)
	{
		if(systemPara.Language == 0)
			GUI_mainMessageDispIsolate("外部IO停止动作！", 16);
		else if(systemPara.Language == 1)
			GUI_mainMessageDispIsolate("External IO stop action.", 24);
	}
    MC_motorStopAll();
	MC_cmd(0,0);
	MC_cmd(1,0);
	MC_cmd(2,0);
	MC_cmd(3,0);
    GUI_runStatusDisp(0);
    if(systemPara.isInEmergencyStopEnable)
    {
		if(IBIO_getInput(12) == 1)
		{
			flag = 1;
			if(systemPara.Language == 0)
				GUI_mainMessageDispIsolate("触发急停，停止所以动作！", 24);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("Trigger emergency stop, stop all actions!", 41);

		}
    }
	EXO_sigOFF(1);
	EXO_sigOFF(2);
	EXO_sigOFF(3);
	EXO_sigOFF(4);
	EXO_sigOFF(5);
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
//    if(systemPara.isInEmergencyStopEnable)
//    {
//		GUI_mainMessageDispIsolate("提示信息：等待解除急停！", 24);
//		while(IBIO_getInput(12) == 1);
//		if(flag == 1)
//		{
//			flag= 0;
//			GUI_mainMessageDisp("提示信息：解除急停！", 20);
//		}
//    }
	Clear();
}

/**
 @brief UI停止动作
 */
void StopModeUI(void *pv)
{

	if(systemPara.Language == 0)
		GUI_mainMessageDisp("UI停止动作！", 16);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("UI stop action.", 15);

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
	EXO_sigOFF(5);
	Clear();
}


/**
 * 打开输出信号
 * * @param -no- 1 贴标完成, 2 取标完成, 3 初始化完成信号, 4 允许打印
 */

void EXO_sigON(uint8_t no)
{
	if(systemPara.status != STATUS_ONLINE)
		return;
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
 * * @param -no- 1 贴标完成, 2 取标完成, 3 初始化完成信号, 4 允许打印
 */
void EXO_sigOFF(uint8_t no)
{
	switch(no)
	{
	case 1:
		if(!EXIO_getOutput(2))
		{
			EXIO_setOutput(2, 1);
			GUI_setOutputSignalColorDisp(10,1);
		}
		break;
	case 2:
		if(!EXIO_getOutput(3))
		{
			EXIO_setOutput(3, 1);
			GUI_setOutputSignalColorDisp(11,1);
		}
		break;
	case 3:
		if(!EXIO_getOutput(0))
		{
			EXIO_setOutput(0,1);
			GUI_setOutputSignalColorDisp(8,1);
		}
		break;
	case 4:
		if(!EXIO_getOutput(1))
		{
			EXIO_setOutput(1,1);
			GUI_setOutputSignalColorDisp(9,1);
		}
		break;
	case 5:
//		EXIO_setOutput(4,1);
//		GUI_setOutputSignalColorDisp(12,1);
		break;
	}
}

/**
 *
 * * @param -no- 1 切料完成, 2 拉料完成, 3 初始化完成, 4 异常报警
 */
//检测输入电平，并超时报警
uint8_t GetIOLevel(uint8_t pin, uint8_t level, uint16_t time)
{
	uint16_t count = 0;
	uint8_t IOlevel;
	delay_ms(100);
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

void Alarm(void *pv)
{
	if(systemPara.status == STATUS_ONLINE)
	{
		EXO_sigOFF(4);
		EXO_sigOFF(1);
		EXO_sigOFF(2);
		EXO_sigOFF(3);
	}
	MC_motorStopAll();

	if(systemPara.AlarmFlag != 3 && systemPara.AlarmFlag != 4 && systemPara.AlarmFlag != 5)
	{
		GUI_runStatusDisp(0);
		Clear();
	}

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

//升降气缸
uint8_t SJ_Action(uint8_t Action)
{
	if(Action == 0)
	{
		if(systemPara.Language == 0)
			MAIN_INFO_SJ_CYL_OUT();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_SJ_CYL_OUT();

		IBIO_setOutput(1, 0);
		GUI_setOutputSignalColorDisp(0,0);
		if(GetIOLevel(6,0,2000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_SJ_CYL_OUT_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_SJ_CYL_OUT_TIMEOUT();
			systemPara.AlarmFlag = 6;
			return 0;
		}
	}
	else
	{

		if(systemPara.Language == 0)
			MAIN_INFO_SJ_CYL_IN();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_SJ_CYL_IN();

		IBIO_setOutput(1, 1);
		GUI_setOutputSignalColorDisp(0,1);
		if(GetIOLevel(5,0,2000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_SJ_CYL_IN_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_SJ_CYL_IN_TIMEOUT();
			systemPara.AlarmFlag = 6;
			return 0;
		}
	}
	return 1;
}

//贴标气缸
uint8_t TB_Action(uint8_t Action)
{
	if(Action == 0)
	{
		if(systemPara.Language == 0)
			MAIN_INFO_TB_CYL_OUT();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_TB_CYL_OUT();

		IBIO_setOutput(2, 0);
		GUI_setOutputSignalColorDisp(1,0);
		if(GetIOLevel(8,0,2000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_TB_CYL_OUT_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_TB_CYL_OUT_TIMEOUT();
			systemPara.AlarmFlag = 7;
			return 0;
		}
	}
	else
	{

		if(systemPara.Language == 0)
			MAIN_INFO_TB_CYL_IN();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_TB_CYL_IN();

		IBIO_setOutput(2, 1);
		GUI_setOutputSignalColorDisp(1,1);
		if(GetIOLevel(7,0,2000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_TB_CYL_IN_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_TB_CYL_IN_TIMEOUT();
			systemPara.AlarmFlag = 7;
			return 0;
		}
	}
	return 1;
}

//旋转气缸
uint8_t XZ_Action(uint8_t Action)
{
	if(Action == 0)
	{
		if(systemPara.Language == 0)
			MAIN_INFO_XZ_CYL_OUT();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_XZ_CYL_OUT();

		IBIO_setOutput(3, 0);
		GUI_setOutputSignalColorDisp(2,0);
		if(GetIOLevel(10,0,2000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_XZ_CYL_OUT_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_XZ_CYL_OUT_TIMEOUT();
			systemPara.AlarmFlag = 8;
			return 0;
		}
	}
	else
	{

		if(systemPara.Language == 0)
			MAIN_INFO_XZ_CYL_IN();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_XZ_CYL_IN();

		IBIO_setOutput(3, 1);
		GUI_setOutputSignalColorDisp(2,1);
		if(GetIOLevel(9,0,2000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_XZ_CYL_IN_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_XZ_CYL_IN_TIMEOUT();
			systemPara.AlarmFlag = 8;
			return 0;
		}
	}
	return 1;
}

//接料平台气缸
uint8_t PT_Action(uint8_t Action)
{
	if(Action == 0)
	{
		if(systemPara.Language == 0)
			MAIN_INFO_PT_CYL_OUT();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_PT_CYL_OUT();

		IBIO_setOutput(4, 0);
		GUI_setOutputSignalColorDisp(3,0);
		if(GetIOLevel(12,0,2000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_PT_CYL_OUT_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_PT_CYL_OUT_TIMEOUT();
			systemPara.AlarmFlag = 9;
			return 0;
		}
	}
	else
	{

		if(systemPara.Language == 0)
			MAIN_INFO_PT_CYL_IN();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_PT_CYL_IN();

		IBIO_setOutput(4, 1);
		GUI_setOutputSignalColorDisp(3,1);
		if(GetIOLevel(11,0,2000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_PT_CYL_IN_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_PT_CYL_IN_TIMEOUT();
			systemPara.AlarmFlag = 9;
			return 0;
		}
	}
	return 1;
}

//真空控制
uint8_t ZK_Action(uint8_t Action)
{
	if(Action == 0)
	{
		if(systemPara.Language == 0)
			MAIN_INFO_ZK_OPEN();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_ZK_OPEN();

		IBIO_setOutput(5, 0);
		GUI_setOutputSignalColorDisp(4,0);
		if(GetIOLevel(4,0,2000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_ZK_OPEN_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_ZK_OPEN_TIMEOUT();
			systemPara.AlarmFlag = 10;
			return 0;
		}
	}
	else
	{

		if(systemPara.Language == 0)
			MAIN_INFO_ZK_CLOSE();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_ZK_CLOSE();

		IBIO_setOutput(5, 1);
		GUI_setOutputSignalColorDisp(4,1);
		if(GetIOLevel(4,1,2000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_ZK_CLOSE_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_ZK_CLOSE_TIMEOUT();
			systemPara.AlarmFlag = 10;
			return 0;
		}
	}
	return 1;
}
//压料平移气缸动作
uint8_t YLPY_Action(uint8_t Action)
{
	if(Action == 0)
	{
		if(systemPara.Language == 0)
			MAIN_INFO_YLPY_CYL_OUT();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_YLPY_CYL_OUT();

		EXIO_setOutput(4, 1);
		GUI_setOutputSignalColorDisp(12,1);
		EXIO_setOutput(5, 0);
		GUI_setOutputSignalColorDisp(13,0);
		if(GetEXIOLevel(5,0,3000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_YLPY_CYL_OUT_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_YLPY_CYL_OUT_TIMEOUT();
			return 0;
		}
	}
	else
	{

		if(systemPara.Language == 0)
			MAIN_INFO_YLPY_CYL_IN();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_YLPY_CYL_IN();

		EXIO_setOutput(4, 0);
		GUI_setOutputSignalColorDisp(12,0);
		EXIO_setOutput(5, 1);
		GUI_setOutputSignalColorDisp(13,1);
		if(GetEXIOLevel(4,0,3000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_YLPY_CYL_IN_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_YLPY_CYL_IN_TIMEOUT();
			return 0;
		}
	}
	return 1;
}

//压料气缸动作
uint8_t YL_Action(uint8_t Action)
{
	if(Action == 0)
	{
		if(systemPara.Language == 0)
			MAIN_INFO_YL_CYL_OUT();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_YL_CYL_OUT();

		EXIO_setOutput(6, 1);
		GUI_setOutputSignalColorDisp(14,1);
		EXIO_setOutput(7, 0);
		GUI_setOutputSignalColorDisp(15,0);
		if(GetEXIOLevel(7,0,3000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_YL_CYL_OUT_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_YL_CYL_OUT_TIMEOUT();
			return 0;
		}
	}
	else
	{

		if(systemPara.Language == 0)
			MAIN_INFO_YL_CYL_IN();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_YL_CYL_IN();

		EXIO_setOutput(6, 0);
		GUI_setOutputSignalColorDisp(14,0);
		EXIO_setOutput(7, 1);
		GUI_setOutputSignalColorDisp(15,1);
		if(GetEXIOLevel(6,0,3000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_YL_CYL_IN_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_YL_CYL_IN_TIMEOUT();
			return 0;
		}
	}
	return 1;
}
void Clear(void)
{
    systemPara.isWorkTaskRun = 0;
    systemPara.Initstatus = 0;
    systemPara.LackMaterral = 1; // 默认有料
    systemPara.doInit = 0;
    systemPara.doGohome = 0;
	systemPara.pickOnceTriggerByUIorIO = 0;
	systemPara.pickOnceOK = 0;
	systemPara.stickOnceTriggerByUIorIO = 0;
	systemPara.stickOnceOK = 0;
	systemPara.stickOnceOK = 1;
	systemPara.RSTOnceTriggerByUIorIO = 0;
	systemPara.STOPOnceTriggerByUIorIO = 0;
	systemPara.RSTOnceOK = 0;
	systemPara.RunStatus = 0;
	systemPara.PrintStatus = 0;
	systemPara.Debug = 0;


	systemPara.ReceivingOnceTriggerByUIorIO = 0;	///< 触发一次接标
	systemPara.ReceivingOnceOK = 0;					///< 触发一次接标完成

	systemPara.ReceivRSTOnceTriggerByUIorIO = 0;	///< 触发一次接标复位
	systemPara.ReceivRSTOnceOK = 0;					///< 触发一次接标复位完成

	ReceiLabel_step = 0;
	ReceiLabelRst_step = 0;

	PickRst_step = 0;
	Pick_step = 0;
	Stick_step = 0;
	Fetch_count = 0;
	Label_count = 0;

	KP_step = 0;  // 空跑步骤
	OLM_step = 0;// 联机模式
	Gohome_step = 0;
	Init_step = 0;
	ms_count = 0;
	replacenment_flag = 0;

	Label_distance = 0;
	Fetch_distance = 0;
	X_position = 0;
	Fetch_time = 0;

	Pick_Position_step = 0;
	Pick_Position_count = 0;
	Stick_Position_step = 0;
	Stick_Position_count = 0;
}

void switchMode(uint8_t mode)
{
	if(mode == 1)
	{
		systemPara.status = STATUS_ONLINE;
		AT24CXX_WriteOneByte(STORAGE_WORKMODE_ADDR, systemPara.status);
		GUI_switchModeDisp(systemPara.status);

		if(systemPara.Language == 0)
			MAIN_INFO_SW2ONLINE();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_SW2ONLINE();

		GUI_switchPage(32);
		if(systemPara.Initstatus == 1)
		{
			if(systemPara.pickOnceOK == 1)
			{
				EXO_sigON(2);
//				USART1_sendBuf(sendBuffer[7],8);
			}
			else
			{
				delay_ms(20);
				if(IBIO_getInput(3) != 0)
				{
					EXO_sigON(4);
//					USART1_sendBuf(sendBuffer[11],8);
				}
				else
				{
					systemPara.pickOnceTriggerByUIorIO = 1;
				}
			}

			EXO_sigON(3);
		}
		else
		{
			systemPara.Initstatus = 0;
			GUI_runStatusDisp(0);
			systemPara.isWorkTaskRun = 0;

			if(systemPara.Language == 0)
				MAIN_TIPS_INNI_FIRST();
			else if(systemPara.Language == 1)
				EN_MAIN_TIPS_INNI_FIRST();
		}
	}
	else
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
		EXO_sigOFF(5);
		systemPara.status = STATUS_AUTO;
		AT24CXX_WriteOneByte(STORAGE_WORKMODE_ADDR, systemPara.status);
		GUI_switchModeDisp(systemPara.status);

		if(systemPara.Language == 0)
			MAIN_INFO_SW2DEBUG();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_SW2DEBUG();
	}
}


void M0driveALM(void)
{
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("报警信息：送料驱动器报警！", 26);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("Alarm info:Feeding driver alarm!", 32);

	LV8731V_cmd(0, 0);
	systemPara.AlarmFlag = 20;
	Alarm(0);
}

void M2driveALM(void)
{
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("报警信息：剥料驱动器报警！", 26);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("Alarm info: Stripping driver alarm!", 35);

	LV8731V_cmd(2, 0);
	systemPara.AlarmFlag = 21;
	Alarm(0);
}

uint8_t EmergencyStop(void)
{
	if(systemPara.isInEmergencyStopEnable)
	{
		if(IBIO_getInput(12) == 1 )
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
