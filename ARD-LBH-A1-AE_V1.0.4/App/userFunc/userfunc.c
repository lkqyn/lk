#include "userfunc/userfunc.h"
#include "motorctrl/motorctrl.h"
#include "paramanager/paramanager.h"
#include "inboardio/inboardio.h"
#include "exio/exio.h"
#include "dwin/gui.h"
#include "pwm1/pwm1.h"
#include "delay/delay.h"
#include "eventHandler/eventHandler.h"
#include "easyModbus/easyModbus.h"
#include "storage/storage.h"
#include "at24cxx/at24cxx.h"
#include "MdriverAlarmIO/mdriveralarmio.h"
#include "usart1/USART1.h"
#include "inputscan/inputscan.h"

extern void selectAutoMode(void *pv);
extern void selectManualMode(void *pv);
extern void selectFreeRunMode(void *pv);

uint64_t songliao_count = 0; // 送料计数

//功能函数步数累加
uint16_t KP_step = 0;  // 空跑步骤
uint16_t OLM_step = 0;// 联机模式
uint16_t OLM_DOWN_step = 0;// 下光纤
uint16_t OLM_UP_step = 0;// 上光纤
uint16_t OLM_FIXED_step = 0;// 上光纤
uint16_t Gohome_step = 0;
uint16_t Init_step = 0;
uint16_t IO_HC_step = 0;
uint16_t IO_RST_step = 0;
uint16_t IO_BL_step = 0;

uint16_t detection = 0;

//功能函数延时累加
uint16_t ms_count = 0;
uint16_t INIT_ms_count = 0;
uint16_t SL_ms_count = 0;
uint16_t BL_ms_count = 0;
uint16_t RST_ms_count = 0;
uint16_t RST_count = 0;
uint16_t SL_count = 0;

uint8_t FLflag = 0; //放料标志位
uint8_t USLflag = 0; //上收料料标志位
uint8_t DSLflag = 0; //下收料料标志位

uint8_t replacenment_flag = 0;//换料标志

uint16_t recvOffsetPulseCount = 0;//偏移距离
uint16_t recvDistance = 0; //送料长度
uint16_t recvBLDistance = 0; //换料后撤行程
uint16_t recvDistance1 = 0; //退料长度
uint8_t recvOffsetdir = 1; //偏移方向，默认正向
uint32_t recvOffsetPulseCount1 = 0, recvOffsetPulseCount2 = 0; //接收补偿暂存

uint32_t offsetSpeed1[20] ={0};
uint32_t TempArray1[20] = {78, 75, 72, 70, 68, 65, 60, 55, 52, 50,
                                    48, 45, 43, 41, 39, 38, 36, 35, 34, 32}; // 齿轮后撤补偿数组

uint32_t TempArray2[20] = {180, 170, 160, 150, 140, 130, 120, 110, 100, 90,
                                    80, 70, 60, 55, 48, 35, 32, 18, 10, 5}; // 丝杆后撤补偿数组

void InitOffsetSpeedArr(uint16_t BO_MOTOR_Amp) // 初始化剥料复位补偿数组
{
    if (BO_MOTOR_Amp >= 30)// 齿轮比>= 30 齿轮
    {
        for (int i = 0; i < 20; i++)
        {
            offsetSpeed1[i] = TempArray1[i];
        }
    }
    else if(BO_MOTOR_Amp < 30)// 齿轮比<30 丝杆
    {
        for (int i = 0; i < 20; i++)
        {
            offsetSpeed1[i] = TempArray2[i];
        }
    }
}

/**
 @brief 控制输出信号
 */
void controlOutputSignal(void *pv)
{
	static uint8_t flag[] =
	{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

	if(systemPara.isInEmergencyStopEnable) // 紧急急停状态
	{
		if(IBIO_getInput(12) == 1 )
		{
			if(systemPara.Language == 0)
				POP_UP_INFO_RES_E_SOTP();
			else if(systemPara.Language == 1)
				EN_POP_UP_INFO_RES_E_SOTP();
			return;
		}
	}

    if(systemPara.status == STATUS_ONLINE )//联机模式
    {
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE1();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE1();
    	return;
    }

    if(systemPara.status == STATUS_FREERUN && systemPara.Initstatus )//空跑模式运行中禁止触发输出信号
   {
		if(systemPara.Language == 0)
			POP_UP_INFO_AGING_RUN();
		else if(systemPara.Language == 1)
			EN_POP_UP_INFO_AGING_RUN();
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

/**
 * @brief 剥刀后撤
 *        代码采用分步扫描方式；
 *        支持2次剥料，支持1次剥料后再次触发剥料；
 *        支持单段慢速后撤再快速后撤
 */
void BoliaoHC_Step_IO(void *pv)
{
	switch(IO_HC_step)
	{
		case 0:
			if (!systemPara.HCOnceTriggerByUIorIO) // 等待一次后撤标志
			{
				return;
			}

			if (systemPara.HCOnceTriggerByUIorIO == 2) // 当前后撤为第二次后撤
			{	//手动2次后撤
				IO_HC_step = 13;
				return;
			}

			if(IBIO_getInput(1))
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("请先复位！", 10);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Please reset first.", 19);
				systemPara.HCOnceTriggerByUIorIO = 0;
				return;
			}

			EXO_sigOFF(1);
			systemPara.HCOnceOK =0;
			systemPara.HCTwoOK = 0;
			systemPara.RSTOnceOK =0;
			systemPara.RunStatus = 3;
			IO_HC_step ++;
			break;
		case 1:
			if(IBIO_getOutput(1))
			{
				if(systemPara.Language == 0)
					MAIN_INFO_YL_CYL_OUT();
				else if(systemPara.Language == 1)
					EN_MAIN_INFO_YL_CYL_OUT();
				YL_Action(0);
				IO_HC_step ++;
			}
			else
				IO_HC_step += 2;
			break;
		case 2:
			delay_ms(10);ms_count ++;
			if(ms_count > YLdelay_count)
			{ms_count = 0;IO_HC_step ++;}
			break;
		case 3:
			if(controlPara.boLowDistance > 0) // 慢速剥料行程 > 0
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("剥刀慢速后撤开始.", 17);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Slow stripping starts.", 22);
				systemPara.RSTmode = 0;
				if(controlPara.boDistance > controlPara.boLowDistance) // 后撤行程1 > 慢速剥料行程 后撤短距离
					MC_motorMoveDistance(BO_MOTOR, controlPara.boLowSpeed, systemPara.MotorBoDir,
											controlPara.boLowDistance);
				else
					MC_motorMoveDistance(BO_MOTOR,controlPara.boLowSpeed, systemPara.MotorBoDir,
											controlPara.boDistance);
				IO_HC_step ++;
			}
			else
				IO_HC_step += 2;
			break;
		case 4:
			if(motor_ch[BO_MOTOR].status == Motor_Stop) // 等待剥料电机停止运动
			{
				IO_HC_step ++;
			}
			break;
		case 5:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("剥刀后撤开始.", 13);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Stripping begins.", 17);
			if(controlPara.boDistance > controlPara.boLowDistance) // 后撤行程1 > 慢速后撤行程 继续后撤
			{
				systemPara.RSTmode = 0;
				MC_motorMoveDistance(BO_MOTOR,controlPara.boSpeed, systemPara.MotorBoDir,
										controlPara.boDistance - controlPara.boLowDistance);
			}
			IO_HC_step ++;
			break;
		case 6:
			if(motor_ch[BO_MOTOR].status == Motor_Stop) // 等待剥料电机停止运动
			{
				if(!IBIO_getInput(1) && controlPara.boDistance >= 10 + controlPara.oriOffset)
				{//后撤距离大于10MM+补偿值，检测是否离开远点感应器，否则报警
					delay_ms(5);
					if(!IBIO_getInput(1))
					{
						if(systemPara.Language == 0)
							ERR_INFO_STEPBACK_TIMEOUT();
						else if(systemPara.Language == 1)
							EN_ERR_INFO_STEPBACK_TIMEOUT();
						systemPara.AlarmFlag = 9;
						Alarm(0);
						return;
					}
				}
				else
				{
					IO_HC_step ++;
				}
			}
			break;
		case 7:
			IO_HC_step ++;
			break;
		case 8:
			IO_HC_step ++;
			break;
		case 9:
			EXO_sigON(1);
			EXO_sigOFF(2);

			if(systemPara.Language == 0)
				GUI_mainMessageDisp("剥刀后撤到位.", 13);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("Stripping completed.", 20);

			IO_HC_step  ++;
			break;
		case 10:
			if(controlPara.boDistance2 > 0 && systemPara.status == STATUS_ONLINE) // 后撤行程2  > 0 && 联机模式
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("等待剥料信号断开.", 17);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Wait for the stripping signal to disconnect.", 44);
				IO_HC_step  ++;
			}
			else
				IO_HC_step = 18;
			break;
		case 11:
#if CONF_FEEDER_VER == 1
			if(EXIO_getInput(1))
			{
#else
			if(EXIO_getInput(2))
			{
#endif
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("等待2次剥料信号.", 16);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Wait for the second stripping signal.", 37);
				IO_HC_step  ++;
			}
			break;
		case 12:
#if CONF_FEEDER_VER == 1
			if(!EXIO_getInput(1))
			{
				delay_ms(5);
				if(!EXIO_getInput(1))
				{
#else
			if(!EXIO_getInput(2))
			{
				delay_ms(5);
				if(!EXIO_getInput(2))
				{
#endif
					EXO_sigOFF(1);
					IO_HC_step  ++;
				}
			}
			break;
		case 13:
			if(controlPara.boLowDistance > 0) // 慢速后撤行程 > 0
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("剥刀慢速后撤开始.", 17);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Slow stripping starts.", 22);

				systemPara.RSTmode = 0;
				if(controlPara.boDistance2 > controlPara.boLowDistance) // 后撤行程2 > 慢速剥料行程 后撤短距离
					MC_motorMoveDistance(BO_MOTOR,controlPara.boLowSpeed, systemPara.MotorBoDir,
											controlPara.boLowDistance);
				else
					MC_motorMoveDistance(BO_MOTOR,controlPara.boLowSpeed, systemPara.MotorBoDir,
											controlPara.boDistance2);
				IO_HC_step ++;
			}
			else
				IO_HC_step += 2;
			break;
		case 14:
			if(motor_ch[BO_MOTOR].status == Motor_Stop) // 等待剥料电机运动停止
			{
				IO_HC_step ++;
			}
			break;
		case 15:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("2次剥刀后撤开始.", 16);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Start of the second stripping.", 30);
			if(controlPara.boDistance2 > controlPara.boLowDistance) // 后撤行程2 > 慢速剥料行程 继续后撤
			{
				systemPara.RSTmode = 0;
				MC_motorMoveDistance(BO_MOTOR,controlPara.boSpeed, systemPara.MotorBoDir,
										controlPara.boDistance2 - controlPara.boLowDistance);
			}
			IO_HC_step ++;
			break;
		case 16:
			if(motor_ch[BO_MOTOR].status == Motor_Stop) // 等待剥料电机运动停止
			{
				IO_HC_step ++;
			}
			break;
		case 17:
			systemPara.HCTwoOK = 1;
			EXO_sigON(1);

			if(systemPara.Language == 0)
				GUI_mainMessageDisp("2次剥刀后撤到位.", 16);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Second stripping completed.", 27);
			IO_HC_step  ++;
			break;
		case 18:
			systemPara.RunStatus = 0;
			systemPara.HCOnceOK = 1;
			systemPara.RSTOnceOK =0;
			systemPara.givenOnceOK = 0;
			systemPara.HCOnceTriggerByUIorIO = 0;
			IO_HC_step = 0;
			break;
	}
}

void BoliaoRST_Step_IO(void *pv)
{
	uint32_t offsetPulseCount;//原点补偿
	uint32_t RunSpeed;

	switch(IO_RST_step)
	{
		case 0:
			if (!systemPara.RSTOnceTriggerByUIorIO) // 等待剥刀复位标志
			{
				return;
			}

			if (!systemPara.HCOnceOK )
			{
				//GUI_mainMessageDisp("提示信息：请先剥料动作.", 23);
				systemPara.RSTOnceTriggerByUIorIO = 0;
				return;
			}

			systemPara.RSTOnceOK = 0;
			systemPara.RunStatus = 4;
			IO_RST_step ++;
			break;
		case 1:
			if(systemPara.isRSTAutoSongEnable && systemPara.status == STATUS_ONLINE)
			{
				YL_Action(1);
				IO_RST_step ++;
			}
			else if(IBIO_getOutput(1))
			{
				YL_Action(0);
				IO_RST_step ++;
			}
			else
				IO_RST_step += 2;
			break;
		case 2:
			delay_ms(10);RST_ms_count ++;
			if(RST_ms_count > YLdelay_count)
			{RST_ms_count = 0;IO_RST_step ++;}
			break;
		case 3://匹配同步送料节拍ControlPara
//			if(systemPara.isRSTAutoSongEnable)
//			{
//				if(++RST_count >=  4)
//				{RST_count = 0;IO_RST_step ++;}
//			}
//			else
			{
				IO_RST_step ++;
			}
			break;
		case 4:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("剥刀复位开始...", 15);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("Stripping reset starts...", 25);
			//后撤行程1 + 后撤行程2 >= 10mm + 偏移量
			if((controlPara.boDistance + controlPara.boDistance2) >= (10 + controlPara.oriOffset))
			{//复位距离大于10mm走传感模式
				//传感模式加补偿
				offsetPulseCount = controlPara.oriOffsetPulseCount;
				RunSpeed = controlPara.boaRstSpeed *0.1f;//获取剥料复位速度
				if(RunSpeed < 1)
					RunSpeed = 1;
				offsetPulseCount = offsetPulseCount + MC_mm2pulse1(BO_MOTOR,offsetSpeed1[RunSpeed-1]);
				systemPara.RSTmode = 1;//传感模式
				MC_motorRunAndOffsetByEXTI(BO_MOTOR, controlPara.boaRstSpeed,
										!systemPara.MotorBoDir, offsetPulseCount, RSTlimitPulseCount);
			}
			else
			{
				systemPara.RSTmode = 0;//定长模式
				if(systemPara.HCTwoOK) // 后撤两次完成时，复位 后撤行程1 + 后撤行程2
					MC_motorMoveDistance(BO_MOTOR,controlPara.boaRstSpeed, !systemPara.MotorBoDir,
											controlPara.boDistance + controlPara.boDistance2);
				else // 只后撤行程1时，复位 后撤行程1
					MC_motorMoveDistance(BO_MOTOR,controlPara.boaRstSpeed, !systemPara.MotorBoDir,
											controlPara.boDistance);
			}
#if CONF_Fiber_VER == 0
			if(IBIO_getInput(2) && systemPara.isRSTAutoSongEnable && systemPara.status == STATUS_ONLINE)
			{//联机复位自动送料
				delay_ms(2);
				if(IBIO_getInput(2))
#else
			if(IBIO_getInput(2) && IBIO_getInput(3) && systemPara.isRSTAutoSongEnable && systemPara.status == STATUS_ONLINE)
			{//联机复位自动送料
				delay_ms(2);
				if(IBIO_getInput(2) && IBIO_getInput(3))
#endif
				{
					if(systemPara.Language == 0)
						GUI_mainMessageDisp("送料中......", 12);
					else if(systemPara.Language == 1)
						GUI_mainMessageDisp("Feeding...", 12);

					systemPara.LowSpeedTrigger = 0;
					if(systemPara.isLowspeedSensorEnable)
					{//检测感应器减速停
						systemPara.SLmode = 2;
						MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenSpeed,systemPara.MotorGivenDir,
								controlPara.givenOffsetPulseCount,controlPara.givenDistancePulseCount);
					}
					else if(controlPara.givenLowDistancePulseCount > 0 && !systemPara.isLowspeedSensorEnable)
					{//走定长减速停
						systemPara.SLmode = 1;
						if(controlPara.givenDistancePulseCount <= controlPara.givenLowDistancePulseCount)
							//送料限位小于变速位置时
							MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenSpeed,systemPara.MotorGivenDir,
									controlPara.givenOffsetPulseCount,controlPara.givenDistancePulseCount);
						else
							MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenSpeed,systemPara.MotorGivenDir,
									controlPara.givenOffsetPulseCount,controlPara.givenLowDistancePulseCount);
					}
					else
					{
						systemPara.SLmode = 1;
						MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenSpeed,systemPara.MotorGivenDir,
								controlPara.givenOffsetPulseCount,controlPara.givenDistancePulseCount);
					}
				}
			}

			IO_RST_step ++;
			break;
		case 5:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				if(IBIO_getInput(1))
				{
					delay_ms(2);
					if(IBIO_getInput(1))
					{
						if(systemPara.Language == 0)
							ERR_INFO_GO_ORG_TIMEOUT();
						else if(systemPara.Language == 1)
							EN_ERR_INFO_GO_ORG_TIMEOUT();

						systemPara.AlarmFlag = 4;
						Alarm(0);
						return;
					}
				}
				IO_RST_step ++;
			}
			break;
		case 6:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("剥刀复位结束.", 13);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("Stripping reset completed.", 26);
			IO_RST_step ++;
			break;
		case 7:
			// 关闭剥料完成信号
			EXO_sigOFF(1);
			EXO_sigON(5);
			systemPara.givenOnceOK = 0;
			systemPara.RunStatus = 0;
			systemPara.HCOnceOK = 0;
			systemPara.HCTwoOK = 0;
			systemPara.RSTOnceOK = 1;
			systemPara.RSTOnceTriggerByUIorIO = 0;
			IO_RST_step = 0;
			break;
	}
}

void BoliaoHCRST_Step_IO(void *pv)
{
	uint32_t offsetPulseCount;//原点补偿
	uint32_t RunSpeed;

	switch(IO_BL_step)
	{
		case 0:
			if (!systemPara.BLOnceTriggerByUIorIO)
			{
				return;
			}
			systemPara.RunStatus = 3;
			IO_BL_step ++;
			break;
		case 1:
			if(IBIO_getOutput(1))
			{
				YL_Action(0);
				IO_BL_step ++;
			}
			else
				IO_BL_step += 2;
			break;
		case 2:
			delay_ms(10);ms_count ++;
			if(ms_count > YLdelay_count)
			{ms_count = 0;IO_BL_step ++;}
			break;
		case 3:
			if(controlPara.boLowDistance > 0)
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("剥刀慢速后撤开始.", 17);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Slow stripping starts.", 22);

				systemPara.RSTmode = 0;
				if(controlPara.boDistance > controlPara.boLowDistance) // 后撤行程1 > 慢速后撤行程， 后撤 慢速后撤行程
					MC_motorMoveDistance(BO_MOTOR,controlPara.boLowSpeed, systemPara.MotorBoDir,
											controlPara.boLowDistance);
				else // 后撤行程1 < 慢速后撤行程，后撤 后撤行程1
					MC_motorMoveDistance(BO_MOTOR,controlPara.boLowSpeed, systemPara.MotorBoDir,
											controlPara.boDistance);

				IO_BL_step ++;
			}
			else
				IO_BL_step += 2;
			break;
		case 4:
			if(motor_ch[BO_MOTOR].status == Motor_Stop) // 等待剥料电机停止运动
			{
				IO_BL_step ++;
			}
			break;
		case 5:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("剥刀后撤开始.", 13);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Stripping begins.", 17);
			systemPara.RSTmode = 0;
			if(controlPara.boDistance < controlPara.boLowDistance) // 后撤行程1 < 慢速剥料行程时，直接后撤2段行程
			{
				MC_motorMoveDistance(BO_MOTOR,controlPara.boSpeed, systemPara.MotorBoDir,
										controlPara.boDistance2);
			}
			else if(controlPara.boDistance >= controlPara.boLowDistance)// 后撤行程1 > 慢速剥料行程时， 后撤距离 = 后撤1 - 慢 + 后撤2
			{
				MC_motorMoveDistance(BO_MOTOR,controlPara.boSpeed, systemPara.MotorBoDir,
									controlPara.boDistance + controlPara.boDistance2 - controlPara.boLowDistance);
			}
			IO_BL_step ++;
			break;
		case 6:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)// 等待剥料电机停止运动
			{
				if(!IBIO_getInput(1) && (controlPara.boDistance) >= 10 + controlPara.oriOffset)
				{
					delay_ms(5);
					if(!IBIO_getInput(1))
					{
						if(systemPara.Language == 0)
							ERR_INFO_STEPBACK_TIMEOUT();
						else if(systemPara.Language == 1)
							EN_ERR_INFO_STEPBACK_TIMEOUT();
						systemPara.AlarmFlag = 9;
						Alarm(0);
						return;
					}
				}
				else
				{
					IO_BL_step ++;
				}
			}
			break;
		case 7:

			if(systemPara.Language == 0)
				GUI_mainMessageDisp("剥刀后撤到位.", 13);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("Stripping completed.", 20);

			IO_BL_step  ++;
			break;
		case 8:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("剥刀复位开始...", 15);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("Stripping reset starts...", 25);
			if(controlPara.boDistance+controlPara.boDistance2 >= 10 + controlPara.oriOffset)
			{//复位距离大于10mm+原点偏移 走传感模式
				//传感模式加补偿
				offsetPulseCount = controlPara.oriOffsetPulseCount;
				RunSpeed = controlPara.boaRstSpeed * 0.1f;//获取剥料复位速度
				if(RunSpeed < 1)
					RunSpeed = 1;
				offsetPulseCount = offsetPulseCount + MC_mm2pulse1(BO_MOTOR,offsetSpeed1[RunSpeed - 1]);
				systemPara.RSTmode = 1;//传感模式
				MC_motorRunAndOffsetByEXTI(BO_MOTOR, controlPara.boaRstSpeed,!systemPara.MotorBoDir,
												offsetPulseCount,RSTlimitPulseCount);
			}
			else
			{
				systemPara.RSTmode = 0;//定长模式
				MC_motorMoveDistance(BO_MOTOR,controlPara.boaRstSpeed, !systemPara.MotorBoDir,
										controlPara.boDistance+controlPara.boDistance2);
			}
			IO_BL_step ++;
			break;
		case 9:
			if(motor_ch[BO_MOTOR].status == Motor_Stop) // 等待剥料电机停止运动
			{
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
				IO_BL_step ++;
			}
			break;
		case 10:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("剥刀复位结束.", 13);
			else if(systemPara.Language == 1)
				GUI_mainMessageDispIsolate("Stripping reset completed.", 26);
			IO_BL_step ++;
			break;
		case 11:
			IO_BL_step ++;
			break;
		case 12:
			IO_BL_step ++;
			break;
		case 13:
			EXO_sigON(5);
			IO_BL_step ++;
			break;
		case 14:
			systemPara.RunStatus = 0;
			systemPara.BLOnceOK = 1;
			systemPara.givenOnceOK = 0;
			systemPara.BLOnceTriggerByUIorIO = 0;
			IO_BL_step = 0;
			break;
	}
}

void huiyuandian_UI(void *pv)
{
//	static 	char GUIDeccmd[]={0x5a,0xa5,0x07,0x82,0x00, 0x80, 0x00, 0x00, 0x00, 0x92};
//	//	旋转页面180°
//	if (GUIDeccmd[9] != 0x90) {
//		GUIDeccmd[9] = 0x90;
//	} else {
//		GUIDeccmd[9] = 0x92;
//	}
//
//	USART2_sendBuf(GUIDeccmd, 10);
	if(EmergencyStop() == 1)
		return;
	systemPara.doGohome = 1;
}

/**
 @brief 联机模式函数调用，分功能
 */
void GivenMatieralOnline(void *pv)
{
	if(systemPara.sensorChosen == 1)
	{
		onlineMode_Step_Down_task(pv);
	}
	else if(systemPara.sensorChosen == 2)
	{
		onlineMode_Step_Up_task(pv);
	}
	else if(systemPara.sensorChosen == 3)
	{
		onlineMode_Step_Fixed_task(pv);
	}
}

/**
 * @brief 联机模式 下光纤
 *        代码采用分步扫描方式；
 *        单感应器模式；v
 *
 */
void onlineMode_Step_Down_task(void *pv)
{
	uint32_t limitPulseCount;
	char str[8]="0";
	uint16_t len;

	if(systemPara.isRSTAutoSongEnable) // 复位自动送料使能
	{
		if(!EXIO_getInput(2))
		{//同步送料有料就清除
			systemPara.givenOnceTriggerByUIorIO = 0;
			systemPara.RunStatus = 0;
			OLM_DOWN_step = 0;
			return;
		}
	}

	switch(OLM_DOWN_step)
	{
		case 0:

			if (systemPara.givenOnceTriggerByUIorIO != 1) // 等待触发一次送料标志
			{
				return;
			}

			//调试模式
			if(systemPara.status != STATUS_ONLINE && !systemPara.RSTOnceOK)
			{
				systemPara.givenOnceTriggerByUIorIO = 0;
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("请先复位！", 10);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Please reset first.", 19);
				return;
			}
#if CONF_Fiber_VER == 0
			if(IBIO_getInput(2) == 0)
#else
			if(IBIO_getInput(2) == 0 || IBIO_getInput(3) == 0)
#endif

			{
				if(systemPara.Language == 0)
					MAIN_TIPS_HAVE_MATERIAL();
				else if(systemPara.Language == 1)
					EN_MAIN_TIPS_HAVE_MATERIAL();

				systemPara.givenOnceTriggerByUIorIO = 0;
				systemPara.givenOnceOK = 1;
				systemPara.AlarmFlag = 7;
				sendBuffer[23][5] = 0x07;
				USART1_sendBuf(sendBuffer[23],8);
				return;
			}

			EXO_sigOFF(2);
			EXO_sigOFF(4);
			systemPara.givenOnceOK = 0;
			systemPara.RunStatus = 2;
			OLM_DOWN_step ++;
			break;
		case 1:
			if(systemPara.isCylinderSensorEnable || (!(IBIO_getOutput(3)) || !(IBIO_getInput(10))) )
			{
				if(SS_Action(1) == 0 )
				{
					Alarm(0);
					return;
				}
			}
			OLM_DOWN_step ++;
			break;
		case 2:
			if(!IBIO_getOutput(1))
			{
				YL_Action(1);
				OLM_DOWN_step ++;
			}
			else {
				OLM_DOWN_step += 2;
			}
			break;
		case 3:
			delay_ms(10);SL_ms_count ++;
			if(SL_ms_count >= YLdelay_count)
			{SL_ms_count = 0;OLM_DOWN_step ++;}
			break;
		case 4:
			if(!IBIO_getOutput(2))
			{
				JL_Action(0);
				OLM_DOWN_step ++;
			}
			else
				OLM_DOWN_step += 2;
			break;
		case 5:
			delay_ms(10);SL_ms_count ++;
			if(SL_ms_count >= JLdelay_count)
			{SL_ms_count = 0;OLM_DOWN_step ++;}
			break;
		case 6:
#if CONF_LET_VER == 1
			if (systemPara.isAutoLetMetalEnable)
			{
				if (IBIO_getInput(7) == 0)
				{
					flTask_handler(0);
				}
			}
#endif
			OLM_DOWN_step ++;
			break;
		case 7:
#if CONF_LET_VER == 0
			DSLflag = 0;
			DownslTask_handler(0);
#endif
			USLflag = 0;
			UpslTask_handler(0);
			OLM_DOWN_step ++;
			break;
		case 8:
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop) // 电机为停止状态
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("送料中......", 12);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Feeding...", 12);

				systemPara.LowSpeedTrigger = 0;
				if(systemPara.isLowspeedSensorEnable)
				{//检测感应器减速停
					systemPara.SLmode = 2;
					MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenSpeed,systemPara.MotorGivenDir,
							controlPara.givenOffsetPulseCount,controlPara.givenDistancePulseCount);
				}
				else if(controlPara.givenLowDistancePulseCount > 0 && !systemPara.isLowspeedSensorEnable)
				{//走定长减速停
					systemPara.SLmode = 1;
					if(controlPara.givenDistancePulseCount <= controlPara.givenLowDistancePulseCount)
						//送料限位小于变速位置时
						MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenSpeed,systemPara.MotorGivenDir,
								controlPara.givenOffsetPulseCount,controlPara.givenDistancePulseCount);
					else
						MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenSpeed,systemPara.MotorGivenDir,
								controlPara.givenOffsetPulseCount,controlPara.givenLowDistancePulseCount);
				}
				else
				{
					systemPara.SLmode = 1;
					MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenSpeed,systemPara.MotorGivenDir,
							controlPara.givenOffsetPulseCount,controlPara.givenDistancePulseCount);
				}
			}
			OLM_DOWN_step ++;
			break;
		case 9:
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop) //等待送料电机停止运动
			{

#if CONF_Fiber_VER == 0
				if(IBIO_getInput(2))
#else
				if(IBIO_getInput(2) || IBIO_getInput(3))
#endif
					OLM_DOWN_step ++;
				else
					OLM_DOWN_step = 13;
			}
			else
			{

#if CONF_LET_VER == 0
				DSLflag = 0;
				DownslTask_handler(0);
#endif
				USLflag = 0;
				UpslTask_handler(0);
			}
			break;
		case 10:
			if(systemPara.LowSpeedTrigger != 2)
			{
				OLM_DOWN_step ++;
			}
			else if(systemPara.LowSpeedTrigger == 2)
				OLM_DOWN_step += 3;
			break;
		case 11:
			if((controlPara.givenLowDistancePulseCount > 0 && !systemPara.isLowspeedSensorEnable) || systemPara.LowSpeedTrigger == 1 )
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("慢速寻标中...", 13);
				else if(systemPara.Language == 1)
					GUI_mainMessageDispIsolate("In slow search.", 15);

				if(controlPara.givenDistancePulseCount <= controlPara.givenLowDistancePulseCount && !systemPara.isLowspeedSensorEnable)
				{//送料限位小于变速位置时
					limitPulseCount = 100;//多送100脉冲停止送料
					systemPara.SLmode = 10;
					MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenLowSpeed,systemPara.MotorGivenDir,
							controlPara.givenOffsetPulseCount,limitPulseCount);
				}
				else
				{
					systemPara.SLmode = 10;
					MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenLowSpeed, systemPara.MotorGivenDir,
												controlPara.givenOffsetPulseCount,
												controlPara.givenDistancePulseCount - PWM1_pulseCount);
				}
			}
			OLM_DOWN_step ++;
			break;
		case 12:
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				OLM_DOWN_step ++;
			}
			else
			{
#if CONF_LET_VER == 0
				DSLflag = 0;
				DownslTask_handler(0);
#endif
				USLflag = 0;
				UpslTask_handler(0);
			}
			break;
		case 13:
			if (motor_ch[GIVEN_MOTOR].stopEvent == EVENT_exceedPulseCountLimit)
			{
				if(systemPara.isGivenEndNoAlarmEnable)
				{
					systemPara.givenOnceOK = 2;
				}
				else
				{
					if(systemPara.Debug == 0) // 当前页面处于 主界面
					{
						GivenMotorExceedPulseLimitAlert();
						systemPara.RSTOnceOK = 1;
						OLM_DOWN_step = 0;
						return;
					}
				}
			}
			OLM_DOWN_step ++;
			break;
		case 14:
			if(!systemPara.isCloseHCEnable)
			{
				if(IBIO_getOutput(1))
				{
					YL_Action(0);
				}
				OLM_DOWN_step ++;
			}
			else
				OLM_DOWN_step += 2;
			break;
		case 15:
			delay_ms(10);SL_ms_count ++;
			if(SL_ms_count > YLdelay_count)
			{SL_ms_count = 0;OLM_DOWN_step ++;}
			break;
		case 16:
			if(systemPara.givenOnceOK != 2)
			{
				songliao_count ++;
				len = sprintf(str,"%d",songliao_count);
				GUI_showText(0x8000, "        ", 8);
				GUI_showText(0x8000, str, len);
#if CONF_Fiber_VER == 0
				EXO_sigON(2);
#endif
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("送料完成.", 9);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Feeding completed.", 18);
			}
			else
			{
				if(systemPara.Language == 0)
					MAIN_TIPS_NO_MATERIAL();
				else if(systemPara.Language == 1)
					EN_MAIN_TIPS_NO_MATERIAL();
			}
			if(systemPara.LackMaterral == 0 && systemPara.status == STATUS_ONLINE)
			{
				systemPara.LackMaterral = 1;
				if(systemPara.Language == 0)
					ERR_INFO_OVRE_MATERIAL();
				else if(systemPara.Language == 1)
					EN_ERR_INFO_OVRE_MATERIAL();
				systemPara.AlarmFlag = 5;
#if CONF_Fiber_VER == 0
				EXO_sigON(4);
#else
				EXO_sigOFF(3);
#endif
			}

			if(motor_ch[LET_MOTOR].status == Motor_Stop && IBIO_getInput(8))
			{//送料完成进行放料动作，放在送料过程中放料影响送料精度
				systemPara.FLmode = 1;
				MC_motorMoveDistance(LET_MOTOR,controlPara.letSpeed,systemPara.MotorLetDir,controlPara.letDistance);
			}
			motor_ch[GIVEN_MOTOR].motorHadRun = 0;
			systemPara.givenOnceTriggerByUIorIO = 0;
			systemPara.givenOnceOK = 1;
			systemPara.RunStatus = 0;
			OLM_DOWN_step = 0;
			break;
	}
}

/**
 * @brief 联机模式 上光纤
*         代码采用分步扫描方式；
*         单感应器模式；
*
 */
void onlineMode_Step_Up_task(void *pv)
{
	uint32_t limitPulseCount;
	char str[8]="0";
	uint16_t len;
	static uint8_t exio1flag = 1;

	switch(OLM_UP_step)
	{
		case 0:
			if(exio1flag && systemPara.RunStatus == 4 && !EXIO_getInput(0) && systemPara.status == STATUS_ONLINE)
			{
				delay_ms(5);
				if(!EXIO_getInput(0))
				{
					exio1flag = 0;
					systemPara.givenOnceTriggerByUIorIO = 2;
				}
			}
			if(EXIO_getInput(0))
				exio1flag = 1;

			if (systemPara.givenOnceTriggerByUIorIO != 2) // 等待触发一次送料标志
			{
				return;
			}

			if(systemPara.status != STATUS_ONLINE && !systemPara.RSTOnceOK)
			{
				systemPara.givenOnceTriggerByUIorIO = 0;
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("请先复位！", 10);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Please reset first.", 19);
				return;
			}
			EXO_sigOFF(2);
			EXO_sigOFF(4);
			systemPara.givenOnceOK = 0;
			systemPara.RunStatus = 2;
			OLM_UP_step ++;
			break;
		case 1:
			if(systemPara.isUpSensorDownWichoutEnable)
			{
#if CONF_Fiber_VER == 0
				if(!IBIO_getInput(2))
				{
					delay_ms(5);
					if(!IBIO_getInput(2))
#else
				if(!IBIO_getInput(2) || !IBIO_getInput(3))
				{
					delay_ms(5);
					if(!IBIO_getInput(2) || !IBIO_getInput(3))
#endif
					{
						if(systemPara.Language == 0)
							MAIN_TIPS_HAVE_MATERIAL();
						else if(systemPara.Language == 1)
							EN_MAIN_TIPS_HAVE_MATERIAL();

						systemPara.AlarmFlag = 7;
						sendBuffer[23][5] = 0x07;
						USART1_sendBuf(sendBuffer[23],8);

#if CONF_Fiber_VER == 0
						EXO_sigON(2);
#endif

						OLM_UP_step = 0;
						systemPara.givenOnceOK = 1;
						systemPara.RunStatus = 0;
						systemPara.givenOnceTriggerByUIorIO = 0;
						return;
					}
				}
			}
			OLM_UP_step ++;
			break;
		case 2:
			if(systemPara.isCylinderSensorEnable)
			{
				if(SS_Action(0) == 0)
				{
					Alarm(0);
					return;
				}
			}
			else if((!(IBIO_getOutput(3)) || !(IBIO_getInput(10))) )
			{
				if(SS_Action(1) == 0 )
				{
					Alarm(0);
					return;
				}
			}
			OLM_UP_step ++;
			break;
		case 3:
			if(!IBIO_getInput(3))
			{
				delay_ms(2);
				if(!IBIO_getInput(3))
				{
					if(systemPara.givenMode == 1)//有料不可送
					{
						if(systemPara.isCylinderSensorEnable && SS_Action(1) == 0)
						{//开启传感器气缸功能
							Alarm(0);
							return;
						}

						if(systemPara.Language == 0)
							MAIN_TIPS_HAVE_MATERIAL();
						else if(systemPara.Language == 1)
							EN_MAIN_TIPS_HAVE_MATERIAL();

						systemPara.AlarmFlag = 7;
						sendBuffer[23][5] = 0x07;
						USART1_sendBuf(sendBuffer[23],8);
#if CONF_Fiber_VER == 0
						EXO_sigON(2);
#endif
						OLM_UP_step = 0;
						systemPara.givenOnceOK = 1;
						systemPara.RunStatus = 0;
						systemPara.givenOnceTriggerByUIorIO = 0;
						return;
					}
					else
					{
						systemPara.ChkSensorLevel = 1;
					}
				}
			}
			else
			{
				systemPara.ChkSensorLevel = 0;
			}
			OLM_UP_step ++;
			break;
		case 4:
			if(!IBIO_getOutput(1))
			{
				YL_Action(1);
				OLM_UP_step ++;
			}
			else {
				OLM_UP_step += 2;
			}
			break;
		case 5:
			delay_ms(10);SL_ms_count ++;
			if(SL_ms_count > YLdelay_count)
			{SL_ms_count = 0;OLM_UP_step ++;}
			break;
		case 6:
			if(!IBIO_getOutput(2))
			{
				JL_Action(0);
				OLM_UP_step ++;
			}else {
				OLM_UP_step += 2;
			}
			break;
		case 7:
			delay_ms(10);SL_ms_count ++;
			if(SL_ms_count > JLdelay_count)
			{SL_ms_count = 0;OLM_UP_step ++;}
			break;
		case 8:
#if CONF_LET_VER == 1
			if (systemPara.isAutoLetMetalEnable)
			{
				if (IBIO_getInput(7) == 0)
				{
					flTask_handler(0);
				}
			}
#endif
			OLM_UP_step ++;
			break;
		case 9:
#if CONF_LET_VER == 0
			DSLflag = 0;
			DownslTask_handler(0);
#endif
			USLflag = 0;
			UpslTask_handler(0);
			OLM_UP_step ++;
			break;
		case 10:
			if(controlPara.givenLowDistancePulseCount > 0)
			{
				systemPara.SLmode = 1;
				if(controlPara.givenDistancePulseCount <= controlPara.givenLowDistancePulseCount)
					//送料限位小于变速位置时
					MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenSpeed, systemPara.MotorGivenDir,
											controlPara.givenOffsetPulseCount,controlPara.givenDistancePulseCount - controlPara.givenOffsetPulseCount);
				else
					MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenSpeed, systemPara.MotorGivenDir,
												controlPara.givenOffsetPulseCount, controlPara.givenLowDistancePulseCount - controlPara.givenOffsetPulseCount);
				OLM_UP_step ++;
			}
			else
				OLM_UP_step += 2;
			break;
		case 11:
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				OLM_UP_step ++;
			}
			else
			{
#if CONF_LET_VER == 0
				DSLflag = 0;
				DownslTask_handler(0);
#endif
				USLflag = 0;
				UpslTask_handler(0);
			}
			break;
		case 12:
			if(controlPara.givenLowDistancePulseCount > 0)
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("慢速寻标中...", 13);
				else if(systemPara.Language == 1)
					GUI_mainMessageDispIsolate("In slow search.", 15);

				if(controlPara.givenDistancePulseCount <= controlPara.givenLowDistancePulseCount)
				{//送料限位小于变速位置时
					limitPulseCount = 100;//多送100脉冲停止送料
					systemPara.SLmode = 10;
					MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenLowSpeed, systemPara.MotorGivenDir,
											controlPara.givenOffsetPulseCount,limitPulseCount);
				}
				else
				{
					systemPara.SLmode = 10;
					MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenLowSpeed, systemPara.MotorGivenDir,
												controlPara.givenOffsetPulseCount, controlPara.givenDistancePulseCount
												- PWM1_pulseCount);
				}
			}
			else
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("送料中...", 9);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Feeding...", 12);

				systemPara.SLmode = 1;
				MC_motorRunAndOffsetByEXTI(GIVEN_MOTOR, controlPara.givenSpeed, systemPara.MotorGivenDir,
										controlPara.givenOffsetPulseCount,controlPara.givenDistancePulseCount);
			}
			OLM_UP_step ++;
			break;
		case 13:
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				OLM_UP_step ++;
			}
			else
			{
#if CONF_LET_VER == 0
				DSLflag = 0;
				DownslTask_handler(0);
#endif
				USLflag = 0;
				UpslTask_handler(0);
			}
			break;
		case 14:
			if (motor_ch[GIVEN_MOTOR].stopEvent == EVENT_exceedPulseCountLimit)
			{
				if(systemPara.isGivenEndNoAlarmEnable)
				{
					systemPara.givenOnceOK = 2;
				}
				else
				{
					if(systemPara.Debug == 0)
					{
						GivenMotorExceedPulseLimitAlert();
						systemPara.RSTOnceOK = 1;
						OLM_UP_step = 0;
						return;
					}
				}
			}
			OLM_UP_step ++;
			break;
		case 15:
			if(systemPara.isUpSensorDownWichoutEnable)
			{

#if CONF_Fiber_VER == 0
				if(!IBIO_getInput(2))
				{
					delay_ms(5);
					if(!IBIO_getInput(2))
#else
				if(!IBIO_getInput(2) || !IBIO_getInput(3))
				{
					delay_ms(5);
					if(!IBIO_getInput(2) || !IBIO_getInput(3))
#endif
					{
						if(++SL_count >= 5)
						{
							if(systemPara.Language == 0)
								MAIN_TIPS_NO_MATERIAL();
							else if(systemPara.Language == 1)
								EN_MAIN_TIPS_NO_MATERIAL();
							Alarm(0);
							SL_count = 0;
							return;
						}
						else
						{
							OLM_UP_step = 3;
							break;
						}
					}
				}
				else
				{
					SL_count = 0;
				}
			}
			OLM_UP_step ++;
			break;
		case 16:
			if(systemPara.isCylinderSensorEnable == 1)
			{
				if(SS_Action(1) == 0)
				{
					Alarm(0);
					return;
				}
			}
			OLM_UP_step ++;
			break;
		case 17:
			if(!systemPara.isCloseHCEnable)
			{
				if(IBIO_getOutput(1))
				{
					YL_Action(0);
					OLM_UP_step ++;
				}
			}
			else
				OLM_UP_step += 2;
			break;
		case 18:
			delay_ms(10);SL_ms_count ++;
			if(SL_ms_count > JLdelay_count)
			{SL_ms_count = 0;OLM_UP_step ++;}
			break;
		case 19:
			if(systemPara.givenOnceOK != 2)
			{
				songliao_count ++;
				len = sprintf(str,"%d",songliao_count);
				GUI_showText(0x8000, "        ", 8);
				GUI_showText(0x8000, str, len);

#if CONF_Fiber_VER == 0
				EXO_sigON(2);
#endif

				if(systemPara.Language == 0)
					GUI_mainMessageDisp("送料完成.", 9);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Feeding completed.", 18);
			}
			else
			{
				if(systemPara.Language == 0)
					MAIN_TIPS_NO_MATERIAL();
				else if(systemPara.Language == 1)
					EN_MAIN_TIPS_NO_MATERIAL();
			}
			if(systemPara.LackMaterral == 0 && systemPara.status == STATUS_ONLINE)
			{
				systemPara.LackMaterral = 1;
				if(systemPara.Language == 0)
					ERR_INFO_OVRE_MATERIAL();
				else if(systemPara.Language == 1)
					EN_ERR_INFO_OVRE_MATERIAL();
				systemPara.AlarmFlag = 5;
#if CONF_Fiber_VER == 0
				EXO_sigON(4);
#else
				EXO_sigOFF(3);
#endif
			}
			motor_ch[GIVEN_MOTOR].motorHadRun = 0;
			systemPara.givenOnceTriggerByUIorIO = 0;
			systemPara.givenOnceOK = 1;
			systemPara.RunStatus = 0;
			OLM_UP_step = 0;
			break;
	}
}

/**
 * @brief 联机模式 上光纤
*         代码采用分步扫描方式；
*         单感应器模式；
*
 */
void onlineMode_Step_Fixed_task(void *pv)
{
	char str[8]="0";
	uint16_t len;

	switch(OLM_FIXED_step)
	{
		case 0:
			if (systemPara.givenOnceTriggerByUIorIO != 3)
			{
				return;
			}

			if(systemPara.status != STATUS_ONLINE && !systemPara.RSTOnceOK)
			{
				systemPara.givenOnceTriggerByUIorIO = 0;

				if(systemPara.Language == 0)
					GUI_mainMessageDisp("请先复位！", 10);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Please reset first.", 19);
				return;
			}
			if(systemPara.givenMode == 1)
			{
#if CONF_Fiber_VER == 0
				if(IBIO_getInput(2) == 0)
#else
				if(IBIO_getInput(2) == 0 || IBIO_getInput(3) == 0)
#endif
				{
					if(systemPara.Language == 0)
						MAIN_TIPS_HAVE_MATERIAL();
					else if(systemPara.Language == 1)
						EN_MAIN_TIPS_HAVE_MATERIAL();

					systemPara.givenOnceTriggerByUIorIO = 0;
					systemPara.givenOnceOK = 1;
					systemPara.AlarmFlag = 7;
					sendBuffer[23][5] = 0x07;
					USART1_sendBuf(sendBuffer[23],8);
					return;
				}
			}
			EXO_sigOFF(2);
			EXO_sigOFF(4);
			systemPara.givenOnceOK = 0;
			systemPara.RunStatus = 2;
			OLM_FIXED_step ++;
			break;
		case 1:

			OLM_FIXED_step ++;
			break;
		case 2:
			if(systemPara.isUpSensorDownWichoutEnable)
			{
#if CONF_Fiber_VER == 0
				if(!IBIO_getInput(2))
				{
					delay_ms(2);
					if(!IBIO_getInput(2))
#else
				if(!IBIO_getInput(2) || !IBIO_getInput(3))
				{
					delay_ms(2);
					if(!IBIO_getInput(2) || !IBIO_getInput(3))
#endif
					{
						if(systemPara.Language == 0)
							MAIN_TIPS_HAVE_MATERIAL();
						else if(systemPara.Language == 1)
							EN_MAIN_TIPS_HAVE_MATERIAL();
						systemPara.AlarmFlag = 7;
						sendBuffer[23][5] = 0x07;
						USART1_sendBuf(sendBuffer[23],8);
#if CONF_Fiber_VER == 0
						EXO_sigON(2);
#endif
						OLM_FIXED_step = 0;
						systemPara.givenOnceOK = 1;
						systemPara.RunStatus = 0;
						systemPara.givenOnceTriggerByUIorIO = 0;
						return;
					}
				}
			}
			OLM_FIXED_step ++;
			break;
		case 3:
			if(!IBIO_getOutput(1))
			{
				YL_Action(1);
				OLM_FIXED_step ++;
			}
			else {
				OLM_FIXED_step += 2;
			}
			break;
		case 4:
			delay_ms(10);SL_ms_count ++;
			if(SL_ms_count > YLdelay_count)
			{SL_ms_count = 0;OLM_FIXED_step ++;}
			break;
		case 5:
			if(!IBIO_getOutput(2))
			{
				JL_Action(0);
				OLM_FIXED_step ++;
			}else {
				OLM_FIXED_step += 2;
			}
			break;
		case 6:
			delay_ms(10);SL_ms_count ++;
			if(SL_ms_count > JLdelay_count)
			{SL_ms_count = 0;OLM_FIXED_step ++;}
			break;
		case 7:
#if CONF_LET_VER == 1
			if (systemPara.isAutoLetMetalEnable)
			{
				if (IBIO_getInput(7) == 0)
				{
					flTask_handler(0);
				}
			}
#endif
			OLM_FIXED_step ++;
			break;
		case 8:
#if CONF_LET_VER == 0
			DSLflag = 0;
			DownslTask_handler(0);
#endif
			USLflag = 0;
			UpslTask_handler(0);
			OLM_FIXED_step ++;
			break;
		case 9:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("送料中......", 12);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Feeding...", 12);
			systemPara.SLmode = 0;
			MC_motorMoveDistance(GIVEN_MOTOR, controlPara.givenSpeed, systemPara.MotorGivenDir, controlPara.givenDistance);
			OLM_FIXED_step ++;
			break;
		case 10:
			if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
			{
				OLM_FIXED_step ++;
			}
			else
			{
#if CONF_LET_VER == 0
				DSLflag = 0;
				DownslTask_handler(0);
#endif
				USLflag = 0;
				UpslTask_handler(0);
			}
			break;
		case 11:
			if(systemPara.isUpSensorDownWichoutEnable)
			{
#if CONF_Fiber_VER == 0
				if(!IBIO_getInput(2))
				{
					delay_ms(2);
					if(!IBIO_getInput(2))
#else
				if(IBIO_getInput(2) || IBIO_getInput(3))
				{
					delay_ms(2);
					if(IBIO_getInput(2) || IBIO_getInput(3))
#endif
					{
						if(++SL_count >=5)
						{
							if(systemPara.Language == 0)
								ERR_INFO_NO_MATERIAL();
							else if(systemPara.Language == 1)
								EN_ERR_INFO_NO_MATERIAL();
							Alarm(0);
							SL_count = 0;
							return;
						}
						else
						{
							OLM_FIXED_step = 7;
							break;
						}
					}
				}
				else
				{
					SL_count = 0;
				}
			}
			OLM_FIXED_step ++;
			break;
		case 12:
			if(!systemPara.isCloseHCEnable)
			{
				if(IBIO_getOutput(1))
				{
					YL_Action(0);
					OLM_FIXED_step ++;
				}
			}
			else
				OLM_FIXED_step += 2;
			break;
		case 13:
			delay_ms(10);SL_ms_count ++;
			if(SL_ms_count > JLdelay_count)
			{SL_ms_count = 0;OLM_FIXED_step ++;}
			break;
		case 14:
			songliao_count ++;
			len = sprintf(str,"%d",songliao_count);
			GUI_showText(0x8000, "        ", 8);
			GUI_showText(0x8000, str, len);
#if CONF_Fiber_VER == 0
			EXO_sigON(2);
#endif
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("送料完成.", 9);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Feeding completed.", 18);

			if(systemPara.LackMaterral == 0 && systemPara.status == STATUS_ONLINE)
			{
				systemPara.LackMaterral = 1;
				if(systemPara.Language == 0)
					ERR_INFO_OVRE_MATERIAL();
				else if(systemPara.Language == 1)
					EN_ERR_INFO_OVRE_MATERIAL();
				systemPara.AlarmFlag = 5;
#if CONF_Fiber_VER == 0
				EXO_sigON(4);
#else
				EXO_sigOFF(3);
#endif
			}
			motor_ch[GIVEN_MOTOR].motorHadRun = 0;
			systemPara.givenOnceTriggerByUIorIO = 0;
			systemPara.givenOnceOK = 1;
			systemPara.RunStatus = 0;
			OLM_FIXED_step = 0;
			break;
	}
}

/**
 空跑模式
 */
void KongPaoMoShi(void *pv)
{
	char str[8]="0";
	uint16_t len;

	switch(KP_step)
	{
		case 0:
			KP_step ++;
			break;
		case 1:

			if(systemPara.Language == 0)
				GUI_mainMessageDisp("空跑开始...", 11);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("In aging...", 11);
			KP_step ++;
			break;
		case 2:
			delay_ms(10);ms_count ++;
			if(ms_count > 20)
			{ms_count = 0;KP_step ++;}
			break;
		case 3:
			if(systemPara.isCylinderSensorEnable)
			{
				if(SS_Action(0) == 0)
				{
					Alarm(0);
					return;
				}
			}
			KP_step ++;
			break;
		case 4:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("送料中......", 12);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Feeding...", 10);
			MC_motorMoveDistance(GIVEN_MOTOR, controlPara.givenSpeed, systemPara.MotorGivenDir,controlPara.givenDistance);
			MC_motorMoveDistance(SHOU_MOTOR, controlPara.shouSpeed, systemPara.MotorShouDir,controlPara.shouDistance);
			MC_motorMoveDistance(LET_MOTOR, controlPara.letSpeed, systemPara.MotorLetDir,controlPara.letDistance);
			KP_step ++;
			break;
		case 5:
			if((motor_ch[GIVEN_MOTOR].status == Motor_Stop) &&
			   (motor_ch[SHOU_MOTOR].status == Motor_Stop) &&
			   (motor_ch[LET_MOTOR].status == Motor_Stop))
			{
				KP_step ++;
			}
			break;
		case 6:
			if(systemPara.isCylinderSensorEnable)
			{
				if(SS_Action(1) == 0)
				{
					Alarm(0);
					return;
				}
			}
			KP_step ++;
			break;
		case 7:
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("送料结束.", 9);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Feeding completed.", 18);
			KP_step ++;
			break;
		case 8:
			delay_ms(10);ms_count ++;
			if(ms_count > 20)
			{ms_count = 0;KP_step ++;}
			break;
		case 9:
			YL_Action(0);
			KP_step ++;
			break;
		case 10:
			delay_ms(10);ms_count ++;
			if(ms_count > 20)
			{ms_count = 0;KP_step ++;}
			break;
		case 11:
			systemPara.HCOnceTriggerByUIorIO = 1;
			KP_step ++;
			break;
		case 12:
			if(systemPara.HCOnceOK == 1)
			{
				KP_step ++;
				delay_ms(100);
			}
			break;
		case 13:
			systemPara.RSTOnceTriggerByUIorIO = 1;
			KP_step ++;
			break;
		case 14:
			if(systemPara.RSTOnceOK == 1)
			{
				KP_step ++;
				delay_ms(100);
			}
			break;
		case 15:
			delay_ms(10);ms_count ++;
			if(ms_count > 20)
			{ms_count = 0;KP_step ++;}
			break;
		case 16:
			KP_step ++;
			break;
		case 17:
			KP_step = 25;
			break;
		case 25:
			YL_Action(1);
			KP_step ++;
			songliao_count ++;
			len = sprintf(str,"%d",songliao_count);
			GUI_showText(0x8000, "        ", 8);
			GUI_showText(0x8000, str, len);
			break;
		case 26:
			delay_ms(10);ms_count ++;
			if(ms_count > 20)
			{ms_count = 0;KP_step = 0;}
			break;
	}
}

/**
 * @brief 回原点动作
 */
uint8_t GoHome_step(void *pv)
{
	uint32_t offsetPulseCount;
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
			if(!IBIO_getInput(1))
			{

				if(systemPara.Language == 0)
					GUI_mainMessageDisp("避开原点...", 11);
				else if(systemPara.Language == 1)
					GUI_mainMessageDispIsolate("Avoid the origin...", 19);

				systemPara.RSTmode = 2;//传感模式
				MC_motorRunAndOffsetByEXTI(BO_MOTOR, controlPara.oriSpeed, systemPara.MotorBoDir, 0, 2000);
			}
			else
			{
				offsetPulseCount = controlPara.oriOffsetPulseCount;
				goHomeSpeed = controlPara.oriSpeed * 0.1f;
				if(goHomeSpeed < 1)
					goHomeSpeed = 1;
				offsetPulseCount = offsetPulseCount + MC_mm2pulse1(BO_MOTOR,offsetSpeed1[goHomeSpeed - 1]);
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("检测原点...", 11);
				else if(systemPara.Language == 1)
					GUI_mainMessageDispIsolate("Back to origin.", 15);
				systemPara.RSTmode = 1;//传感模式
				MC_motorRunAndOffsetByEXTI(BO_MOTOR, controlPara.oriSpeed, !systemPara.MotorBoDir,
											offsetPulseCount, RSTlimitPulseCount);
			}
			Gohome_step  ++;
			break;
		case 3:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				if(IBIO_getInput(1) && systemPara.RSTmode == 1)
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
				else if(!IBIO_getInput(1) && systemPara.RSTmode == 2)
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
			systemPara.HCOnceOK = 0;
			systemPara.HCTwoOK = 0;
			systemPara.RSTOnceOK = 1;
			Gohome_step = 0;
			return 1;
			break;
	}
}
void Changeover(void)
{
	if(systemPara.status == STATUS_FREERUN)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE3();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE3();
		return;
	}
	else if(systemPara.status == STATUS_ONLINE)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE1();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE1();
		return;
	}

	if(systemPara.RunStatus == 0)
	{
		EXO_sigOFF(1);
		EXO_sigOFF(2);
		EXO_sigOFF(3);
		EXO_sigOFF(4);
		EXO_sigOFF(5);
		GUI_runStatusDisp(0);
		Clear();

		JL_Action(1);
		YL_Action(1);
		GUI_switchPage(17);
		if(IBIO_getInput(1) == 0)
		{
			systemPara.RSTmode = 0;
			MC_motorMoveDistance(BO_MOTOR, CONF_BM_JOGSPEED, systemPara.MotorBoDir,
										controlPara.HLDistance);
		}
	}
	else
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE2();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE2();
	}

	while(motor_ch[BO_MOTOR].status != Motor_Stop);

	MC_cmd(1,0);
	MC_cmd(2,0);//换料结束后取消电机使能，可以推动
	MC_cmd(3,0);
	MC_cmd(4,0);
#if	CONF_COUNT_VER == 1
	//换料完成，重置送料计数值
	songliao_count = 0;
	GUI_showText(0x8000, "        ", 8);
	GUI_showText(0x8000, "0", 1);
#endif
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("换料动作完成.", 13);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("The material change action is completed!", 40);
	systemPara.ChangeOver = 1;
	USART1_sendBuf(sendBuffer[13],8);

}

void AddMetal_task(void)
{
	if(!systemPara.AddMetalOnceTriggerByUIorIO || replacenment_flag == 1)
	{
		systemPara.AddMetalOnceTriggerByUIorIO = 0;
		return;
	}

	if(systemPara.Initstatus)
	{
		StopModeUI(0);
	}

	if(systemPara.status == STATUS_ONLINE)
	{
		switchMode(0);
	}

	JL_Action(1);
	YL_Action(1);

	if(IBIO_getInput(1)==0 && recvBLDistance !=0)
	{
		ADJ_BL_step(systemPara.MotorBoDir,recvBLDistance/10);
	}
	while(motor_ch[BO_MOTOR].status != Motor_Stop);

	if(systemPara.Language == 0)
		GUI_mainMessageDisp("换料动作完成，请更换物料！", 26);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("The material change action is completed!", 40);
	USART1_sendBuf(sendBuffer[13],8);
	replacenment_flag = 1;
	systemPara.AddMetalOnceTriggerByUIorIO = 0;
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
			if(systemPara.AlarmFlag == 1)
			{
				//关闭使能，清除驱动器报警信号
				LV8731V_cmd(GIVEN_MOTOR, 0);
				delay_ms(100);
			}
			LV8731V_cmd(GIVEN_MOTOR, 1);

			if(systemPara.AlarmFlag == 2)
			{
				LV8731V_cmd(BO_MOTOR, 0);
				delay_ms(100);
			}
			LV8731V_cmd(BO_MOTOR, 1);
			
			systemPara.AlarmFlag = 0;
			
			if(systemPara.isAutoLetMetalEnable)
				LV8731V_cmd(LET_MOTOR, 1);
			if(systemPara.isUpShouEnable)
				LV8731V_cmd(SHOU_MOTOR, 1);

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
				USART1_sendBuf(sendBuffer[12],8);
			}
			delay_ms(100);
			Init_step ++;
			break;
		case 2:
			JL_Action(0);
			Init_step ++;
			break;
		case 3:
			if(systemPara.isCylinderSensorEnable || (!(IBIO_getOutput(3)) || !(IBIO_getInput(10))) )
			{
				if(SS_Action(1) == 0 )
				{
					Alarm(0);
					return;
				}
			}
			Init_step ++;
			break;
		case 4:
			if(IBIO_getOutput(1))
			{
				YL_Action(0);
				Init_step ++;
			}
			else
				Init_step += 2;
			break;
		case 5:
			delay_ms(10);ms_count ++;
			if(ms_count > 15)
			{ms_count = 0;Init_step ++;}
			break;
		case 6://复位
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("复位中...", 9);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Back to origin...", 17);
			systemPara.doGohome = 1;
			Init_step ++;
			break;
		case 7://
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
		case 8:
			Init_step ++;
			break;
		case 9:
			if(systemPara.logginStatus > 10 && systemPara.status == STATUS_FREERUN)
			{	 // 登录状态下，空跑模式初始化，禁止修改数据
				 GUI_switchPage(32);
			}
			if(systemPara.Language == 0)
				GUI_mainMessageDisp("初始化完成！", 11);
			else if(systemPara.Language == 1)
				GUI_mainMessageDisp("Initialization is complete!", 27);
			EXO_sigON(3);
			EXO_sigON(5);
			USLflag = 0;
			UpslTask_handler(0);
#if	CONF_LET_VER == 0
			DSLflag = 0;
			DownslTask_handler(0);
#else
			flTask_handler(0);//放松料带
#endif

#if CONF_BLRST_VER == 1
		if(systemPara.status == STATUS_ONLINE)
		{
			systemPara.HCOnceTriggerByUIorIO = 1;
		}
#endif
			GUI_runStatusDisp(1);
			systemPara.doInit = 0;
			systemPara.givenOnceOK = 0;
			systemPara.HCOnceOK = 0;
			systemPara.RSTOnceOK = 1;
			systemPara.isWorkTaskRun = 1;
			systemPara.RunStatus = 0;
			systemPara.Initstatus = 1;
			systemPara.LackMaterral = 1; // 默认有料
			systemPara.ChangeOver = 0;
			systemPara.AdjustGivenOK =0;
			systemPara.AdjustStripOK =0;
			Init_step = 0;
			break;
	}
}
/**
 上收料动作
 */
void UpslTask_handler(void *pv)
{
	static uint16_t delay_count = 0;

	if(motor_ch[UP_MOTOR].status != Motor_Stop)
		return;

	if (systemPara.isUpShouEnable)
	{
		if (systemPara.isUpShouSensorEnable == 0)
		{
			systemPara.UPSLmode = 0;
			MC_motorMoveDistance(UP_MOTOR,controlPara.shouSpeed,systemPara.MotorShouDir,controlPara.shouDistance);
		}
		else if (systemPara.isUpShouSensorEnable == 1)
		{
			if(!IBIO_getInput(5) && USLflag == 0)
			{
				USLflag = 1;
				systemPara.UPSLmode = 1;
				MC_motorMoveDistance(UP_MOTOR,controlPara.shouSpeed,systemPara.MotorShouDir,controlPara.shouDistance);
			}
			if(IBIO_getInput(5))
			{
				USLflag = 0;
			}
		}
		else if (systemPara.isUpShouSensorEnable == 2)
		{
			if(IBIO_getInput(6) && USLflag == 0)
			{
				USLflag = 1;
				delay_count = 0;
				systemPara.UPSLmode = 1;
				MC_motorMoveDistance(UP_MOTOR,controlPara.shouSpeed,systemPara.MotorShouDir,controlPara.shouDistance);
			}
			if(IBIO_getInput(6))
			{
				if(++delay_count <= 500)
					delay_ms(1);
				else
				{
					delay_count = 0;
					USLflag = 0;
				}
			}
		}
	}
}
#if CONF_LET_VER == 1
void flTask_handler(void *pv)
{
	if(systemPara.isAutoLetMetalEnable)
	{
		if(motor_ch[LET_MOTOR].status != Motor_Stop)
			return;

		if(!IBIO_getInput(7) && FLflag == 0)
		{
			FLflag = 1;
			systemPara.FLmode = 1;
			MC_motorMoveDistance(LET_MOTOR,controlPara.letSpeed,systemPara.MotorLetDir,controlPara.letDistance);
		}
		if(IBIO_getInput(7))
		{
			FLflag = 0;
		}
	}
}
#else
void DownslTask_handler(void *pv)
{
	static uint16_t delay_count = 0;

	if(motor_ch[LET_MOTOR].status != Motor_Stop)
		return;

	if (systemPara.isDownShouEnable)
	{
		if(systemPara.isDownShouSensorEnable == 0)
		{
			systemPara.FLmode = 0;
			MC_motorMoveDistance(LET_MOTOR, controlPara.letSpeed,systemPara.MotorLetDir, controlPara.letDistance);
		}
		else if(systemPara.isDownShouSensorEnable == 1)
		{

			if(IBIO_getInput(7) == 0 && DSLflag == 0)
			{
				DSLflag = 1;
				systemPara.FLmode = 1;
				MC_motorMoveDistance(LET_MOTOR, controlPara.letSpeed,systemPara.MotorLetDir, controlPara.letDistance);
			}
			if(IBIO_getInput(7) == 1)
			{
				DSLflag = 0;
			}
		}
		else if(systemPara.isDownShouSensorEnable == 2)
		{
			if(IBIO_getInput(8) && DSLflag == 0)
			{
				delay_count = 0;
				DSLflag = 1;
				systemPara.FLmode = 1;
				MC_motorMoveDistance(LET_MOTOR, controlPara.letSpeed,systemPara.MotorLetDir, controlPara.letDistance);
			}
			if(IBIO_getInput(8) == 1)
			{
				if(++delay_count <= 2000)
					delay_ms(1);
				else
				{
					delay_count = 0;
					DSLflag = 0;
				}
			}
		}
	}
}
#endif

//防夹检测
void Platform_signal_detection(void)
{
	switch(detection)
	{
		case  0:
		if(IBIO_getInput(11) == 0 &&  systemPara.Platform_flagH == 0 )
		{
			delay_ms(5);
			if(IBIO_getInput(11) == 0)
			{
				systemPara.Platform_flagH = 1;
				systemPara.Platform_flagL = 1;
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("报警提示：剥刀复位检测到物体！", 30);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Alarm info: Stripping knife reset detected object!", 51);
				detection ++;
			}
		}
		if(IBIO_getInput(11) == 1)
		{
			systemPara.Platform_flagH = 0;
			return ;
		}
		case 1:
			if(systemPara.Platform_flagL == 1)
			{
				systemPara.Platform_flagL = 0;
				systemPara.RSTmode = 0;
				MC_motorMoveDistance(BO_MOTOR, controlPara.boSpeed,systemPara.MotorBoDir,50);
				detection ++;
			}
			break;
		case 2:
			if(motor_ch[BO_MOTOR].status == Motor_Stop)
			{
				detection ++;
			}
			break;
		case 3:
			LV8731V_cmd(BO_MOTOR, 0);
			Alarm(0);
			detection = 0;
			break;
	}
}
void GivenMotorExceedPulseLimitAlert(void)
{
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("报警信息：送料行程走完，未检测到有料！", 38);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("Alarm info:feeding ovre,no material is detected!", 48);
	systemPara.AlarmFlag = 8;
#if CONF_Fiber_VER == 0
	EXO_sigON(4);
#endif
	EXO_sigOFF(3);
	MC_motorStopAll();
	GUI_runStatusDisp(0);
	Clear();
}


void LetMotorExceedPulseLimitAlert(void)
{
	MC_motorStop(LET_MOTOR);
	MC_motorStop(GIVEN_MOTOR);
	GUI_runStatusDisp(0);
	systemPara.isWorkTaskRun = 0;
	systemPara.Initstatus = 0;
	systemPara.RunStatus = 0;
}

/**
 @brief UI控制切换紧急停止状态
 */
void EnterStopModeUI(void *pv)
{
	MC_motorStopAllEmergency();
    GUI_runStatusDisp(2);

	if(systemPara.Language == 0)
	    GUI_mainMessageDisp("UI紧急停止动作！", 16);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("UI E-stop action!", 17);
	EXO_sigOFF(1);
	EXO_sigOFF(2);
	EXO_sigOFF(3);
	EXO_sigOFF(4);
	EXO_sigOFF(5);
	Clear();
//取消重启
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
	USART1_sendBuf(sendBuffer[1],8);
    MC_motorStopAll();
	MC_cmd(0,0);
	MC_cmd(1,0);
	MC_cmd(2,0);
	MC_cmd(3,0);
    GUI_runStatusDisp(0);
    if(systemPara.isInEmergencyStopEnable)
    {
		if(IBIO_getInput(12))
		{
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
	Clear();
}

/**
 @brief UI停止动作
 */
void StopModeUI(void *pv)
{
	if(systemPara.logginStatus > 10 && systemPara.status == STATUS_FREERUN)
	{	//登录状态下，空跑运行中点击停止后，可以修改数据
		GUI_switchPage(0);
	}
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("UI停止动作！", 16);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("UI stop action.", 15);

	MC_motorStopAll();
	USART1_sendBuf(sendBuffer[1],8);
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
 * * @param -no- 1 剥料完成, 2 送料完成, 3 初始化完成信号, 4 异常报警, 5复位完成
 */
void EXO_sigON(uint8_t no)
{
	if(systemPara.status != STATUS_ONLINE)
		return;
	switch(no)
	{
	case 1:
		if(EXIO_getOutput(2))
		{
			EXIO_setOutput(2,0);
			GUI_setOutputSignalColorDisp(10,0);
			USART1_sendBuf(sendBuffer[11],8);
		}
		break;
	case 2:
		if(EXIO_getOutput(3))
		{
			EXIO_setOutput(3, 0);
			GUI_setOutputSignalColorDisp(11,0);
			USART1_sendBuf(sendBuffer[9],8);
		}
		break;
	case 3:
		if(EXIO_getOutput(0))
		{
			EXIO_setOutput(0,0);
			GUI_setOutputSignalColorDisp(8,0);
			USART1_sendBuf(sendBuffer[5],8);
		}
		break;
	case 4:
		if(EXIO_getOutput(1))
		{
			EXIO_setOutput(1,0);
			GUI_setOutputSignalColorDisp(9,0);
			switch(systemPara.AlarmFlag)
			{
				case 1:sendBuffer[23][5] = 0x01;break;
				case 2:sendBuffer[23][5] = 0x02;break;
				case 3:sendBuffer[23][5] = 0x03;break;
				case 4:sendBuffer[23][5] = 0x04;break;
				case 5:sendBuffer[23][5] = 0x05;break;
				case 6:sendBuffer[23][5] = 0x06;break;
				case 7:sendBuffer[23][5] = 0x07;break;
				case 8:sendBuffer[23][5] = 0x08;break;
				case 9:sendBuffer[23][5] = 0x09;break;
				case 10:sendBuffer[23][5] = 0x0a;break;
				default:	break;
			}
//			systemPara.AlarmFlag = 0;
			USART1_sendBuf(sendBuffer[23],8);
		}
		break;
	case 5:
//		EXIO_setOutput(4,0);
//		GUI_setOutputSignalColorDisp(12,0);
		break;
	}
}

/**
 * 关闭输出信号
 * * @param -no- 1 剥料完成, 2 送料完成, 3 初始化完成信号, 4 异常报警, 5复位完成
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
			USART1_sendBuf(sendBuffer[10],8);
		}
		break;
	case 2:
		if(!EXIO_getOutput(3))
		{
			EXIO_setOutput(3, 1);
			GUI_setOutputSignalColorDisp(11,1);
			USART1_sendBuf(sendBuffer[8],8);
		}
		break;
	case 3:
		if(!EXIO_getOutput(0))
		{
			EXIO_setOutput(0,1);
			GUI_setOutputSignalColorDisp(8,1);
			USART1_sendBuf(sendBuffer[4],8);
		}
		break;
	case 4:
		if(!EXIO_getOutput(1))
		{
			EXIO_setOutput(1,1);
			GUI_setOutputSignalColorDisp(9,1);
			USART1_sendBuf(sendBuffer[22],8);
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
	delay_ms(20);
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
#if CONF_Fiber_VER == 0
		EXO_sigON(4);
#endif
		EXO_sigOFF(1);
		EXO_sigOFF(2);
		EXO_sigOFF(3);
	}
	MC_motorStopAll();
	GUI_runStatusDisp(0);
	Clear();
}

void YL_Action(uint8_t Action)
{
	if(Action == 0)//压料//气缸气管反向接
	{
		if(systemPara.Language == 0)
			MAIN_INFO_YL_CYL_OUT();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_YL_CYL_OUT();

		IBIO_setOutput(1, 0);
		GUI_setOutputSignalColorDisp(0,0);
		GUI_mainMessageDispIsolate("压料气缸压紧.", 15);
		USART1_sendBuf(sendBuffer[17],8);
	}
	else
	{
		if(systemPara.Language == 0)
			MAIN_INFO_YL_CYL_IN();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_YL_CYL_IN();

		IBIO_setOutput(1, 1);
		GUI_setOutputSignalColorDisp(0,1);
		GUI_mainMessageDispIsolate("压料气缸松开.", 15);
		USART1_sendBuf(sendBuffer[16],8);
	}
}

uint8_t SS_Action(uint8_t Action)
{
	if(Action == 0)//传感器气缸
	{
		if(systemPara.Language == 0)
			MAIN_INFO_USL_CYL_OUT();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_USL_CYL_OUT();

		IBIO_setOutput(3, 0);
		GUI_setOutputSignalColorDisp(2,0);
		if(GetIOLevel(10,0,2000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_USL_CYL_OUT_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_USL_CYL_OUT_TIMEOUT();
//			Alarm(0);
			return 0;
		}
		GUI_mainMessageDispIsolate("传感器气缸伸出.", 15);
		USART1_sendBuf(sendBuffer[17],8);
	}
	else
	{

		if(systemPara.Language == 0)
			MAIN_INFO_USL_CYL_IN();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_USL_CYL_IN();

		IBIO_setOutput(3, 1);
		GUI_setOutputSignalColorDisp(2,1);
		if(GetIOLevel(9,0,2000) == 0)
		{
			if(systemPara.Language == 0)
				ERR_INFO_USL_CYL_IN_TIMEOUT();
			else if(systemPara.Language == 1)
				EN_ERR_INFO_USL_CYL_IN_TIMEOUT();
//			Alarm(0);
			return 0;
		}
		GUI_mainMessageDispIsolate("传感器气缸缩回.", 15);
		USART1_sendBuf(sendBuffer[16],8);
	}
	return 1;
}

void JL_Action(uint8_t Action)
{
	if(Action == 1)//夹料//气缸气管反向接
	{
		if(systemPara.Language == 0)
			MAIN_INFO_JL_CYL_IN();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_JL_CYL_IN();

		IBIO_setOutput(2, 0);
		GUI_setOutputSignalColorDisp(1,1);
		GUI_mainMessageDispIsolate("夹料气缸松开.", 15);
		USART1_sendBuf(sendBuffer[17],8);
	}
	else
	{
		if(systemPara.Language == 0)
			MAIN_INFO_JL_CYL_IN();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_JL_CYL_IN();

		IBIO_setOutput(2, 1);
		GUI_setOutputSignalColorDisp(1,0);
		GUI_mainMessageDispIsolate("夹料气缸夹紧.", 15);
		USART1_sendBuf(sendBuffer[16],8);
	}
}

void Clear(void)
{
    systemPara.isWorkTaskRun = 0;
    systemPara.Initstatus = 0;
    systemPara.LackMaterral = 1; // 默认有料
    systemPara.doInit = 0;
    systemPara.doGohome = 0;
	systemPara.givenOnceTriggerByUIorIO = 0;
	systemPara.BLOnceTriggerByUIorIO = 0;
	systemPara.HCOnceTriggerByUIorIO = 0;
	systemPara.RSTOnceTriggerByUIorIO = 0;
	systemPara.STOPOnceTriggerByUIorIO = 0;
	systemPara.BLOnceOK = 0;
	systemPara.HCTwoOK = 0;
	systemPara.givenOnceOK = 0;
	systemPara.RSTOnceOK = 0;
	systemPara.RunStatus = 0;
	systemPara.ChangeOver = 0;
	systemPara.AdjustGivenOK =0;
	systemPara.AdjustStripOK =0;
	KP_step = 0;  // 空跑步骤
	OLM_step = 0;// 联机模式
	OLM_DOWN_step = 0;// 下光纤
	OLM_UP_step = 0;// 上光纤
	OLM_FIXED_step = 0;
	Gohome_step = 0;
	Init_step = 0;
	IO_HC_step = 0;
	IO_RST_step = 0;
	IO_BL_step = 0;
	ms_count = 0;
	SL_ms_count = 0;
	BL_ms_count = 0;
	RST_ms_count = 0;
	replacenment_flag = 0;
	SL_count = 0;
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

		USART1_sendBuf(sendBuffer[25],8);

		GUI_switchPage(32);
		if(IBIO_getInput(1) == 0 && systemPara.Initstatus == 1)
		{
			EXO_sigON(3);
			EXO_sigON(5);
#if CONF_Fiber_VER == 0
			if(systemPara.givenOnceOK == 1)
				EXO_sigON(2);
#endif
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
		systemPara.givenOnceTriggerByUIorIO = 0;
		systemPara.status = STATUS_AUTO;
		AT24CXX_WriteOneByte(STORAGE_WORKMODE_ADDR, systemPara.status);
		GUI_switchModeDisp(systemPara.status);

		if(systemPara.Language == 0)
			MAIN_INFO_SW2DEBUG();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_SW2DEBUG();

		USART1_sendBuf(sendBuffer[24],8);
	}
}

void ADJ_SL_step(uint8_t dir, uint16_t step)
{
	if(step == 0)
		MC_GMOffsetRunDistance(dir,1);
	else
		MC_GMOffsetRunDistance(dir,step/10);

	while(motor_ch[GIVEN_MOTOR].status != Motor_Stop);

	if(dir)
	{
		USART1_sendBuf(sendBuffer[14],8);
	}
	else
	{
		USART1_sendBuf(sendBuffer[15],8);
	}
	systemPara.AdjustGivenOK = 1;
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("送料微调完成.", 19);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("Feed fine-tuning completed.", 26);
}

void ADJ_BL_step(uint8_t dir, uint16_t step)
{
	if(step == 0)
		MC_BMOffsetRunDistance(dir,1);
	else
		MC_BMOffsetRunDistance(dir,step/10);

	while(motor_ch[BO_MOTOR].status != Motor_Stop);

	if(!dir)
	{
		USART1_sendBuf(sendBuffer[16],8);
	}
	else
	{
		USART1_sendBuf(sendBuffer[17],8);
	}
	systemPara.AdjustStripOK = 1;
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("剥料微调完成.", 19);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("Stripping fine-tuning completed.", 32);
}


void M0driveALM(void)
{
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("报警信息：送料驱动器报警！", 26);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("Alarm info:Feeding driver alarm!", 32);

	LV8731V_cmd(0, 0);
	systemPara.AlarmFlag = 1;
	Alarm(0);
}

void M2driveALM(void)
{
	if(systemPara.Language == 0)
		GUI_mainMessageDisp("报警信息：剥料驱动器报警！", 26);
	else if(systemPara.Language == 1)
		GUI_mainMessageDisp("Alarm info: Stripping driver alarm!", 35);

	LV8731V_cmd(2, 0);
	systemPara.AlarmFlag = 2;
	Alarm(0);
}

uint8_t EmergencyStop(void)
{
	if(systemPara.isInEmergencyStopEnable)
	{
		if(IBIO_getInput(12))
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
