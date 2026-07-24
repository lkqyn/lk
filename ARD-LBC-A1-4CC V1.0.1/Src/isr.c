/*
 * isr.c
 *
 *  Created on: 2020年4月2日
 *      Author: ylj
 */

#include "stm32f1xx_hal.h"
#include "motorctrl/motorctrl.h"
#include "pwm1/pwm1.h"
#include "pwm3/pwm3.h"
#include "led/led.h"
#include "queue/queue.h"
#include "eventHandler/eventHandler.h"
#include "paraManager/paraManager.h"
#include "dwin/gui.h"
#include "inputscan/inputscan.h"
#include "userfunc/userfunc.h"
#include "inboardio/inboardio.h"
#include "dwin/gui.h"

void EXTI0_IRQHandler(void)
{
	//停止触发EXIO-3
	//GUI_mainMessageDispIsolate("1", 1);
	if(systemPara.status == STATUS_ONLINE)
	{
//		systemPara.STOPOnceTriggerByUIorIO = 1;
	}
	//systemPara.STOPOnceTriggerByUIorIO = 1;
	EXTI->PR |= EXTI_PR_PR0;
}

void EXTI1_IRQHandler(void)
{
	//初始化触发EXIO-2
	//GUI_mainMessageDispIsolate("0", 1);
	if(systemPara.status == STATUS_ONLINE)
	{
		if(systemPara.RunStatus == 0)
		{
//			if(systemPara.Err == 1)
//			{
//				systemPara.Err = 0;
//				EXO_sigOFF(4);//清除报警信号
//			}
//			if(systemPara.Initstatus == 0)
//			{
//				EXO_sigOFF(4);//清除报警信号
//				systemPara.doInit = 1;
//			}
		}
	}
	EXTI->PR |= EXTI_PR_PR1;
}

void EXTI2_IRQHandler(void)
{
	//剥料触发EXIO-1
	//GUI_mainMessageDispIsolate("2", 1);
    if(systemPara.status == STATUS_ONLINE)
	{
    	if(systemPara.isWorkTaskRun == 1 && systemPara.RunStatus == 0)
		{
    		//systemPara.CUTOnceTriggerByUIorIO = 1;
		}
	}
	EXTI->PR |= EXTI_PR_PR2;
}

void EXTI3_IRQHandler(void)
{
	//送料触发 EXIO-0

    if(systemPara.status == STATUS_ONLINE)
	{
    	if(systemPara.isWorkTaskRun == 1 && systemPara.RunStatus == 0)
		{
			//systemPara.givenOnceTriggerByUIorIO = 1;
		}
	}

	EXTI->PR |= EXTI_PR_PR3;
}

void EXTI4_IRQHandler(void)
{
	// 送料动作, 传感器检测到后, 开始跑偏移

//	motor_ch[GIVEN_MOTOR].beginRunOffset = 1;
//	motor_ch[GIVEN_MOTOR].offsetPulseCount += PWM1_pulseCount;

	EXTI->PR |= EXTI_PR_PR4;

	//IBIO_INPUT2_EXTI_DISABLE();

}

void EXTI15_10_IRQHandler(void)
{


    // 閫佹枡鍔ㄤ綔, 涓婂厜绾?
//    if((EXTI->PR & EXTI_PR_PR10) == EXTI_PR_PR10)
//    {
//        EXTI->PR |= EXTI_PR_PR10;
//
//        motor_ch[GIVEN_MOTOR].beginRunOffset = 1;
//		motor_ch[GIVEN_MOTOR].offsetPulseCount += PWM1_pulseCount;
//
//
//        IBIO_INPUT3_EXTI_DISABLE();
//    }

//    //if((EXTI->PR & EXTI_PR_PR13) == EXTI_PR_PR13)
//    if(EXTI->PR & EXTI_PR_PR13)
//    {
//    		//收料停止
//    	//GUI_mainMessageDispIsolate("6", 1);
//    	if(motor_ch[DOWN_MOTOR].status != Motor_Stop)
//    		MC_motorStop(DOWN_MOTOR);
//        EXTI->PR |= EXTI_PR_PR13;
//    }
//
//	//if ((EXTI->PR & EXTI_PR_PR14) == EXTI_PR_PR14)
//    if(EXTI->PR & EXTI_PR_PR14)
//	{
//		//收料信号
//    	//GUI_mainMessageDispIsolate("5", 1);
//    	if(motor_ch[DOWN_MOTOR].status == Motor_Stop)
//    		slDownTask_handler(0);
//		EXTI->PR |= EXTI_PR_PR14;
//	}

	if (EXTI->PR & EXTI_PR_PR10)
	{
			//放料启动
		//GUI_mainMessageDispIsolate("7", 1);
//		if(motor_ch[LET_MOTOR].status == Motor_Stop)
//			flTask_handler(0);
		EXTI->PR |= EXTI_PR_PR10;
	}
	//放料停止
    if(EXTI->PR & EXTI_PR_PR15)
    {
    	//GUI_mainMessageDispIsolate("8", 1);
//    	if(motor_ch[LET_MOTOR].status != Motor_Stop)
//    		MC_motorStop(LET_MOTOR);
    	EXTI->PR |= EXTI_PR_PR15;
    }

//    //if ((EXTI->PR & EXTI_PR_PR10) == EXTI_PR_PR10)
//	if(EXTI->PR & EXTI_PR_PR10)
//	{
//	//上收料信号
//		//GUI_mainMessageDispIsolate("3", 1);
//		if(motor_ch[UP_MOTOR].status == Motor_Stop)
//			slUpTask_handler(0);
//		EXTI->PR |= EXTI_PR_PR10;
//	}
//
//	if(EXTI->PR & EXTI_PR_PR15)
//	{
//			//上收料停止
//		//GUI_mainMessageDispIsolate("4", 1);
//		if(motor_ch[UP_MOTOR].status != Motor_Stop)
//			MC_motorStop(UP_MOTOR);
//
//		EXTI->PR |= EXTI_PR_PR15;
//	}


}

struct CommData recvData;

void USART2_IRQHandler(void)
{
	static uint8_t count = 0;

	recvData.data[count] = USART2->DR;

	count ++;

	if(count == 1)
	{
		if(recvData.data[0] != 0x5a)
		{
			count = 0;
			return;
		}
	}

	else if(count == 2)
	{

		if(!((recvData.data[0] == 0x5a) && (recvData.data[1] == 0xa5)))
		{
			count = 0;
			return;
		}
	}

	else if((count > 2)&&((count-3) == recvData.data[2]))
	{
		recvData.length = count;
		count = 0;

		if((recvData.data[3]==0x83)&&(recvData.data[4]==0x60)&&(recvData.data[5]==0))
		{//停止或紧急停止
			eventCallback[0](&(recvData.data[8]));
		}
		else
		{
			QUEUE_add(&recvQueue,recvData);
		}
	}

	if(count > 30)
	{
		count = 0;
	}

	USART2->SR &= ~USART_SR_RXNE;
}

void SysTick_Handler(void)
{
	static uint32_t count;
	static uint32_t logout_count;
	static uint32_t uiFreshCount;



	if(++count > 500)//500
	{
		count = 0;
		LED_task(0);
	}

	if(++uiFreshCount > 50)
	{
		uiFreshCount = 0;

		systemPara.refreshMatalUI = 1;
	}

	if(systemPara.logginStatus > 10)
	{
		if(++logout_count >= 1000)//1000
		{
			logout_count = 0;
			if(++logout_count1 >= 120)//120S
			{
				logout_count1 =0;
				systemPara.logginStatus = 0;
				GUI_switchPage(32);
				GUI_showText(LOGGINSTATE_DISP_ADDR,"用户登录",8);
				GUI_mainMessageDisp("系统自动退出成功.", 17);
			}
		}
	}

}

void TIM5_IRQHandler(void)
{
	if(TIM5->SR & TIM_SR_UIF)
	{
		IN_scan(0);

		TIM5->SR &= ~TIM_SR_UIF;
	}
}
