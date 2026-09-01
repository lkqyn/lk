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

#include "sysconfig/sysconfig.h"
#include "easyModbus/easyModbus.h"


struct CommData recvData;

void USART2_IRQHandler(void)
{
	static uint8_t count = 0;

	USART2->SR &= ~USART_SR_RXNE;

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

	//按下换料完成按钮
	if(changeoverflag)
	{
		ChangeoverTime ++;
	}
	else
	{
		ChangeoverTime = 0;
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

void USART1_IRQHandler(void)
{

	static uint8_t count=0;

	USART1->SR &= ~USART_SR_RXNE;

	recvBuffer[count] = USART1->DR;
	count ++;

	if(recvBuffer[0] != CONF_SLAVE_ADDR)
	{
		count = 0;
		return;
	}

//	if((recvBuffer[1] == 6) && (count >= 8))
//	{
//		count = 0;
//		//GUI_mainMessageDisp("06", 2);
//		gNewComRecv = 1;
//	}

	if((recvBuffer[count-1] == 0xff) &&(recvBuffer[count-2]==0xff))
	{
		count = 0;
		gNewComRecv = 1;
	}
}
