#include "inputscan/inputscan.h"
#include "dwin/gui.h"
#include "inboardio/inboardio.h"
#include "exio/exio.h"
#include "paraManager/paramanager.h"
#include "eventHandler/eventHandler.h"

/**
 @brief 更新输入信号页面UI控制
 @param -cmd- 0-不更新, 1-更新
*/
void IN_refreshUICmd(uint8_t cmd)
{
	switch(cmd)
	{
	case 1:
		TIM5->CR1 |= TIM_CR1_CEN;
		break;

	case 0:
		TIM5->CR1 &= ~TIM_CR1_CEN;
		break;
	}
}

void IN_scan(void *pv)
{
	uint8_t i;
	uint8_t level;

	for (i = 1; i <= 12; i++)
	{
		level = IBIO_getInput(i);
		GUI_sendWord(0x7600 + i - 1, 0, level);
	}

	for (i = 0; i < 4; i++)
	{
		level = EXIO_getInput(i);
		GUI_sendWord(0x760c + i, 0, level);
	}

	for (i = 4; i < 8; i++)
	{
		level = EXIO_getInput(i);
		GUI_sendWord(0x7822 + i - 4, 0, level);
	}

}

void IN_checkMetal(void)
{
	uint8_t method;

	if(systemPara.isWorkTaskRun == 0)
	{
		return;
	}

	if(!systemPara.refreshMatalUI)
	{
		return;
	}

	method = IBIO_getInput(1);
	EXIO_setOutput(2,method);
	GUI_setOutputSignalColorDisp(10, method);

	method = IBIO_getInput(2);
	EXIO_setOutput(3, method);
	GUI_setOutputSignalColorDisp(11, method);
}

/**
 扫描INPUT_1,INPUT_2,INPUT_3的电平, 并启动对应线程
*/
void IN_exTrigger(void *pv)
{
    uint8_t method;
    uint8_t input1_level_old=1,input1_level_current;
    while(1)
    {
        if(systemPara.sensorChosen == 1)
        {
            method = 2;
        }
        else
        {
            method = 3;
        }
        
        method = IBIO_getInput(method);
        EXIO_setOutput(4,method);
        GUI_sendWord(0x766B,0,method);
        
        if(systemPara.status!=STATUS_FREERUN)
        {
            input1_level_current = EXIO_getInput(1);
            if((input1_level_current==0)&&(input1_level_old==1))
            {
                startupInit(0);
            }
            input1_level_old = input1_level_current;
            
            if(systemPara.isWorkTaskRun == 1)
            {
                if(EXIO_getInput(2) == 0)
                {
                    start_mainsongliao(0);
                }
                else if(EXIO_getInput(3) == 0)
                {
                    method = 5;
                    start_mainboliao((void*)&method);
                }
            }
            
        }
    }
    
}
