#include "inputscan/inputscan.h"
#include "dwin/gui.h"
#include "inboardio/inboardio.h"
#include "exio/exio.h"
#include "paraManager/paramanager.h"
#include "eventHandler/eventHandler.h"
#include "motorctrl/motorctrl.h"
#include "delay/delay.h"
#include "userfunc/userfunc.h"
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
// 1. 定义静态变量来存储上一次的电平状态
// 总信号数: 12 (IBIO) + 8 (EXIO) = 20个
#define TOTAL_SIGNALS 20
static uint8_t s_oldLevel[TOTAL_SIGNALS] = {0xFF}; // 初始化为0xFF确保第一次全部更新

void IN_scan(void *pv)
{
    uint8_t i;
    uint8_t level;
    uint8_t index = 0; // 统一的索引计数器

    // 扫描12个IBIO输入 (地址: 0x7600 - 0x760B)
    for (i = 1; i <= 12; i++)
    {
        level = IBIO_getInput(i);
        // 只有电平发生变化时才发送
        if(s_oldLevel[index] != level)
        {
            s_oldLevel[index] = level; // 更新缓存
            GUI_sendWord(0x7600 + i - 1, 0, level);
        }
        index++; // 索引递增
    }

    // 扫描前4个EXIO输入 (地址: 0x760C - 0x760F)
    for (i = 0; i < 4; i++)
    {
        level = EXIO_getInput(i);
        if(s_oldLevel[index] != level)
        {
            s_oldLevel[index] = level;
            GUI_sendWord(0x760c + i, 0, level);
        }
        index++;
    }

    // 扫描后4个EXIO输入 (地址: 0x7822 - 0x7825)
    for (i = 4; i < 8; i++)
    {
        level = EXIO_getInput(i);
        if(s_oldLevel[index] != level)
        {
            s_oldLevel[index] = level;
            GUI_sendWord(0x7822 + i - 4, 0, level);
        }
        index++;
    }
}
//void IN_scan(void *pv)
//{
//	uint8_t i;
//	uint8_t level;
//
//	for (i = 1; i <= 12; i++)
//	{
//		level = IBIO_getInput(i);
//		GUI_sendWord(0x7600 + i - 1, 0, level);
//	}
//
//	for (i = 0; i < 4; i++)
//	{
//		level = EXIO_getInput(i);
//		GUI_sendWord(0x760c + i, 0, level);
//	}
//
//	for (i = 4; i < 8; i++)
//	{
//		level = EXIO_getInput(i);
//		GUI_sendWord(0x7822 + i - 4, 0, level);
//	}
//
//}

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

	//电机停止状态，实时检测有料信号
	if(systemPara.status == STATUS_ONLINE)
	{
		if(motor_ch[GIVEN_MOTOR].status == Motor_Stop)
		{
#if CONF_Fiber_VER == 0
			//下光纤模式
			if(systemPara.sensorChosen == 1)
			{
				if(IBIO_getInput(2) == 0 )
				{
					delay_ms(2);
					if(IBIO_getInput(2) == 0 )
					{
						EXO_sigON(2);
					}
				}

				if(IBIO_getInput(2) == 1 )
				{
					EXO_sigOFF(2);
				}
			}
			else//上光纤检测
			{
				if(systemPara.isCylinderSensorEnable && IBIO_getInput(9) == 1 )//伸缩气缸不在原点
				{
					EXO_sigOFF(2);
				}
				if(systemPara.isUpSensorIsRealtime && !systemPara.isCylinderSensorEnable)
				{
					if(IBIO_getInput(3) == 0 )
					{
						delay_ms(2);
						if(IBIO_getInput(3) == 0 )
						{
							EXO_sigON(2);
						}
					}

					if(IBIO_getInput(3) == 1 )
					{
						EXO_sigOFF(2);
					}
				}
			}
#else
				if(IBIO_getInput(2) == 0)
				{
					delay_ms(2);
					if(IBIO_getInput(2) == 0)
					{
						EXO_sigON(2);
					}
				}
				if(IBIO_getInput(2) == 1)
				{
					EXO_sigOFF(2);
				}
				if( IBIO_getInput(3) == 0)
				{
					delay_ms(2);
					if(IBIO_getInput(3) == 0)
					{
						EXO_sigON(4);
					}
				}
				if(IBIO_getInput(3) == 1)
				{
					EXO_sigOFF(4);
				}
#endif
		}
	}
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






