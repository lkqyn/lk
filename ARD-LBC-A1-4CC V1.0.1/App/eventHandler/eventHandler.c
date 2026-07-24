/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * All GUI button's events must write values to address between 0x6000 and 0x6999.
 * This file offers a task to check address between 0x6000 and 0x6999 which is not zero,
 * then run the function.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-05     ylj       the first version
 */

#include "eventHandler/eventHandler.h"
#include "paraManager/paraManager.h"
#include "dwin/gui.h"
#include "motorctrl/motorctrl.h"
#include "userfunc/userfunc.h"
#include "inputscan/inputscan.h"
#include "storage/storage.h"
#include "exio/exio.h"
#include "at24cxx/at24cxx.h"
#include "delay/delay.h"
#include "inboardio/inboardio.h"
#include "pwm1/pwm1.h"
//#include "extrigger/extrigger.h"
//#include "signal_event/signal_event.h"


void checkPassword(void *pv);
void runStatusControl(void *pv);
void fillUserName(void *pv);
void returnPage(void *pv);
void gotoFeederParameterPage(void *pv);
void logout(void *pv);
void changePassword(void *pv);
void userManagePage(void *pv);
void jump2MotorParameterPage(void *pv);
void jump2MainPage(void *pv);
void jump2debugPage(void *pv);
void jump2inputPage(void *pv);
void jump2outputPage(void *pv);
void jump2loginPage(void *pv);
void startupInit(void *pv);
void selectOnlineMode(void *pv);
void selectAutoMode(void *pv);
void selectFreeRunMode(void *pv);
void stopMotor1(void *pv);
void stopMotor2(void *pv);
void stopMotor3(void *pv);
void stopMotor4(void *pv);
void CH1_CW(void *pv);
void CH1_CCW(void *pv);
void CH2_CW(void *pv);
void CH2_CCW(void *pv);
void CH3_CW(void *pv);
void CH3_CCW(void *pv);
void CH4_CW(void *pv);
void CH4_CCW(void *pv);
void tuiliaodongzuo(void *pv);
void offsetSelect(void *pv);
void start_xiashouliaodongzuo(void *pv);
void lianxushangshouliao(void *pv);
void lianxuxiashouliao(void *pv);
void GMOffsetEnter(void *pv);

//主页面操作
void start_mainGive(void *pv);
void start_mainCut(void *pv);
void start_mainRst(void *pv);
//调试界面操作
void start_DebugGive(void *pv);
void start_DebugCut(void *pv);
void start_DebugRst(void *pv);
void start_DebugFL(void *pv);
void start_DebugCtnFL(void *pv);
void start_DebugUpSL(void *pv);
void start_DebugCtnUpSL(void *pv);
void start_DebugDownSL(void *pv);
void start_DebugCtnDownSL(void *pv);
void start_DebugSL(void *pv);
void start_DebugQG(void *pv);

void QL_Action1(void);
void JL_Action1(void);
void YL_Action1(void);

void start_lianxusongliao(void *pv);
void start_fangliaodongzuo(void *pv);
void start_shangshouliaodongzuo(void *pv);
void start_lianxufangliao(void *pv);
void sensorSelect(void *pv);
void givenModeSelect(void *pv);
void Language_Selection(void *pv);
void resetFeederPara(void *pv);
void functionChose(void *pv);

static uint8_t passwordCompare(uint8_t which, uint32_t pw);

EventCallback eventCallback[] = {runStatusControl,              // 0
                                checkPassword,                  // 1
                                returnPage,                     // 2
                                fillUserName,                   // 3
                                gotoFeederParameterPage,        // 4
                                logout,                         // 5
                                changePassword,                 // 6
                                userManagePage,                 // 7
                                jump2MotorParameterPage,        // 8
								start_DebugGive,				//9
								start_DebugCut,                 // 10 A
								start_DebugFL,             		// 11 B
								start_DebugCtnFL,           	// 12 C
								start_DebugRst,           		// 13 D
								start_DebugUpSL,                // 14 E
								start_DebugCtnUpSL,           	// 15 F
								start_DebugDownSL,				// 16 10
								start_DebugCtnDownSL,           // 17 11
                                CH1_CW,                         // 18
                                CH1_CCW,                        // 19
                                CH2_CW,                         // 20
                                CH2_CCW,                        // 21
                                CH3_CW,                         // 22
                                CH3_CCW,                        // 23
                                CH4_CW,                         // 24
                                CH4_CCW,                        // 25
                                startupInit,                    // 26
                                selectOnlineMode,               // 27
                                selectAutoMode,                 // 28
                                selectFreeRunMode,              // 29
                                jump2MainPage,                  // 30
                                jump2debugPage,                 // 31 1F
                                jump2inputPage,                 // 32 20
                                jump2outputPage,                // 33
                                jump2loginPage,                 // 34
                                stopMotor1,                     // 35
                                stopMotor2,                     // 36
                                stopMotor3,                     // 37
                                stopMotor4,                     // 38
                                controlOutputSignal,            // 39
                                offsetSelect,                   // 40
								huiyuandian_UI,                 // 41
								start_mainGive,            		// 42 2A
								start_mainCut,               	// 43 2B
								start_mainRst,                  // 44 2C
                                givenModeSelect,                // 45
                                resetFeederPara,                // 46
								functionChose,					// 47
								start_DebugQG,					// 48
								Language_Selection,				// 49
								konghanshu,						// 50
								start_DebugRst,					// 51
								konghanshu,
								konghanshu,
};          

//void EVENT_task(void *pv)
//{
//    uint8_t i;
//
//    while(1)
//    {
//        for(i=0;i<ADDR6000_NUM;i++)
//        {
//            if(ADDR_6000_L[i] != 0)
//            {
//                eventCallback[i](&(ADDR_6000_L[i]));
//            }
//        }
//
//        rt_thread_mdelay(50);
//    }
//}

/**
 * @brief 判断密码，处理登录请求
 * @param pv
 */
void checkPassword(void *pv)
{
    uint32_t password;

    password = PARA_readParameter(0x7000);
    password <<= 16;
    password += PARA_readParameter(0x7001);

    if(systemPara.logginStatus == 0)
    {
    	if(systemPara.Language == 0)
    		GUI_showMessage("请选择用户！",12);
    	else if(systemPara.Language == 1)
    		GUI_showMessage("Please select user!",19);
    	return;
    }

    if(systemPara.passwordStatus == 0)
    {
    	if(systemPara.Language == 0)
    		GUI_showMessage("请输入密码！",12);
    	else if(systemPara.Language == 1)
    		GUI_showMessage("Please enter password!",22);
    	return;
    }

    switch(systemPara.logginStatus)
    {
        case 1:
            if(passwordCompare(0, password))
            {
            	if(systemPara.status != STATUS_AUTO)
				{
            		 GUI_switchPage(32);
				}
				else
				{//其他
					 GUI_switchPage(0);
				}
                GUI_showText(PASSWORD_DISP_ADDR,"      ",6);
                GUI_showText(USERNAME_DISP_ADDR,"      ",6);
                systemPara.logginStatus += 10;      // 记录以User登录
                GUI_showText(LOGGINSTATE_DISP_ADDR,"   User ",8);
            }
            else
            {
                GUI_showText(PASSWORD_DISP_ADDR, "      ", 6);

            	if(systemPara.Language == 0)
            		 GUI_showMessage("密码错误",8);
            	else if(systemPara.Language == 1)
            		GUI_showMessage("Wrong password!",15);
            }
        break;

        case 2:
            if(passwordCompare(1, password))
            {

            	if(systemPara.status != STATUS_AUTO)
				{
            		 GUI_switchPage(32);
				}
				else
				{//其他
					 GUI_switchPage(0);
				}
                GUI_showText(PASSWORD_DISP_ADDR,"      ",6);
                GUI_showText(USERNAME_DISP_ADDR,"      ",6);
                systemPara.logginStatus += 10;      // 记录以Vendor登录
                GUI_showText(LOGGINSTATE_DISP_ADDR,"  Vendor",8);
            }
            else if(passwordCompare(2,password))
            {
            	if(systemPara.status != STATUS_AUTO)
				{
            		 GUI_switchPage(32);
				}
				else
				{//其他
					 GUI_switchPage(0);
				}
                GUI_showText(PASSWORD_DISP_ADDR,"      ",6);
                GUI_showText(USERNAME_DISP_ADDR,"      ",6);
                systemPara.logginStatus += 20;      // 记录以Vendor登录
                GUI_showText(LOGGINSTATE_DISP_ADDR,"Super   ",8);
            }
            else
            {
                GUI_showText(PASSWORD_DISP_ADDR, "      ", 6);
            	if(systemPara.Language == 0)
            		 GUI_showMessage("密码错误",8);
            	else if(systemPara.Language == 1)
            		GUI_showMessage("Wrong password!",15);
            }
        break;
    }


    ADDR_6000_L[1] = 0;
}


/**
 * @brief 停止进程
 * @param pv
 */
void runStatusControl(void *pv)
{  

    uint8_t i;
    
    switch(*((uint8_t *)pv))
    {
        case 1://停止
			if(systemPara.status != STATUS_ONLINE)
        	{
				for(i=0;i<4;i++)
				{
					MC_motorStop(i);
	//                MC_AmpDecay(i);
				}

				GUI_runStatusDisp(0);
				StopModeUI(0);
        	}
        	else
        	{
        		if(systemPara.Language == 0)
        			CANT_SWITCH_PAGE_MESSAGE1();
        		else
        			CANT_SWITCH_PAGE_MESSAGE1();
        		return;
        	}
            break;
        case 4://紧急停止
        	EnterStopModeUI(0);
            break;
    }

}

/**
 * @brief 消息页返回
 * @param pv
 */
void returnPage(void *pv)
{
	if(systemPara.Language == 0)
		GUI_switchPage(pageNoBeforMessage);
//	if(systemPara.Language == 1)
//		GUI_switchPage(pageNoBeforMessage-50);

    ADDR_6000_L[2] = 0;
}

/**
 * @brief 填充用户名
 * @param pv
 */
void fillUserName(void *pv)
{
    switch(*(uint8_t *)pv)
    {
        case 1:
            GUI_showText(USERNAME_DISP_ADDR,"User  ",6);
            systemPara.logginStatus = 1;
        break;

        case 2:
            GUI_showText(USERNAME_DISP_ADDR,"Vendor",6);
            systemPara.logginStatus = 2;
        break;
    }

    ADDR_6000_L[3] = 0;

    GUI_showText(PASSWORD_DISP_ADDR,"      ",6);//清除密码框
}


/**
 * @brief 登出
 * @param pv
 */
void logout(void *pv)
{
    ADDR_6000_L[5] = 0;

    GUI_showText(USERNAME_DISP_ADDR, "      ", 6);
    GUI_showText(PASSWORD_DISP_ADDR, "      ", 6);
    systemPara.logginStatus = 0;

    if(systemPara.Language == 0)
    	GUI_showMessage("登出中...",9);
	else if(systemPara.Language == 1)
		GUI_showMessage("Logging out...",14);
    delay_ms(500);

    if(systemPara.Language == 0)
    	GUI_showMessage("已退出登录", 10);
	else if(systemPara.Language == 1)
		GUI_showMessage("Logging out",11);
    delay_ms(2);
    GUI_showTextIsolate(LOGGINSTATE_DISP_ADDR,USERNAME_DISP_NAME,USERNAME_DISP_NAME_LEN);
}

/**
 * @brief 修改密码
 * @param pv
 */
void changePassword(void *pv)
{
    uint32_t password;

    ADDR_6000_L[6] = 0;


    if(systemPara.logginStatus == 0)
    {
    	if(systemPara.Language == 0)
    		GUI_showMessage("请选择用户！",12);
    	else if(systemPara.Language == 1)
    		GUI_showMessage("Please select user!",19);
    	return;
    }

    if(systemPara.passwordStatus < 1)
    {
    	if(systemPara.Language == 0)
    		GUI_showMessage("请输入旧密码！",14);
    	else if(systemPara.Language == 1)
    		GUI_showMessage("Please enter old password!",26);
    	return;
    }
    else if(systemPara.passwordStatus < 2)
    {
    	if(systemPara.Language == 0)
    		GUI_showMessage("请输入新密码！",14);
    	else if(systemPara.Language == 1)
    		GUI_showMessage("Please enter new password!",26);
    	return;
    }
    else if(systemPara.passwordStatus < 3)
    {
    	if(systemPara.Language == 0)
    		GUI_showMessage("请输入确认新密码！",18);
    	else if(systemPara.Language == 1)
    		GUI_showMessage("Please enter a new password to confirm!",39);
    	return;
    }
    password = PARA_readParameter(0x7000);
    password <<= 16;
    password += PARA_readParameter(0x7001);

    if(systemPara.logginStatus > 10)
        systemPara.logginStatus -= 10;

    if((systemPara.logginStatus == 1) && (passwordCompare(0, password)))
    {
        uint32_t password1,password2;

        password1 = PARA_readParameter(0x7002);
        password1 <<= 16;
        password1 += PARA_readParameter(0x7003);

        password2 = PARA_readParameter(0x7004);
        password2 <<= 16;
        password2 += PARA_readParameter(0x7005);

        if(password1 == password2)
        {
            PARA_setPasswordInt(0, password1);
            STORAGE_updataPassword(0);

            GUI_showText(PASSWORD_DISP_ADDR, "      ", 6);
            GUI_showText(NPASS_DISP_ADDR, "      ", 6);
            GUI_showText(NCPASS_DISP_ADDR, "      ", 6);

            if(systemPara.Language == 0)
            	GUI_showMessage("密码修改完成", 10);
        	else if(systemPara.Language == 1)
        		GUI_showMessage("Password change completed.",26);
        }
        else
        {
            GUI_showText(PASSWORD_DISP_ADDR, "      ", 6);
            GUI_showText(NPASS_DISP_ADDR, "      ", 6);
            GUI_showText(NCPASS_DISP_ADDR, "      ", 6);
            
            if(systemPara.Language == 0)
            	GUI_showMessage("密码不一致", 10);
			else if(systemPara.Language == 1)
				GUI_showMessage("The passwords are inconsistent.",31);
        }

    }
    else if((systemPara.logginStatus == 2) && (passwordCompare(1,password)))
    {
        uint32_t password1,password2;

        password1 = PARA_readParameter(0x7002);
        password1 <<= 16;
        password1 += PARA_readParameter(0x7003);

        password2 = PARA_readParameter(0x7004);
        password2 <<= 16;
        password2 += PARA_readParameter(0x7005);

        if(password1 == password2)
        {
            PARA_setPasswordInt(1, password1);
            STORAGE_updataPassword(1);

            GUI_showText(PASSWORD_DISP_ADDR, "      ", 6);
            GUI_showText(NPASS_DISP_ADDR, "      ", 6);
            GUI_showText(NCPASS_DISP_ADDR, "      ", 6);

            if(systemPara.Language == 0)
				GUI_showMessage("密码修改完成", 10);
			else if(systemPara.Language == 1)
				GUI_showMessage("Password change completed",23);
        }
        else
        {
            GUI_showText(PASSWORD_DISP_ADDR, "      ", 6);
            GUI_showText(NPASS_DISP_ADDR, "      ", 6);
            GUI_showText(NCPASS_DISP_ADDR, "      ", 6);
            if(systemPara.Language == 0)
				GUI_showMessage("密码不一致", 10);
			else if(systemPara.Language == 1)
				GUI_showMessage("The passwords are inconsistent.",31);
        }
    }
    else
    {
        GUI_showText(PASSWORD_DISP_ADDR, "      ", 6);
        GUI_showText(NPASS_DISP_ADDR, "      ", 6);
        GUI_showText(NCPASS_DISP_ADDR, "      ", 6);
    	if(systemPara.Language == 0)
    		 GUI_showMessage("密码错误",8);
    	else if(systemPara.Language == 1)
    		GUI_showMessage("Wrong password!",15);
    }
}

/**
 *
 * @param pv
 */
void userManagePage(void *pv)
{//0x6007
    ADDR_6000_L[7] = 0;

    GUI_showText(0x7500, "      ", 6);
    GUI_showText(0x7505, "      ", 6);
    GUI_showText(0x7520, "      ", 6);
    GUI_showText(0x7523, "      ", 6);
    systemPara.passwordStatus = 0;
    GUI_switchPage(8);

}

/**
 * @brief 跳转至电机控制页
 * @param pv
 */
void jump2MotorParameterPage(void *pv)
{
    ADDR_6000_L[8] = 0;
    
    if(systemPara.status == STATUS_ONLINE)
    {
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE1();
		else
			CANT_SWITCH_PAGE_MESSAGE1();
    	return;
    }

    IN_refreshUICmd(0);

    if(systemPara.logginStatus < 12)
    {//未登录
        GUI_switchPage(31);
    }
    else
    {//已登录
        GUI_switchPage(5);
    }
}

void jump2MainPage(void *pv)
{
    ADDR_6000_L[30] = 0;
    
    IN_refreshUICmd(0);
    
    systemPara.Page = 0;

    if(systemPara.logginStatus < 10 || systemPara.status == STATUS_ONLINE)
    {//未登录和工作模式
        GUI_switchPage(32);
    }
    else
    {//其他
        GUI_switchPage(0);
    }
/*
    switch(systemPara.status)
    {
        case STATUS_UNINIT:
            GUI_switchPage(0);
            break;
        case STATUS_INITED:
            GUI_switchPage(21);
            break;
        case STATUS_PRODUCT:
            GUI_switchPage(22);
            break;
        case STATUS_DEBUG:
            GUI_switchPage(23);
            break;
        case STATUS_FREERUN:
            GUI_switchPage(24);
            break;
    }
*/
}

/**
 * @brief 跳转至参数设置页面
 * @param pv
 */
void gotoFeederParameterPage(void *pv)
{
    ADDR_6000_L[4] = 0;
	

    if(systemPara.status == STATUS_ONLINE)
    {
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE1();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE1();
    	return;
    }

    
    IN_refreshUICmd(0);

    if(systemPara.logginStatus < 10)
    {//未登录
        GUI_switchPage(30);
    }
    else
    {//已登录
        GUI_switchPage(1);
    }
}

void jump2debugPage(void *pv)
{
    ADDR_6000_L[31] = 0;
    
	if(systemPara.status == STATUS_ONLINE)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE1();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE1();
		return;
	}
	systemPara.Page = 4;

    IN_refreshUICmd(0);
    
    GUI_switchPage(2);
}

void selectOnlineMode(void *pv)
{
    ADDR_6000_L[27] = 0;
    

    systemPara.status = STATUS_ONLINE;
    AT24CXX_WriteOneByte(STORAGE_WORKMODE_ADDR, systemPara.status);
    GUI_switchModeDisp(systemPara.status);

    if((IBIO_getInput(1) == 0 || systemPara.givenOK ==1)&& systemPara.Initstatus == 1)
    {
		if(systemPara.Initstatus)
		{
			EXO_sigON(3);
			EXO_sigON(5);
			if(systemPara.givenOK)
				EXO_sigON(2);
		}
    }
    else
    {
    	systemPara.Initstatus = 0;
    	GUI_runStatusDisp(0);
			MAIN_TIPS_INNI_FIRST();
    }

	GUI_switchPage(32);
		MAIN_INFO_SW2ONLINE();


    
}

void selectAutoMode(void *pv)
{
    ADDR_6000_L[28] = 0;
    
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
	if(systemPara.Language == 0)
		MAIN_INFO_SW2DEBUG();
	else
		EN_MAIN_INFO_SW2DEBUG();
    systemPara.status = STATUS_AUTO;
    AT24CXX_WriteOneByte(STORAGE_WORKMODE_ADDR, systemPara.status);
    GUI_switchModeDisp(systemPara.status);
}

void selectFreeRunMode(void *pv)
{
    ADDR_6000_L[29] = 0;
    
    
#ifdef CONF_EXTRIG_INT    
    EXT_interruptMode(EXT_INT_MODE_NONE);
#endif
    
	if(systemPara.Language == 0)
		MAIN_INFO_SW2DEBUG();
	else
		EN_MAIN_INFO_SW2DEBUG();
    systemPara.status = STATUS_FREERUN;
    AT24CXX_WriteOneByte(STORAGE_WORKMODE_ADDR, systemPara.status);
    GUI_switchModeDisp(systemPara.status);
    
}

void jump2inputPage(void *pv)
{
    ADDR_6000_L[32] = 0;

    GUI_switchPage(3);
    
    IN_refreshUICmd(1);
    
}

void jump2outputPage(void *pv)
{
    ADDR_6000_L[33] = 0;
    
	if(systemPara.status == STATUS_ONLINE)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE1();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE1();
		return;
	}
    
    IN_refreshUICmd(0);
    
    GUI_switchPage(4);
}

void jump2loginPage(void *pv)
{
    ADDR_6000_L[34] = 0;
    
    IN_refreshUICmd(0);
    systemPara.passwordStatus = 0;
    GUI_showText(PASSWORD_DISP_ADDR,"      ",6);
   	GUI_showText(USERNAME_DISP_ADDR,"      ",6);
    GUI_switchPage(7);
}

void stopMotor1(void *pv)
{
    ADDR_6000_L[35] = 0;
    
    MC_motorStop(0);
}

void stopMotor2(void *pv)
{
    ADDR_6000_L[36] = 0;
    
    MC_motorStop(1);
}

void stopMotor3(void *pv)
{
    ADDR_6000_L[37] = 0;
    
    MC_motorStop(2);
}

void stopMotor4(void *pv)
{
    ADDR_6000_L[38] = 0;
    
    MC_motorStop(3);
}

void CH1_CW(void *pv)
{
    ADDR_6000_L[18] = 0;
    
    MC_motorMoveForever(0,0);
}

void CH1_CCW(void *pv)
{
    ADDR_6000_L[19] = 0;
    
    MC_motorMoveForever(0,1);
}

void CH2_CW(void *pv)
{
    ADDR_6000_L[20] = 0;
    
    MC_motorMoveForever(1,0);
}

void CH2_CCW(void *pv)
{
    ADDR_6000_L[21] = 0;
    
    MC_motorMoveForever(1,1);
}

void CH3_CW(void *pv)
{
    ADDR_6000_L[22] = 0;
    
    MC_motorMoveForever(2,0);
}

void CH3_CCW(void *pv)
{
    ADDR_6000_L[23] = 0;
    
    MC_motorMoveForever(2,1);
}

void CH4_CW(void *pv)
{
    ADDR_6000_L[24] = 0;
    
    MC_motorMoveForever(3,0);
}

void CH4_CCW(void *pv)
{
    ADDR_6000_L[25] = 0;
    
    MC_motorMoveForever(3,1);
}

/**
 * 退料动作和连续退料在同一个函数
 */
void tuiliaodongzuo(void *pv)
{
	uint32_t limitPulseCount;
	static uint8_t currentStatus=0;

	switch (*((uint8_t*) pv))
	{
	case 1:
		limitPulseCount = (ADDR_5000_H[1] << 8) | ADDR_5000_L[1];

		MC_motorMoveDistance(GIVEN_MOTOR, 1, limitPulseCount);

		while ((motor_ch[GIVEN_MOTOR].status != Motor_Stop)
				&& (systemPara.isWorkTaskRun))
			;
		break;

	case 2:
		if(currentStatus)
		{
			MC_motorStop(GIVEN_MOTOR);
			currentStatus = 0;
		}
		else
		{
			MC_motorMoveForever(GIVEN_MOTOR, 1);
			currentStatus = 1;
		}

		GUI_sendWord(0x7823, 0, currentStatus);
		break;
	}


}

/**
 @brief 偏移量方向选择
*/
void offsetSelect(void *pv)
{
    switch(ADDR_6000_L[40])
    {
        case 1:
            GUI_sendWord(SINGAL_SELECT_1_ADDR,0,1);
            GUI_sendWord(SINGAL_SELECT_2_ADDR,0,0);
        
            MOTOR_offsetOrigin = 1;
            AT24CXX_WriteOneByte(0x43,1);
            break;
        
        case 2:
            GUI_sendWord(SINGAL_SELECT_1_ADDR,0,0);
            GUI_sendWord(SINGAL_SELECT_2_ADDR,0,1);
        
            MOTOR_offsetOrigin = 2;
            AT24CXX_WriteOneByte(0x43,2);
            break;
    }
    
    ADDR_6000_L[40] = 0;
}


/**
 将 剥刀动作 用anyTask线程运行
*/
void start_boliaodongzuo(void *pv)
{
    ADDR_6000_L[10] = 0;

}

/**
 连续上收料
*/
void lianxushangshouliao(void *pv)
{

	static uint8_t currentStatus = 0;

	if (currentStatus)
	{
		currentStatus = 0;

		MC_motorStop(DOWN_MOTOR);
	}
	else
	{
		currentStatus = 1;

		MC_motorMoveForever(DOWN_MOTOR, 0);
	}

	GUI_sendWord(0x7693, 0, currentStatus);

}



void GMOffsetEnter(void *pv)
{
//    extern uint16_t tempGMOffset;
//
//    ADDR_6000_L[41] = 0;
//
////    PARA_writeParameter(0x5012,tempGMOffset);
//
//    AT24CXX_WriteOneByte(0x34,ADDR_5000_L[18]);
//    AT24CXX_WriteOneByte(0x35,ADDR_5000_H[18]);
//
//    tempGMOffset = 0;
//
//    GUI_showMessage("参数已确认",10);
}


/**
 置位启动线程标志
*/
void startupTask(void)
{

}


/**
 @brief 初始化回调函数.
 执行初始化功能
*/

void startupInit(void *pv)
{
	ADDR_6000_L[26] = 0;
	if(systemPara.status == STATUS_ONLINE)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE1();
		else
			CANT_SWITCH_PAGE_MESSAGE1();
		return;
	}
    //GUI_mainMessageDispIsolate("执行初始化动作.", 15);
    //delay_ms(200);

//    GUI_showMessage("初始化",6);
   // ChuShiHua(0);

	if(systemPara.status == STATUS_FREERUN && systemPara.Initstatus == 1)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE3();
		else
			CANT_SWITCH_PAGE_MESSAGE3();
		return;
	}
	if(systemPara.RunStatus == 0)
	{
//		if(systemPara.status == STATUS_ONLINE)
//		{
//			if(systemPara.Err == 1)
//			{
//				systemPara.Err = 0;
//				EXO_sigOFF(4);//清除报警信号
//			}
//		}
		//if(systemPara.Initstatus == 0)
			systemPara.doInit = 1;
	}
	else
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE2();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE2();
	}

}


void start_mainGive(void *pv)
{
	ADDR_6000_L[42] = 0;

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


	if(systemPara.Initstatus == STATUS_UNINIT)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE0();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE0();
		return;
	}

	if(systemPara.RunStatus == 0)
	{
		if(systemPara.givenOK == 1)
		{
			if(systemPara.Language == 0)
				MAIN_TIPS_NO_CUT_FEED();
			else if(systemPara.Language == 1)
				EN_MAIN_TIPS_CUT_FEED();
		}
		else
		{
//			systemPara.givenOnceTriggerByUIorIO = 1;
			systemPara.RSTOnceTriggerByUIorIO = 1;
		}
	}
	else
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE2();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE2();
	}
}

void start_mainCut(void *pv)
{
	ADDR_6000_L[43] = 0;

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


	if(systemPara.Initstatus == STATUS_UNINIT)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE0();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE0();
		return;
	}

	if(IBIO_getInput(1) == 0)
	{
		if(systemPara.Language == 0)
			GUI_mainMessageDisp("原点位，禁止裁切.", 25);
		else
			GUI_mainMessageDisp("Origin position, cutting is prohibited.", 39);
		return;
	}
	//GUI_mainMessageDispIsolate("1", 10);
	if(systemPara.RunStatus == 0)
		systemPara.CUTOnceTriggerByUIorIO = 1;
	else
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE2();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE2();
	}

}

void start_mainRst(void *pv)
{
	ADDR_6000_L[44] = 0;

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

	if(systemPara.RunStatus != 0)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE2();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE2();
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
		systemPara.Initstatus = 0;
		replacenment_flag = 0;
		systemPara.givenOK = 0;
		systemPara.CUTOK = 0;
		systemPara.RSTOK = 0;

		IBIO_setOutput(1, 0);
		GUI_setOutputSignalColorDisp(0,1);
		IBIO_setOutput(2, 0);
		GUI_setOutputSignalColorDisp(1,1);
		IBIO_setOutput(3, 1);
		GUI_setOutputSignalColorDisp(2,1);
		IBIO_setOutput(4, 1);
		GUI_setOutputSignalColorDisp(3,1);

		GUI_mainMessageDisp("全部气缸已经打开.", 17);

		if(IBIO_getInput(1) == 0)
		{
			//换料自动退40mm
			systemPara.Initstatus = 1;
			systemPara.givenOnceTriggerByUIorIO = 3;
		}
//		else
//		{
//			GUI_mainMessageDisp("换料动作完成.", 13);
//		}
	}
	else
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE2();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE2();
	}
}

void start_DebugGive(void *pv)
{
	ADDR_6000_L[9] = 0;
	if(systemPara.Initstatus == STATUS_UNINIT)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE0();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE0();
		return;
	}

	if(systemPara.RunStatus == 0)
	{
		systemPara.RSTOnceTriggerByUIorIO = 1;
	}
	else
	{
			CANT_SWITCH_PAGE_MESSAGE2();
	}

}
void start_DebugCut(void *pv)
{
	ADDR_6000_L[10] = 0;
	if(systemPara.Initstatus == STATUS_UNINIT)
	{
			CANT_SWITCH_PAGE_MESSAGE0();
		return;
	}


	switch (*((uint8_t*) pv))
	{
	case 1:
		if(systemPara.RunStatus == 0)
		{
			systemPara.CUTOnceTriggerByUIorIO = 1;
		}
		else
		{
				CANT_SWITCH_PAGE_MESSAGE2();
		}
		break;
	case 2:
		if(systemPara.RunStatus == 0)
		{
			systemPara.RSTCUTOnceTriggerByUIorIO = 1;
		}
		else
		{
			CANT_SWITCH_PAGE_MESSAGE2();
		}
		break;
	}
}
void start_DebugRst(void *pv)
{
	ADDR_6000_L[13] = 0;
	if(systemPara.Initstatus == STATUS_UNINIT)
	{
		CANT_SWITCH_PAGE_MESSAGE0();
		return;
	}

	if(systemPara.RunStatus != 0)
	{
		CANT_SWITCH_PAGE_MESSAGE2();
	}
	switch (*((uint8_t*) pv))
	{
	case 1:
		if(IBIO_getInput(1) == 1)
		{
			GUI_showMessage("请先复位动作.", 13);
			return;
		}

		systemPara.givenOnceTriggerByUIorIO = 2;

		break;
	case 2:
		if(IBIO_getInput(1) == 0)
		{
			GUI_showMessage("请先送料动作.", 13);
			return;
		}

		systemPara.RSTOnceTriggerByUIorIO = 3;

		break;

	}
}

void start_DebugFL(void *pv)
{
	ADDR_6000_L[11] = 0;
	uint32_t limitPulseCount;

	if(systemPara.isAutoLetMetalEnable == 0)
	{
		if(systemPara.Language == 0)
			GUI_showMessage("放料功能未启用.", 13);
		if(systemPara.Language == 1)
			GUI_showMessage("The unwinding function is not enabled.", 38);
		return;
	}

	if(motor_ch[LET_MOTOR].status == Motor_Stop)
	{
		limitPulseCount = (ADDR_5000_H[3]<<8)|ADDR_5000_L[3];
		systemPara.FLmode = 1;
		MC_motorMoveDistance(LET_MOTOR,0,limitPulseCount);
	}
}

void start_DebugCtnFL(void *pv)
{
	ADDR_6000_L[12] = 0;

	static uint8_t currentStatus = 0;

	if(systemPara.isAutoLetMetalEnable == 0)
	{
		if(systemPara.Language == 0)
			GUI_showMessage("放料功能未启用.", 13);
		if(systemPara.Language == 1)
			GUI_showMessage("The unwinding function is not enabled.", 38);
		return;
	}

	if (currentStatus)
	{
		currentStatus = 0;

		MC_motorStop(LET_MOTOR);
	}
	else
	{
		currentStatus = 1;
		systemPara.FLmode = 0;
		MC_motorMoveForever(LET_MOTOR, 0);
	}

	GUI_sendWord(0x7902, 0, currentStatus);
}

void start_DebugUpSL(void *pv)
{
	ADDR_6000_L[14] = 0;
	uint32_t limitPulseCount;

	if(systemPara.isUpShouEnable == 0)
	{
		if(systemPara.Language == 0)
			GUI_showMessage("收料功能未启用.", 13);
		if(systemPara.Language == 1)
			GUI_showMessage("The take-up function is not enabled.", 36);
		return;
	}

	if(motor_ch[UP_MOTOR].status == Motor_Stop)
	{
		limitPulseCount = (ADDR_5000_H[7]<<8)|ADDR_5000_L[7];
		systemPara.UPSLmode = 1;//传感模式
		MC_motorMoveDistance(UP_MOTOR, 0, limitPulseCount);
	}

}
void start_DebugCtnUpSL(void *pv)
{
	ADDR_6000_L[15] = 0;

    static uint8_t currentStatus=0;

	if(systemPara.isUpShouEnable == 0)
	{
		if(systemPara.Language == 0)
			GUI_showMessage("收料功能未启用.", 13);
		if(systemPara.Language == 1)
			GUI_showMessage("The take-up function is not enabled.", 36);
		return;
	}

    if(currentStatus)
    {
    	currentStatus = 0;

    	MC_motorStop(UP_MOTOR);
    }
    else
    {
    	currentStatus = 1;
    	systemPara.UPSLmode = 0;
    	MC_motorMoveForever(UP_MOTOR, 0);
    }

    GUI_sendWord(0x7690, 0, currentStatus);
}
void start_DebugDownSL(void *pv)
{
	ADDR_6000_L[16] = 0;
	uint32_t limitPulseCount;

	if(motor_ch[DOWN_MOTOR].status == Motor_Stop)
	{
		limitPulseCount = (ADDR_5000_H[5]<<8)|ADDR_5000_L[5];
		systemPara.DOWNSLmode = 1;//传感模式
		MC_motorMoveDistance(DOWN_MOTOR, 0, limitPulseCount);
	}
}


void start_DebugCtnDownSL(void *pv)
{
	ADDR_6000_L[17] = 0;
	static uint8_t currentStatus = 0;

	if (currentStatus)
	{
		currentStatus = 0;

		MC_motorStop(DOWN_MOTOR);
	}
	else
	{
		currentStatus = 1;
		systemPara.DOWNSLmode = 0;
		MC_motorMoveForever(DOWN_MOTOR, 0);
	}

	GUI_sendWord(0x7693, 0, currentStatus);
}

void start_DebugQG(void *pv)
{
	switch (*((uint8_t*) pv))
	{
	case 1:
		if(IBIO_getOutput(1) == 1)
		{
			IBIO_setOutput(1, 0);
			GUI_setOutputSignalColorDisp(0,0);
		}
		else
		{
			IBIO_setOutput(1, 1);
			GUI_setOutputSignalColorDisp(0,1);
		}
		break;

	case 2:
		if(IBIO_getOutput(2) == 1)
		{
			IBIO_setOutput(2, 0);
			GUI_setOutputSignalColorDisp(1,0);
		}
		else
		{
			IBIO_setOutput(2, 1);
			GUI_setOutputSignalColorDisp(1,1);
		}
		break;

	case 3:
		if(IBIO_getOutput(3) == 1)
		{
			IBIO_setOutput(3, 0);
			GUI_setOutputSignalColorDisp(2,0);
		}
		else
		{
			IBIO_setOutput(3, 1);
			GUI_setOutputSignalColorDisp(2,1);
		}
		break;
	case 4:
		if(IBIO_getOutput(4) == 1)
		{
			IBIO_setOutput(4, 0);
			GUI_setOutputSignalColorDisp(3,0);
		}
		else
		{
			IBIO_setOutput(4, 1);
			GUI_setOutputSignalColorDisp(3,1);
		}
		break;
	}
}

//切料气缸动作
void QL_Action1(void)
{
	if(IBIO_getOutput(1) == 1)
	{
		IBIO_setOutput(1, 0);
		GUI_setOutputSignalColorDisp(0,0);
	}
	else
	{
		IBIO_setOutput(1, 1);
		GUI_setOutputSignalColorDisp(0,1);
	}
}


//夹料气缸动作
void JL_Action1(void)
{
	if(IBIO_getOutput(2) == 1)
	{
		IBIO_setOutput(2, 0);
		GUI_setOutputSignalColorDisp(1,0);
	}
	else
	{
		IBIO_setOutput(2, 1);
		GUI_setOutputSignalColorDisp(1,1);
	}
}

//压料气缸动作
void YL_Action1(void)
{
	if(IBIO_getOutput(3) == 1)
	{
		IBIO_setOutput(3, 0);
		GUI_setOutputSignalColorDisp(2,0);
	}
	else
	{
		IBIO_setOutput(3, 1);
		GUI_setOutputSignalColorDisp(2,1);
	}
}
void sensorSelect(void *pv)
{
//    switch(ADDR_6000_L[44])
//    {
//        case 1:
//            GUI_sendWord(SINGAL_SENSOR_DOWN_ADDR,0,1);
//            GUI_sendWord(SINGAL_SENSOR_UP_ADDR,0,0);
//
//            systemPara.sensorChosen = 1;
//            AT24CXX_WriteOneByte(0x44,1);
//            break;
//
//        case 2:
//            GUI_sendWord(SINGAL_SENSOR_DOWN_ADDR,0,0);
//            GUI_sendWord(SINGAL_SENSOR_UP_ADDR,0,1);
//
//            systemPara.sensorChosen = 2;
//            AT24CXX_WriteOneByte(0x44,2);
//            break;
//    }
//
//    ADDR_6000_L[44] = 0;
}

void givenModeSelect(void *pv)
{
    switch(ADDR_6000_L[45])
    {
        case 1:
            GUI_sendWord(SINGAL_GIVENMODE_N,0,1);
            GUI_sendWord(SINGAL_GIVENMODE_Y,0,0);
        
            systemPara.givenMode = 1;
            AT24CXX_WriteOneByte(0x45,1);
            break;
        
        case 2:
            GUI_sendWord(SINGAL_GIVENMODE_N,0,0);
            GUI_sendWord(SINGAL_GIVENMODE_Y,0,1);
        
            systemPara.givenMode = 2;
            AT24CXX_WriteOneByte(0x45,2);
            break;
    }
    
    ADDR_6000_L[45] = 0;
}

void Language_Selection(void *pv)
{
	if(systemPara.Language == 0)
	{
//    	systemPara.Language = 1;
//		AT24CXX_WriteOneByte(STORAGE_LANGUAGE_ADDR,1);
//		GUI_mainMessageDisp("The language selection is Chinese.", 34);
//	    GUI_initDisplay();
		//sendBuffer[43][5] = 0x01;
		//USART1_sendBuf(sendBuffer[43],8);
	}
	else if(systemPara.Language == 1)
	{
    	systemPara.Language = 0;
		AT24CXX_WriteOneByte(STORAGE_LANGUAGE_ADDR,0);
		GUI_mainMessageDisp("语言选择为中文.", 15);
	    GUI_initDisplay();
    }

	if(systemPara.status != STATUS_AUTO)
	{
		GUI_switchPage(32);
	}
	else
	{//其他
		GUI_switchPage(0);
	}
}

void resetFeederPara(void *pv)
{
    ADDR_6000_L[46] = 0;
    
    GUI_showText(0x7508,"                    ",20);
    if(systemPara.Language == 0)
    	GUI_showText(INFO_DISP_ADDR,"正在恢复出厂参数...",19);
    else if(systemPara.Language == 1)
    	GUI_showText(INFO_DISP_ADDR,"Restoring factory parameters...",31);
    pageNoBeforMessage = 12;
    GUI_switchPage(12);
    
    //恢复供料器参数
    if(systemPara.logginStatus == 11)
    	STORAGE_resetFeederParam();
    else
    	STORAGE_setDefaultParameter();
    
    systemPara.logginStatus = 0;
    GUI_showText(0x7508,"                    ",46);
    if(systemPara.Language == 0)
    	GUI_showText(INFO_DISP_ADDR,"恢复完成, 请断电重启！",24);
    else if(systemPara.Language == 1)
       GUI_showText(INFO_DISP_ADDR,"Please power off and restart!",29);
}

void functionChose(void *pv)
{
	switch(*((uint8_t *)pv))
	{
	case 1:
		if(systemPara.isAutoLetMetalEnable)
		{
			systemPara.isAutoLetMetalEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_1_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_LET_EN_ADDR, 0);
			GUI_showTextIsolate(INPUT_08_NAME_ADDR,CONF_INPUT_008_NAME,CONF_INPUT_008_NAME_LEN);
			GUI_showTextIsolate(INPUT_09_NAME_ADDR,CONF_INPUT_009_NAME,CONF_INPUT_009_NAME_LEN);

		}
		else
		{
			systemPara.isAutoLetMetalEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_1_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_LET_EN_ADDR, 1);
			GUI_showTextIsolate(INPUT_08_NAME_ADDR,CONF_INPUT_08_NAME,CONF_INPUT_08_NAME_LEN);
			GUI_showTextIsolate(INPUT_09_NAME_ADDR,CONF_INPUT_09_NAME,CONF_INPUT_09_NAME_LEN);

		}
		break;
	case 2:

		break;
	case 3:
		if(systemPara.isUpShouSensorEnable)
		{
			systemPara.isUpShouSensorEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_3_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_UPSSOR_EN_ADDR, 0);
		}
		else
		{
			systemPara.isUpShouSensorEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_3_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_UPSSOR_EN_ADDR, 1);
		}
		break;
	case 4:
		if(systemPara.isFeedInPlaceEnable)
		{
			systemPara.isFeedInPlaceEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_4_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_FEEDINPLASE_ADDR, 0);
		}
		else
		{
			systemPara.isFeedInPlaceEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_4_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_FEEDINPLASE_ADDR, 1);
		}
		break;
	case 5:
		if(systemPara.isLackMaterral_NC)
		{
			systemPara.isLackMaterral_NC = 0;
			GUI_sendWord(FUNCTION_CHOSE_5_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_LACKMATERRAL_NC_EN_ADDR, 0);
		}
		else
		{
			systemPara.isLackMaterral_NC = 1;
			GUI_sendWord(FUNCTION_CHOSE_5_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_LACKMATERRAL_NC_EN_ADDR, 1);
		}
		break;
	case 6:
		if(systemPara.isInEmergencyStopEnable)
		{
			systemPara.isInEmergencyStopEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_6_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_EM_STOP_EN_ADDR, 0);
		}
		else
		{
			systemPara.isInEmergencyStopEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_6_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_EM_STOP_EN_ADDR, 1);
		}
		break;
	case 7:
		if(systemPara.isInitSwOnLINEEnable)
		{
			systemPara.isInitSwOnLINEEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_7_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_INITSWONLINE_EN_ADDR, 0);
		}
		else
		{
			systemPara.isInitSwOnLINEEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_7_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_INITSWONLINE_EN_ADDR, 1);
		}
		break;
	}
}

static uint8_t passwordCompare(uint8_t which, uint32_t pw)
{
    uint8_t rtl = 1;
    uint8_t i;

    switch(which)
    {
    case 0:
        for(i=6;i>0;i--)
        {
            if(((pw % 10) + '0') != systemPara.userPassword[i-1])
            {
                rtl = 0;
                break;
            }
            pw /= 10;
        }
        break;

    case 1:
        for(i=6;i>0;i--)
        {
            if(((pw % 10) + '0') != systemPara.vendorPassword[i-1])
            {
                rtl = 0;
                break;
            }
            pw /= 10;
        }
        break;
        
    case 2:
        for(i=6;i>0;i--)
        {
            if(((pw % 10) + '0') != systemPara.superPassword[i-1])
            {
                rtl = 0;
                break;
            }
            pw /= 10;
        }
        break;
    }

    return rtl;
}
