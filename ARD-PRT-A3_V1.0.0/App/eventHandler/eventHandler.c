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
#include "easyModbus/easyModbus.h"
//#include "extrigger/extrigger.h"
//#include "signal_event/signal_event.h"

void konghanshu(void *pv);

void runStatusControl(void *pv);
void checkPassword(void *pv);
void changePassword(void *pv);
void logout(void *pv);
void fillUserName(void *pv);

void returnPage(void *pv);
void jumpPage(void *pv);
void selectMode(void *pv);

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

void GMOffsetEnter(void *pv);

//主页面操作
void start_mainTake(void *pv);
void start_mainLabel(void *pv);
void start_mainRst(void *pv);
void start_mainHL(void *pv);
//调试界面操作
void start_DebugTake(void *pv);
void start_DebugLabel(void *pv);
void start_DebugRst(void *pv);

void start_DebugUpSL(void *pv);
void start_DebugQG(void *pv);
void LableSelect(void *pv);
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
								jumpPage,        				// 4 切换页面
                                logout,                         // 5
                                changePassword,                 // 6
								selectMode,                 	// 7
								huiyuandian_UI,        			// 8
								startupInit,          			// 9
								start_mainTake,					// 10 A
								start_mainLabel,          		// 11 B
								konghanshu,     				// 12 C
								konghanshu,          			// 13 D
								konghanshu,                   	// 14 E
								konghanshu,            			// 15 F
								konghanshu,                  	// 16 10
								konghanshu,               		// 17 11
								CH1_CW,                         // 18 12
								CH1_CCW,                        // 19 13
								CH2_CW,                         // 20
								CH2_CCW,                        // 21
								CH3_CW,                         // 22
								CH3_CCW,                        // 23
								CH4_CW,                         // 24 18
								CH4_CCW,                        // 25 19
								start_DebugTake,            	// 26 1A
								start_DebugLabel,               // 27 1B
								start_DebugUpSL,                // 28 1C
								start_DebugQG,                  // 29 1D
								konghanshu,                  	// 30 1E
								konghanshu,                 	// 31 2F
								konghanshu,                 	// 32 20
								konghanshu,                		// 33 21
								konghanshu,                 	// 34 22
								stopMotor1,                     // 35
								stopMotor2,                     // 36
								stopMotor3,                     // 37
								stopMotor4,                     // 38
                                controlOutputSignal,            // 39 27
								konghanshu,                   // 40 28
								konghanshu,						//41 29
								konghanshu,           			// 42 2A
								konghanshu,             		// 43 2B
								LableSelect,                   	// 44 2C
								konghanshu,                		// 45 2D
                                resetFeederPara,                // 46 2E
								functionChose,					// 47 2F
								Language_Selection,				// 48 30
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
        		else if(systemPara.Language == 1)
        			EN_CANT_SWITCH_PAGE_MESSAGE1();
        		return;
        	}
            break;
        case 4://紧急停止
        	EnterStopModeUI(0);
            break;
    }
}


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
    //systemPara.logginStatus = 0;
    systemPara.passwordStatus = 0;
    ADDR_6000_L[1] = 0;
}



/**
 * @brief 消息页返回
 * @param pv
 */
void returnPage(void *pv)
{
	if(systemPara.Language == 0)
		GUI_switchPage(pageNoBeforMessage);
	if(systemPara.Language == 1)
		GUI_switchPage(pageNoBeforMessage-50);

    ADDR_6000_L[2] = 0;
}

/**
 * @brief 填充用户名
 * @param pv
 */
void fillUserName(void *pv)
{
	 ADDR_6000_L[3] = 0;

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
 * @brief 跳转页面
 * @param pv
 */
void jumpPage(void *pv)
{
	ADDR_6000_L[4] = 0;
	switch(*((uint8_t *)pv))
	{
	case 1://主页面
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
		break;
	case 2://供料参数
//		if(systemPara.status == STATUS_ONLINE)
//		{
//			if(systemPara.Language == 0)
//				CANT_SWITCH_PAGE_MESSAGE1();
//			else if(systemPara.Language == 1)
//				EN_CANT_SWITCH_PAGE_MESSAGE1();
//			return;
//		}
		IN_refreshUICmd(0);
		systemPara.Page = 1;
		if(systemPara.logginStatus < 10 || systemPara.status == STATUS_ONLINE)
		{//未登录
			GUI_switchPage(30);
		}
		else
		{//已登录
			GUI_switchPage(1);
		}
		break;
	case 3://单步调试
	    if(systemPara.status == STATUS_ONLINE)
		{
			if(systemPara.Language == 0)
				CANT_SWITCH_PAGE_MESSAGE1();
			else if(systemPara.Language == 1)
				EN_CANT_SWITCH_PAGE_MESSAGE1();
			return;
		}
	    systemPara.Page = 2;
	    IN_refreshUICmd(0);
	    GUI_switchPage(2);
	    break;
	case 4://输入信号
	    systemPara.Page = 3;
	    IN_refreshUICmd(1);
	    GUI_switchPage(3);
		break;
	case 5://输出信号
		systemPara.Page = 4;
		IN_refreshUICmd(0);
		GUI_switchPage(4);
		break;
	case 6:
//		 if(systemPara.status == STATUS_ONLINE)
//		{
//			if(systemPara.Language == 0)
//				CANT_SWITCH_PAGE_MESSAGE1();
//			else if(systemPara.Language == 1)
//				EN_CANT_SWITCH_PAGE_MESSAGE1();
//			return;
//		}
		IN_refreshUICmd(0);
		systemPara.Page = 5;
		if(systemPara.logginStatus < 12 || systemPara.status == STATUS_ONLINE)
		{//未登录
			GUI_switchPage(31);
		}
		else
		{//已登录
			GUI_switchPage(5);
		}
		break;
	case 7://登录
	    IN_refreshUICmd(0);
	    GUI_showText(PASSWORD_DISP_ADDR,"      ",6);
	    GUI_showText(USERNAME_DISP_ADDR,"      ",6);
	    systemPara.passwordStatus = 0;
	    GUI_switchPage(7);
		break;
	case 8://修改密码
	    GUI_showText(USERNAME_DISP_ADDR, "      ", 6);
	    GUI_showText(PASSWORD_DISP_ADDR, "      ", 6);
	    GUI_showText(NPASS_DISP_ADDR, "      ", 6);
	    GUI_showText(NCPASS_DISP_ADDR, "      ", 6);
		systemPara.passwordStatus = 0;
	    GUI_switchPage(8);
		break;
	}
}

/**
 *	@brief 模式切换
 * @param pv
 */
void selectMode(void *pv)
{
	switch (*((uint8_t*) pv))
	{
	case 1:
		if(systemPara.status == STATUS_FREERUN && systemPara.Initstatus == 1)
		{
			if(systemPara.Language == 0)
				POP_UP_INFO_AGING_RUN();
			else if(systemPara.Language == 1)
				EN_POP_UP_INFO_AGING_RUN();
			return;
		}

		if(systemPara.RunStatus != 0)
		{
			if(systemPara.Language == 0)
				CANT_SWITCH_PAGE_MESSAGE2();
			else if(systemPara.Language == 1)
				EN_CANT_SWITCH_PAGE_MESSAGE2();

			return;
		}

		switchMode(1);
		break;
	case 2:
		if(systemPara.status == STATUS_FREERUN && systemPara.Initstatus == 1)
		{
			if(systemPara.Language == 0)
				POP_UP_INFO_AGING_RUN();
			else if(systemPara.Language == 1)
				EN_POP_UP_INFO_AGING_RUN();
			return;
		}

		switchMode(0);
		break;
	case 3:
		if(systemPara.Initstatus == 1)
		{
			if(systemPara.Language == 0)
				CANT_SWITCH_PAGE_MESSAGE();
			else if(systemPara.Language == 1)
				EN_CANT_SWITCH_PAGE_MESSAGE();
			return;
		}

		#ifdef CONF_EXTRIG_INT
		    EXT_interruptMode(EXT_INT_MODE_NONE);
		#endif
		//空跑模式关闭交互信号
		EXO_sigOFF(1);
		EXO_sigOFF(2);
		EXO_sigOFF(4);
		EXO_sigOFF(5);
		if(systemPara.Language == 0)
			MAIN_INFO_SW2AGING();
		else if(systemPara.Language == 1)
			EN_MAIN_INFO_SW2AGING();
		systemPara.status = STATUS_FREERUN;
		AT24CXX_WriteOneByte(STORAGE_WORKMODE_ADDR, systemPara.status);
		GUI_switchModeDisp(systemPara.status);
		break;
	}
}



void stopMotor1(void *pv)
{
    ADDR_6000_L[35] = 0;

    MC_motorStopDec(0);
}

void stopMotor2(void *pv)
{
    ADDR_6000_L[36] = 0;

    MC_motorStopDec(1);
}

void stopMotor3(void *pv)
{
    ADDR_6000_L[37] = 0;

    MC_motorStopDec(2);
}

void stopMotor4(void *pv)
{
    ADDR_6000_L[38] = 0;

    MC_motorStopDec(3);
}

void CH1_CW(void *pv)
{
    ADDR_6000_L[18] = 0;

    MC_motorMoveForever(0,systemPara.MotorGivenDir);
}

void CH1_CCW(void *pv)
{
    ADDR_6000_L[19] = 0;

    MC_motorMoveForever(0,!systemPara.MotorGivenDir);
}

void CH2_CW(void *pv)
{
    ADDR_6000_L[20] = 0;

    MC_motorMoveForever(1,systemPara.MotorLetDir);
}

void CH2_CCW(void *pv)
{
    ADDR_6000_L[21] = 0;

    MC_motorMoveForever(1,!systemPara.MotorLetDir);
}

void CH3_CW(void *pv)
{
    ADDR_6000_L[22] = 0;

    MC_motorMoveForever(2,systemPara.MotorUpShouDir);
}

void CH3_CCW(void *pv)
{
    ADDR_6000_L[23] = 0;

    MC_motorMoveForever(2,!systemPara.MotorUpShouDir);
}

void CH4_CW(void *pv)
{
    ADDR_6000_L[24] = 0;

    MC_motorMoveForever(3,systemPara.MotorDownShouDir);
}

void CH4_CCW(void *pv)
{
    ADDR_6000_L[25] = 0;

    MC_motorMoveForever(3,!systemPara.MotorDownShouDir);
}

//剥料控制
void start_DebugHCRST(void *pv)
{
	uint32_t distance,distance1;
	static uint8_t time1 = 0;
	distance1 = ((ADDR_5000_H[7] << 8) | ADDR_5000_L[7]);

	if(EmergencyStop() == 1)
		return;

	if(systemPara.RunStatus == 0 )
	{
		switch (*((uint8_t*) pv))
		{
		case 1:
			if(systemPara.Initstatus == STATUS_UNINIT)
			{
				if(systemPara.Language == 0)
					CANT_SWITCH_PAGE_MESSAGE0();
				else if(systemPara.Language == 1)
					EN_CANT_SWITCH_PAGE_MESSAGE0();
				return;
			}

			if(systemPara.RunStatus == 0)
			{


			}
			else
			{
				if(systemPara.Language == 0)
					CANT_SWITCH_PAGE_MESSAGE2();
				else if(systemPara.Language == 1)
					EN_CANT_SWITCH_PAGE_MESSAGE2();
			}
			break;
		case 2:

			break;
		case 3:

			break;
		case 4:
			if(IBIO_getInput(1) == 0)
			{

			}
			else
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("请先复位.", 9);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Rest Please.", 12);
			}
			break;
		}
	}
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


void start_mainTake(void *pv)
{
	ADDR_6000_L[10] = 0;

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
		systemPara.doTaking = 1;
	}
	else
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE2();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE2();
	}
}

void start_mainLabel(void *pv)
{
	ADDR_6000_L[11] = 0;

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
		systemPara.doLabeling = 1;
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
}

void start_mainHL(void *pv)
{
	ADDR_6000_L[44] = 0;
}
//调试页面取标动作和到取标位
void start_DebugTake(void *pv)
{
	ADDR_6000_L[26] = 0;
	if(systemPara.Initstatus == STATUS_UNINIT)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE0();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE0();
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
	switch (*((uint8_t*) pv))
	{
	case 1:
		systemPara.doTaking = 1;
		break;
	case 2:
		IBIO_setOutput(3, 0);
		IBIO_setOutput(4, 1);
		GUI_setOutputSignalColorDisp(2,0);
		GUI_setOutputSignalColorDisp(3,1);
//		systemPara.doTakePosi = 1;
		break;
	}

}

//调试页面贴标动作、到贴标位
void start_DebugLabel(void *pv)
{
	ADDR_6000_L[27] = 0;
	if(systemPara.Initstatus == STATUS_UNINIT)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE0();
		else
			EN_CANT_SWITCH_PAGE_MESSAGE0();
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
	switch (*((uint8_t*) pv))
	{
	case 1:
		systemPara.doLabeling = 1;
		break;
	case 2:
		IBIO_setOutput(3, 1);
		IBIO_setOutput(4, 0);
		GUI_setOutputSignalColorDisp(2,1);
		GUI_setOutputSignalColorDisp(3,0);
//		systemPara.doLabelPosi = 1;
		break;
	}
}


void start_DebugRst(void *pv)
{
	ADDR_6000_L[13] = 0;
}

void start_DebugUpSL(void *pv)
{
	ADDR_6000_L[28] = 0;
	uint32_t limitPulseCount;
    static uint8_t currentStatus=0;

	switch (*((uint8_t*) pv))
	{
	case 1:
		if(motor_ch[UP_MOTOR].status == Motor_Stop)
		{
			limitPulseCount = (ADDR_5000_H[7]<<8)|ADDR_5000_L[7];
			systemPara.UPSLmode = 1;//传感模式
			MC_motorMoveDistance(UP_MOTOR, systemPara.MotorUpShouDir, limitPulseCount);
		}
		break;
	case 2:
	    if(currentStatus)
	    {
	    	currentStatus = 0;

	    	MC_motorStop(UP_MOTOR);
	    }
	    else
	    {
	    	currentStatus = 1;
	    	systemPara.UPSLmode = 0;
	    	MC_motorMoveForever(UP_MOTOR,systemPara.MotorUpShouDir);
	    }

	    GUI_sendWord(0x7690, 0, currentStatus);
		break;
	}
}


void start_DebugQG(void *pv)
{

	ADDR_6000_L[29] = 0;

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
	case 5:
		if(IBIO_getOutput(5) == 1)
		{
			IBIO_setOutput(5, 0);
			GUI_setOutputSignalColorDisp(4,0);
		}
		else
		{
			IBIO_setOutput(5, 1);
			GUI_setOutputSignalColorDisp(4,1);
		}
		break;
	}
}



void LableSelect(void *pv)
{
    switch(ADDR_6000_L[44])
    {
        case 1:
            GUI_sendWord(SINGAL_LABLE1_ADDR,0,1);
            GUI_sendWord(SINGAL_LABLE2_ADDR,0,0);

            systemPara.Lableposition = 1;
            AT24CXX_WriteOneByte(0x44,1);
            break;

        case 2:
            GUI_sendWord(SINGAL_LABLE1_ADDR,0,0);
            GUI_sendWord(SINGAL_LABLE2_ADDR,0,1);

            systemPara.Lableposition = 2;
            AT24CXX_WriteOneByte(0x44,2);
            break;
    }

    ADDR_6000_L[44] = 0;
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
    	systemPara.Language = 1;
		AT24CXX_WriteOneByte(STORAGE_LANGUAGE_ADDR,1);
		GUI_mainMessageDisp("The language selection is Chinese.", 34);
	    GUI_initDisplay();
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
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(EXINPUT_04_NAME_ADDR,CONF_EXI04_NAME,CONF_EXI04_NAME_LEN);
				GUI_showTextIsolate(EXINPUT_05_NAME_ADDR,CONF_EXI05_NAME,CONF_EXI05_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(EXINPUT_04_NAME_ADDR,EN_CONF_EXI04_NAME,EN_CONF_EXI04_NAME_LEN);
				GUI_showTextIsolate(EXINPUT_05_NAME_ADDR,EN_CONF_EXI05_NAME,EN_CONF_EXI05_NAME_LEN);
			}
		}
		else
		{
			systemPara.isAutoLetMetalEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_1_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_LET_EN_ADDR, 1);
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(EXINPUT_04_NAME_ADDR,CONF_EXI4_NAME,CONF_EXI4_NAME_LEN);
				GUI_showTextIsolate(EXINPUT_05_NAME_ADDR,CONF_EXI5_NAME,CONF_EXI5_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(EXINPUT_04_NAME_ADDR,EN_CONF_EXI4_NAME,EN_CONF_EXI4_NAME_LEN);
				GUI_showTextIsolate(EXINPUT_05_NAME_ADDR,EN_CONF_EXI5_NAME,EN_CONF_EXI5_NAME_LEN);
			}
		}
		break;
	case 2:
		if(systemPara.isUpShouEnable)
		{
			systemPara.isUpShouEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_2_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_UP_EN_ADDR, 0);
		}
		else
		{
			systemPara.isUpShouEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_2_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_UP_EN_ADDR, 1);
		}
		break;
	case 3:
		if(systemPara.isDownShouEnable)
		{
			systemPara.isDownShouEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_3_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_DOWN_EN_ADDR, 0);
		}
		else
		{
			systemPara.isDownShouEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_3_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_DOWN_EN_ADDR, 1);
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
			//待定

		}
		else
		{
			systemPara.isInitSwOnLINEEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_7_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_INITSWONLINE_EN_ADDR, 1);
			//待定
		}
		break;
	case 20:
		if(systemPara.MotorGivenDir)
		{
			systemPara.MotorGivenDir = 0;
			GUI_sendWord(0x7940, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_GIVEN_DIR_ADDR, 0);
		}
		else
		{
			systemPara.MotorGivenDir = 1;
			GUI_sendWord(0x7940, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_GIVEN_DIR_ADDR, 1);
		}
		break;
	case 9:
		if(systemPara.MotorLetDir)
		{
			systemPara.MotorLetDir = 0;
			GUI_sendWord(MOTOR2DIR_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_LET_DIR_ADDR, 0);
		}
		else
		{
			systemPara.MotorLetDir = 1;
			GUI_sendWord(MOTOR2DIR_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_LET_DIR_ADDR, 1);
		}
		break;
	case 10:
		if(systemPara.MotorUpShouDir)
		{
			systemPara.MotorUpShouDir = 0;
			GUI_sendWord(MOTOR3DIR_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_UpShou_DIR_ADDR, 0);
		}
		else
		{
			systemPara.MotorUpShouDir = 1;
			GUI_sendWord(MOTOR3DIR_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_UpShou_DIR_ADDR, 1);
		}
		break;
	case 23:
		if(systemPara.MotorDownShouDir)
		{
			systemPara.MotorDownShouDir = 0;
			GUI_sendWord(0x7943, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_DownShow_DIR_ADDR, 0);
		}
		else
		{
			systemPara.MotorDownShouDir = 1;
			GUI_sendWord(0x7943, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_DownShow_DIR_ADDR, 1);
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
