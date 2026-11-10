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

void motorControl(void *pv);

void GMOffsetEnter(void *pv);
void start_DebugSL(void *pv);
void start_DebugFL(void *pv);
void start_DebugUpSL(void *pv);
void start_DebugDownSL(void *pv);
void start_DebugTL(void *pv);
void start_DebugHCRST(void *pv);
void start_DebugCylinder(void *pv);

void startupInit(void *pv);
void start_mainSL(void *pv);
void start_mainBL(void *pv);
void start_mainJB(void *pv);

void start_mainHC(void *pv);
void start_mainRST(void *pv);
void DOWNFilmSensorSelect(void *pv);
void UPFilmSensorSelect(void *pv);
void sensorSelect(void *pv);
void givenModeSelect(void *pv);

void Language_Selection(void *pv);

void resetFeederPara(void *pv);
void functionChose(void *pv);
void offsetSelect(void *pv);

uint8_t Rst2Feedflag = 0;

static uint8_t passwordCompare(uint8_t which, uint32_t pw);


EventCallback eventCallback[] = {runStatusControl,              // 0
                                checkPassword,                  // 1
                                returnPage,                     // 2
                                fillUserName,                   // 3
								jumpPage,        				// 4 切换页面
                                logout,                         // 5
                                changePassword,                 // 6
								selectMode,                 	// 7
								motorControl,        			// 8
								startupInit,          			// 9
								start_mainSL,					// 10 A
								start_mainBL,          			// 11 B
								start_mainJB,     				// 12 C
								konghanshu,          			// 13 D
								konghanshu,                   	// 14 E
								konghanshu,            			// 15 F
								start_DebugSL,                  // 16 10
								start_DebugHCRST,               // 17 11
								start_DebugFL,                  // 18 12
								start_DebugUpSL,                // 19 13
								start_DebugDownSL,              // 20 14
								start_DebugTL,                  // 21 15
								konghanshu,                     // 22 16
								huiyuandian_UI,                 // 23 17
								konghanshu,                     // 24 18
								konghanshu,                     // 25 19
								start_DebugCylinder,            // 26 1A
								konghanshu,               		// 27 1B
								konghanshu,                		// 28 1C
								konghanshu,                     // 29 1D
								konghanshu,                  	// 30 1E
								konghanshu,                 	// 31 2F
								konghanshu,                 	// 32 20
								konghanshu,                		// 33 21
								konghanshu,                 	// 34 22
								konghanshu,                     // 35 23
								konghanshu,                     // 36 24
								konghanshu,                     // 37 25
								konghanshu,                     // 38 26
                                controlOutputSignal,            // 39 27
                                offsetSelect,                   // 40 28
								konghanshu,						//41 29
								konghanshu,           			// 42 2A
								konghanshu,             		// 43 2B
								konghanshu,                   	// 44 2C
								konghanshu,                		// 45 2D
                                resetFeederPara,                // 46 2E
								functionChose,					// 47 2F
								Language_Selection,				// 48 30
};


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
    //密码转换
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
                GUI_showText(LOGGINSTATE_DISP_ADDR,"User    ",8);
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
                GUI_showText(LOGGINSTATE_DISP_ADDR,"Vendor  ",8);
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
 * @brief 判断密码
 * @param pv
 */

static uint8_t passwordCompare(uint8_t which, uint32_t pw)
{
    uint8_t rtl = 1;
    uint8_t i;

    switch(which)
    {
    case 0:
        for(i=6;i>0;i--)
        {//整数转换为字符一个个比较
            if(((pw % 10) + '0') != systemPara.userPassword[i-1])
            {
                rtl = 0;
                break;
            }
            pw /= 10;//去掉最低位
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


void motorControl(void *pv)
{
	if(EmergencyStop() == 1)
		return;

	switch(*((uint8_t*) pv))
	{
	case 0x01:
		MC_motorStop(0);
		LV8731V_cmd(0, 0);
		break;
	case 0x02:
		MC_motorStop(1);
		LV8731V_cmd(1, 0);
		break;
	case 0x03:
		MC_motorStop(2);
		LV8731V_cmd(2, 0);
		break;
	case 0x04:
		MC_motorStop(3);
		LV8731V_cmd(3, 0);
		break;
	}
}

//剥料控制
void start_DebugHCRST(void *pv)
{
	uint32_t distance,distance1;
	static uint8_t time1 = 0;

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
				systemPara.Debug = 2;
				systemPara.stickOnceTriggerByUIorIO = 1;

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
			if(IBIO_getInput(1) == 0)
			{
				distance = MC_mm2pulse(BO_MOTOR,((ADDR_5000_H[2] << 8) | ADDR_5000_L[2]));
				MC_motorRunAndOffsetByEXTI(BO_MOTOR, systemPara.MotorBoDir, 0,distance);
			}
			else
			{
				if(systemPara.Language == 0)
					GUI_mainMessageDisp("请先复位.", 9);
				else if(systemPara.Language == 1)
					GUI_mainMessageDisp("Rest Please.", 12);
			}
			break;
		case 3:
			Stick_Position(0);
			break;
		case 4:
			if(IBIO_getInput(1) == 0)
			{
				if(distance1 > 0)
				{
					MC_motorMoveDistance(BO_MOTOR, systemPara.MotorBoDir, distance1);
				}
				else
				{
					if(systemPara.Language == 0)
						GUI_mainMessageDisp("拍照位置参数未设置.", 19);
					else if(systemPara.Language == 1)
						GUI_mainMessageDisp("The photo position parameter is not set.", 40);
				}
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
 * 退料动作和连续退料在同一个函数
 */
void start_DebugTL(void *pv)
{
	uint32_t limitPulseCount;
	static uint8_t currentStatus=0;

	if(EmergencyStop() == 1)
		return;

	switch (*((uint8_t*) pv))
	{
	case 1:

		limitPulseCount = (ADDR_5000_H[1] << 8) | ADDR_5000_L[1];

		MC_motorMoveDistance1(GIVEN_MOTOR, !systemPara.MotorGivenDir, limitPulseCount);

		break;
	case 2:
		if(currentStatus)
		{
			MC_motorStopDec(GIVEN_MOTOR);
			currentStatus = 0;
		}
		else
		{
			MC_motorMoveForever(GIVEN_MOTOR, !systemPara.MotorGivenDir);
			currentStatus = 1;
		}

		GUI_sendWord(CTNTL_FLAG_ADDR, 0, currentStatus);
		break;
	}
}

/**
 * 放料动作和连续放料在同一个函数
 */
void start_DebugFL(void *pv)
{
	ADDR_6000_L[11] = 0;
	uint32_t limitPulseCount;
	static uint8_t currentStatus = 0;

	if(EmergencyStop() == 1)
		return;

	if(!systemPara.isDownShouEnable)
	{
		if(systemPara.Language == 0)
			GUI_showMessage("下收料功能未启用！",18);
		else if(systemPara.Language == 1)
			GUI_showMessage("DOWN Recycle film feature not enabled.", 36);
		return;
	}

	switch (*((uint8_t*) pv))
	{
	case 1:
		limitPulseCount = (ADDR_5000_H[3]<<8)|ADDR_5000_L[3];

		if(motor_ch[LET_MOTOR].status == Motor_Stop && IBIO_getInput(8) != 0)
		{
			systemPara.FLmode = 1;
			MC_motorMoveDistance1(LET_MOTOR,systemPara.MotorLetDir,limitPulseCount);
		}
		break;
	case 2:
		if(currentStatus)
		{
			MC_motorStopDec(LET_MOTOR);
			currentStatus = 0;
		}
		else
		{
			MC_motorMoveForever(LET_MOTOR, systemPara.MotorLetDir);
			currentStatus = 1;
		}
		GUI_sendWord(CTNFL_FLAG_ADDR, 0, currentStatus);
		break;
	}
}

void start_DebugUpSL(void *pv)
{
	ADDR_6000_L[14] = 0;
	uint32_t limitPulseCount;
	static uint8_t currentStatus=0;

	if(EmergencyStop() == 1)
		return;

	if(!systemPara.isUpShouEnable)
	{
		if(systemPara.Language == 0)
			GUI_showMessage("上收料功能未启用！",18);
		else if(systemPara.Language == 1)
			GUI_showMessage("UP Recycle film feature not enabled.", 36);
		return;
	}

	switch (*((uint8_t*) pv))
	{
	case 1:
		limitPulseCount = (ADDR_5000_H[7]<<8)|ADDR_5000_L[7];
		if(motor_ch[UP_MOTOR].status == Motor_Stop && IBIO_getInput(6) != 0)
		{
			systemPara.UPSLmode = 1;
			MC_motorMoveDistance1(UP_MOTOR, systemPara.MotorShouDir, limitPulseCount);
		}
		break;
	case 2:
		if(currentStatus)
		{
			currentStatus = 0;

			MC_motorStopDec(UP_MOTOR);
		}
		else
		{
			currentStatus = 1;
			systemPara.UPSLmode = 0;
			MC_motorMoveForever(UP_MOTOR, systemPara.MotorShouDir);
		}

		GUI_sendWord(CTNUPSL_FLAG_ADDR, 0, currentStatus);
		break;
	}
}

void start_DebugDownSL(void *pv)
{
	ADDR_6000_L[16] = 0;
	uint32_t limitPulseCount;
	static uint8_t currentStatus = 0;

	if(EmergencyStop() == 1)
		return;

	switch (*((uint8_t*) pv))
	{
	case 1:
		limitPulseCount = (ADDR_5000_H[5]<<8)|ADDR_5000_L[5];
		if(motor_ch[DOWN_MOTOR].status == Motor_Stop)
			MC_motorMoveDistance1(DOWN_MOTOR, 0, limitPulseCount);
		break;
	case 2:
		if (currentStatus)
		{
			currentStatus = 0;

			MC_motorStopDec(DOWN_MOTOR);
		}
		else
		{
			currentStatus = 1;

			MC_motorMoveForever(DOWN_MOTOR, systemPara.MotorShouDir);
		}

		GUI_sendWord(CTNDOWNS_FLAGL_ADDR, 0, currentStatus);
		break;
	}
}

/**
 执行送料动作
*/
void start_DebugSL(void *pv)
{
	uint32_t distance;
	int count;

	if(EmergencyStop() == 1)
		return;

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
			systemPara.Debug = 1;
			systemPara.pickOnceTriggerByUIorIO = 1;

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
//		distance = MC_mm2pulse(BO_MOTOR,((ADDR_5000_H[1] << 8) | ADDR_5000_L[1]));
//		MC_motorRunAndOffsetByEXTI(BO_MOTOR, systemPara.MotorBoDir, 0,distance);
		Pick_Position(0);
		break;
	case 3:
		systemPara.doGohome = 1;
		break;
	}
}

void start_DebugCylinder(void *pv)
{
	if(EmergencyStop() == 1)
		return;

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
	case 6:
		if(IBIO_getOutput(6) == 1)
		{
			IBIO_setOutput(6, 0);
			GUI_setOutputSignalColorDisp(5,0);
		}
		else
		{
			IBIO_setOutput(6, 1);
			GUI_setOutputSignalColorDisp(5,1);
		}
		break;
	case 7:
		if(IBIO_getOutput(7) == 1)
		{
			IBIO_setOutput(7, 0);
			GUI_setOutputSignalColorDisp(6,0);
		}
		else
		{
			IBIO_setOutput(7, 1);
			GUI_setOutputSignalColorDisp(6,1);
		}
		break;
	case 8:
		if(IBIO_getOutput(8) == 1)
		{
			IBIO_setOutput(8,0);
			GUI_setOutputSignalColorDisp(7,0);
		}
		else
		{
			IBIO_setOutput(8, 1);
			GUI_setOutputSignalColorDisp(7,1);
		}
		break;
	}
}


void GMOffsetEnter(void *pv)
{
    extern uint16_t tempGMOffset;
    
    ADDR_6000_L[41] = 0;


    AT24CXX_WriteOneByte(0x34,ADDR_5000_L[18]);
    AT24CXX_WriteOneByte(0x35,ADDR_5000_H[18]);
    
    tempGMOffset = 0;
    
    //GUI_showMessage("参数已确认",10);
}


/**
 @brief 初始化回调函数.
 执行初始化功能
*/
void startupInit(void *pv)
{
    ADDR_6000_L[26] = 0;

	if(EmergencyStop() == 1)
		return;

    if(systemPara.status == STATUS_ONLINE)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE1();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE1();
		return;
	}
    if(systemPara.status == STATUS_FREERUN && systemPara.Initstatus == 1)
	{
		if(systemPara.Language == 0)
			POP_UP_INFO_AGING_RUN();
		else if(systemPara.Language == 1)
			EN_POP_UP_INFO_AGING_RUN();
		return;
	}

	if(systemPara.RunStatus == 0)
		systemPara.doInit = 1;
	else
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE2();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE2();
	}
}

void start_mainSL(void *pv)
{
	ADDR_6000_L[42] = 0;

	if(EmergencyStop() == 1)
		return;

	if(systemPara.Initstatus == STATUS_UNINIT && systemPara.Page == 0)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE0();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE0();
		return;
	}
	if(systemPara.status == STATUS_ONLINE  )
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE1();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE1();
		return;
	}

	if(systemPara.RunStatus)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE2();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE2();
    	return;
	}

	systemPara.pickOnceTriggerByUIorIO = 1;

}

void start_mainBL(void *pv)
{
	uint32_t distance1,distance2;

	if(EmergencyStop() == 1)
		return;

	if(systemPara.Initstatus == STATUS_UNINIT)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE0();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE0();
		return;
	}
    if(systemPara.status == STATUS_ONLINE)
    {
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE1();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE1();
    	return;
    }

	if(systemPara.RunStatus)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE2();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE2();
    	return;
	}
	systemPara.stickOnceTriggerByUIorIO = 1;
}

void start_mainJB(void *pv)
{
	uint32_t distance1,distance2;

	if(EmergencyStop() == 1)
		return;

	if(systemPara.Initstatus == STATUS_UNINIT)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE0();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE0();
		return;
	}
    if(systemPara.status == STATUS_ONLINE)
    {
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE1();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE1();
    	return;
    }

	if(systemPara.RunStatus)
	{
		if(systemPara.Language == 0)
			CANT_SWITCH_PAGE_MESSAGE2();
		else if(systemPara.Language == 1)
			EN_CANT_SWITCH_PAGE_MESSAGE2();
    	return;
	}

	if(!systemPara.ReceivingOnceOK)
		systemPara.ReceivingOnceTriggerByUIorIO = 1;
	else
		systemPara.ReceivRSTOnceTriggerByUIorIO = 1;
}


#if 0
void DOWNFilmSensorSelect(void *pv)
{

	if(!systemPara.isDownShouEnable)
	{
		if(systemPara.Language == 0)
			GUI_showMessage("下收料功能未启用！",18);
		else if(systemPara.Language == 1)
			GUI_showMessage("Lower film feature not enabled.", 31);
		return;
	}

    switch(ADDR_6000_L[42])
    {
        case 1://双感应器
            GUI_sendWord(SINGAL_DUAL_SENSOR_DOWN_ADDR,0,1);
            GUI_sendWord(SINGAL_SINGLE_SENSOR_DOWN_ADDR,0,0);
            GUI_sendWord(SINGAL_NOT_SENSOR_DOWN_ADDR,0,0);

            systemPara.isDownShouSensorEnable = 1;
            AT24CXX_WriteOneByte(STORAGE_DOWNSENSORREAL_EN_ADDR,1);

			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_06_NAME,CONF_INPUT_06_NAME_LEN);
				GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_07_NAME,CONF_INPUT_07_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_06_NAME,EN_CONF_INPUT_06_NAME_LEN);
				GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_07_NAME,EN_CONF_INPUT_07_NAME_LEN);
			}
            break;

        case 2://单感应器
            GUI_sendWord(SINGAL_DUAL_SENSOR_DOWN_ADDR,0,0);
            GUI_sendWord(SINGAL_SINGLE_SENSOR_DOWN_ADDR,0,1);
            GUI_sendWord(SINGAL_NOT_SENSOR_DOWN_ADDR,0,0);

            systemPara.isDownShouSensorEnable = 2;
            AT24CXX_WriteOneByte(STORAGE_DOWNSENSORREAL_EN_ADDR,2);

			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_07_NAME,CONF_INPUT_07_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_07_NAME,EN_CONF_INPUT_07_NAME_LEN);
			}
            break;
        case 3://无感应器

            GUI_sendWord(SINGAL_DUAL_SENSOR_DOWN_ADDR,0,0);
            GUI_sendWord(SINGAL_SINGLE_SENSOR_DOWN_ADDR,0,0);
            GUI_sendWord(SINGAL_NOT_SENSOR_DOWN_ADDR,0,1);

            systemPara.isDownShouSensorEnable = 0;
            AT24CXX_WriteOneByte(STORAGE_DOWNSENSORREAL_EN_ADDR,0);

			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_006_NAME,CONF_INPUT_006_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_006_NAME,EN_CONF_INPUT_006_NAME_LEN);
			}
        	break;
    }

    ADDR_6000_L[42] = 0;
}

void UPFilmSensorSelect(void *pv)
{

	if(!systemPara.isUpShouEnable)
	{
		if(systemPara.Language == 0)
			GUI_showMessage("上收料功能未启用！",18);
		else if(systemPara.Language == 1)
			GUI_showMessage("Upper film feature not enabled.", 31);
		return;
	}

    switch(ADDR_6000_L[43])
    {
        case 1://双感应器
            GUI_sendWord(SINGAL_DUAL_SENSOR_UP_ADDR,0,1);
            GUI_sendWord(SINGAL_SINGLE_SENSOR_UP_ADDR,0,0);
            GUI_sendWord(SINGAL_NOT_SENSOR_UP_ADDR,0,0);

            systemPara.isUpShouSensorEnable = 1;
            AT24CXX_WriteOneByte(STORAGE_UPSSOR_EN_ADDR,1);

			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_04_NAME_ADDR,CONF_INPUT_04_NAME,CONF_INPUT_04_NAME_LEN);
				GUI_showTextIsolate(INPUT_05_NAME_ADDR,CONF_INPUT_05_NAME,CONF_INPUT_05_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_04_NAME_ADDR,EN_CONF_INPUT_04_NAME,EN_CONF_INPUT_04_NAME_LEN);
				GUI_showTextIsolate(INPUT_05_NAME_ADDR,EN_CONF_INPUT_05_NAME,EN_CONF_INPUT_05_NAME_LEN);
			}
            break;

        case 2://单感应器
            GUI_sendWord(SINGAL_DUAL_SENSOR_UP_ADDR,0,0);
            GUI_sendWord(SINGAL_SINGLE_SENSOR_UP_ADDR,0,1);
            GUI_sendWord(SINGAL_NOT_SENSOR_UP_ADDR,0,0);

            systemPara.isUpShouSensorEnable = 2;
            AT24CXX_WriteOneByte(STORAGE_UPSSOR_EN_ADDR,2);

			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_05_NAME_ADDR,CONF_INPUT_05_NAME,CONF_INPUT_05_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_05_NAME_ADDR,EN_CONF_INPUT_05_NAME,EN_CONF_INPUT_05_NAME_LEN);
			}
            break;
            break;
        case 3://无感应器

            GUI_sendWord(SINGAL_DUAL_SENSOR_UP_ADDR,0,0);
            GUI_sendWord(SINGAL_SINGLE_SENSOR_UP_ADDR,0,0);
            GUI_sendWord(SINGAL_NOT_SENSOR_UP_ADDR,0,1);

            systemPara.isUpShouSensorEnable = 0;
            AT24CXX_WriteOneByte(STORAGE_UPSSOR_EN_ADDR,0);

			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_04_NAME_ADDR,CONF_INPUT_004_NAME,CONF_INPUT_004_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_04_NAME_ADDR,EN_CONF_INPUT_004_NAME,EN_CONF_INPUT_004_NAME_LEN);
			}
        	break;
    }

    ADDR_6000_L[43] = 0;
}

void sensorSelect(void *pv)
{
    switch(ADDR_6000_L[44])
    {
        case 1://下光纤
            GUI_sendWord(SINGAL_SENSOR_DOWN_ADDR,0,1);
            GUI_sendWord(SINGAL_SENSOR_UP_ADDR,0,0);
            GUI_sendWord(SINGAL_SENSOR_NO_ADDR,0,0);
        
            systemPara.sensorChosen = 1;
            AT24CXX_WriteOneByte(STORAGE_SENSORSELECT_ADDR,1);
            sendBuffer[41][5] = 0x01;
            USART1_sendBuf(sendBuffer[41],8);
            if(systemPara.givenMode == 2)
            {
            	GUI_sendWord(SINGAL_GIVENMODE_N,0,1);
				GUI_sendWord(SINGAL_GIVENMODE_Y,0,0);

				systemPara.givenMode = 1;
				AT24CXX_WriteOneByte(STORAGE_GIVENMODESELECT_ADDR,1);
				sendBuffer[43][5] = 0x01;
				USART1_sendBuf(sendBuffer[43],8);
            }
            break;
        
        case 2://上光纤
        	if(systemPara.isLowspeedSensorEnable)
        	{
				if(systemPara.Language == 0)
					GUI_showMessage("传感器减速功能已开启，请关闭！",30);
				else if(systemPara.Language == 1)
					GUI_showMessage("Please turn off sensor deceleration!",36);
        		sendBuffer[41][5] = 0xff;
        		USART1_sendBuf(sendBuffer[41],8);
        		break;
        	}

            GUI_sendWord(SINGAL_SENSOR_DOWN_ADDR,0,0);
            GUI_sendWord(SINGAL_SENSOR_UP_ADDR,0,1);
            GUI_sendWord(SINGAL_SENSOR_NO_ADDR,0,0);

            systemPara.sensorChosen = 2;
            AT24CXX_WriteOneByte(STORAGE_SENSORSELECT_ADDR,2);
            sendBuffer[41][5] = 0x02;
            USART1_sendBuf(sendBuffer[41],8);

            //上光纤不支持自动送料和复位同步送料
            systemPara.isAutoSongEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_4_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_AUTOSONG_EN_ADDR, 0);
            systemPara.isRSTAutoSongEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_7_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_RST_AUTOSONG_EN_ADDR, 0);

            break;
        case 3://定长

            GUI_sendWord(SINGAL_SENSOR_DOWN_ADDR,0,0);
            GUI_sendWord(SINGAL_SENSOR_UP_ADDR,0,0);
            GUI_sendWord(SINGAL_SENSOR_NO_ADDR,0,1);

        	systemPara.sensorChosen = 3;
			AT24CXX_WriteOneByte(STORAGE_SENSORSELECT_ADDR,3);
			sendBuffer[41][5] = 0x03;
			USART1_sendBuf(sendBuffer[41],8);

			//上光纤不支持自动送料和复位同步送料
			systemPara.isAutoSongEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_4_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_AUTOSONG_EN_ADDR, 0);
			systemPara.isRSTAutoSongEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_7_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_RST_AUTOSONG_EN_ADDR, 0);
        	break;
    }
    
    ADDR_6000_L[44] = 0;
}

void givenModeSelect(void *pv)
{
    switch(ADDR_6000_L[45])
    {
        case 1://有料不可送
            GUI_sendWord(SINGAL_GIVENMODE_N,0,1);
            GUI_sendWord(SINGAL_GIVENMODE_Y,0,0);
        
            systemPara.givenMode = 1;
            AT24CXX_WriteOneByte(STORAGE_GIVENMODESELECT_ADDR,1);
            sendBuffer[43][5] = 0x01;
            USART1_sendBuf(sendBuffer[43],8);
            break;
        
        case 2://有料可送
        	if(systemPara.sensorChosen == 1)
        	{
				if(systemPara.Language == 0)
					GUI_showMessage("当前感应器模式不可选此模式！",28);
				else if(systemPara.Language == 1)
					GUI_showMessage("Feed mode does not support this feature.",40);
        		sendBuffer[41][5] = 0xff;
        		USART1_sendBuf(sendBuffer[41],8);
        		break;
        	}
        	else
        	{
				GUI_sendWord(SINGAL_GIVENMODE_N,0,0);
				GUI_sendWord(SINGAL_GIVENMODE_Y,0,1);

				systemPara.givenMode = 2;
				AT24CXX_WriteOneByte(STORAGE_GIVENMODESELECT_ADDR,2);
				sendBuffer[41][5] = 0x02;
				USART1_sendBuf(sendBuffer[41],8);
        	}
            break;
    }
    
    ADDR_6000_L[45] = 0;
}


#endif

void Language_Selection(void *pv)
{
	if(systemPara.Language == 0)
	{
    	systemPara.Language = 1;
		AT24CXX_WriteOneByte(STORAGE_LANGUAGE_ADDR,1);
	    GUI_initDisplay();
	}
	else if(systemPara.Language == 1)
	{
    	systemPara.Language = 0;
		AT24CXX_WriteOneByte(STORAGE_LANGUAGE_ADDR,0);
	    GUI_initDisplay();
    }

	if(systemPara.logginStatus >= 11 && systemPara.status == STATUS_AUTO)
	{
		GUI_switchPage(0);
	}
	else
	{//其他
		GUI_switchPage(32);
	}
}

void resetFeederPara(void *pv)
{
    ADDR_6000_L[46] = 0;
    
    GUI_showText(INFO_DISP_ADDR,"                    ",20);
    if(systemPara.Language == 0)
    	GUI_showText(INFO_DISP_ADDR,"正在恢复出厂参数...",19);
    else if(systemPara.Language == 1)
    	GUI_showText(INFO_DISP_ADDR,"Restoring factory parameters...",31);
    pageNoBeforMessage = 12;

    GUI_switchPage(12);
    
    GUI_showText(INFO_DISP_ADDR,"                                        ",46);

    if(systemPara.Language == 0)
    	GUI_showText(INFO_DISP_ADDR,"恢复完成, 请断电重启！",24);
    else if(systemPara.Language == 1)
       GUI_showText(INFO_DISP_ADDR,"Please power off and restart!",29);

    //恢复供料器参数
    if(systemPara.logginStatus == 11)
    	STORAGE_resetFeederParam();
    else
    	STORAGE_setDefaultParameter();
    
    systemPara.logginStatus = 0;


}

void functionChose(void *pv)
{
	switch(*((uint8_t *)pv))
	{
#if 0
	case 1:
		if(systemPara.isAutoLetMetalEnable)
		{
			systemPara.isAutoLetMetalEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_1_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_LET_EN_ADDR, 0);
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_006_NAME,CONF_INPUT_006_NAME_LEN);
				GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_007_NAME,CONF_INPUT_007_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_006_NAME,EN_CONF_INPUT_006_NAME_LEN);
				GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_007_NAME,EN_CONF_INPUT_007_NAME_LEN);
			}
		}
		else
		{
			systemPara.isAutoLetMetalEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_1_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_LET_EN_ADDR, 1);
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_06_NAME,CONF_INPUT_06_NAME_LEN);
				GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_07_NAME,CONF_INPUT_07_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_06_NAME,EN_CONF_INPUT_06_NAME_LEN);
				GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_07_NAME,EN_CONF_INPUT_07_NAME_LEN);
			}
		}
		break;
	case 2:
		if(systemPara.isUpShouEnable)
		{
			systemPara.isUpShouEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_2_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_UP_EN_ADDR, 0);
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_04_NAME_ADDR,CONF_INPUT_004_NAME,CONF_INPUT_004_NAME_LEN);
				GUI_showTextIsolate(INPUT_05_NAME_ADDR,CONF_INPUT_005_NAME,CONF_INPUT_005_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_04_NAME_ADDR,EN_CONF_INPUT_004_NAME,EN_CONF_INPUT_004_NAME_LEN);
				GUI_showTextIsolate(INPUT_05_NAME_ADDR,EN_CONF_INPUT_005_NAME,EN_CONF_INPUT_005_NAME_LEN);
			}
		}
		else
		{
			systemPara.isUpShouEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_2_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_UP_EN_ADDR, 1);
			if(systemPara.Language == 0)
			{
				if(systemPara.isUpShouSensorEnable == 1){
					GUI_showTextIsolate(INPUT_04_NAME_ADDR,CONF_INPUT_04_NAME,CONF_INPUT_04_NAME_LEN);
					GUI_showTextIsolate(INPUT_05_NAME_ADDR,CONF_INPUT_05_NAME,CONF_INPUT_05_NAME_LEN);
				}
				else if(systemPara.isUpShouSensorEnable == 2){
					GUI_showTextIsolate(INPUT_04_NAME_ADDR,CONF_INPUT_004_NAME,CONF_INPUT_004_NAME_LEN);
					GUI_showTextIsolate(INPUT_05_NAME_ADDR,CONF_INPUT_05_NAME,CONF_INPUT_05_NAME_LEN);
				}
				else if(systemPara.isUpShouSensorEnable == 0){
					GUI_showTextIsolate(INPUT_04_NAME_ADDR,CONF_INPUT_004_NAME,CONF_INPUT_004_NAME_LEN);
					GUI_showTextIsolate(INPUT_05_NAME_ADDR,CONF_INPUT_005_NAME,CONF_INPUT_005_NAME_LEN);
				}
			}
			else if(systemPara.Language == 1)
			{
				if(systemPara.isUpShouSensorEnable == 1){
					GUI_showTextIsolate(INPUT_04_NAME_ADDR,EN_CONF_INPUT_04_NAME,EN_CONF_INPUT_04_NAME_LEN);
					GUI_showTextIsolate(INPUT_05_NAME_ADDR,EN_CONF_INPUT_05_NAME,EN_CONF_INPUT_05_NAME_LEN);
				}
				else if(systemPara.isUpShouSensorEnable == 2){
					GUI_showTextIsolate(INPUT_04_NAME_ADDR,EN_CONF_INPUT_004_NAME,EN_CONF_INPUT_004_NAME_LEN);
					GUI_showTextIsolate(INPUT_05_NAME_ADDR,EN_CONF_INPUT_05_NAME,EN_CONF_INPUT_05_NAME_LEN);
				}
				else if(systemPara.isUpShouSensorEnable == 0){
					GUI_showTextIsolate(INPUT_04_NAME_ADDR,EN_CONF_INPUT_004_NAME,EN_CONF_INPUT_004_NAME_LEN);
					GUI_showTextIsolate(INPUT_05_NAME_ADDR,EN_CONF_INPUT_005_NAME,EN_CONF_INPUT_005_NAME_LEN);
				}
			}
		}
		break;
	case 3:
		if(systemPara.isDownShouEnable)
		{
			systemPara.isDownShouEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_3_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_DOWN_EN_ADDR, 0);
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_006_NAME,CONF_INPUT_006_NAME_LEN);
				GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_007_NAME,CONF_INPUT_007_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_006_NAME,EN_CONF_INPUT_006_NAME_LEN);
				GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_007_NAME,EN_CONF_INPUT_007_NAME_LEN);
			}
		}
		else
		{
			systemPara.isDownShouEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_3_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_DOWN_EN_ADDR, 1);
			if(systemPara.Language == 0)
			{
				if(systemPara.isDownShouSensorEnable == 1)
				{
					GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_06_NAME,CONF_INPUT_06_NAME_LEN);
					GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_07_NAME,CONF_INPUT_07_NAME_LEN);
				}
				else if(systemPara.isDownShouSensorEnable == 2)
				{
					GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_006_NAME,CONF_INPUT_006_NAME_LEN);
					GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_07_NAME,CONF_INPUT_07_NAME_LEN);
				}
				else if(systemPara.isDownShouSensorEnable == 0)
				{
					GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_006_NAME,CONF_INPUT_006_NAME_LEN);
					GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_07_NAME,CONF_INPUT_07_NAME_LEN);
				}
			}
			else if(systemPara.Language == 1)
			{
				if(systemPara.isDownShouSensorEnable == 1)
				{
					GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_06_NAME,EN_CONF_INPUT_06_NAME_LEN);
					GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_07_NAME,EN_CONF_INPUT_07_NAME_LEN);
				}
				else if(systemPara.isDownShouSensorEnable == 2)
				{
					GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_006_NAME,EN_CONF_INPUT_006_NAME_LEN);
					GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_07_NAME,EN_CONF_INPUT_07_NAME_LEN);
				}
				else if(systemPara.isDownShouSensorEnable == 0)
				{
					GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_006_NAME,EN_CONF_INPUT_006_NAME_LEN);
					GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_07_NAME,EN_CONF_INPUT_07_NAME_LEN);
				}
			}
		}
		break;
#endif
	case 4:
		if(systemPara.isAutoSongEnable)
		{
			systemPara.isAutoSongEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_4_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_AUTOSONG_EN_ADDR, 0);
		}
		else
		{
			if(systemPara.sensorChosen == 2 || systemPara.sensorChosen == 3)
			{
				if(systemPara.Language == 0)
					GUI_showMessage("当前感应器模式不可选此模式！",28);
				else if(systemPara.Language == 1)
					GUI_showMessage("Feed mode does not support this feature.",40);
				break;
			}
			if(systemPara.isGivenEndNoAlarmEnable)
			{
				if(systemPara.Language == 0)
					GUI_showMessage("送料失败不切换停止已开启",24);
				else if(systemPara.Language == 1)
					GUI_showMessage("Feeding failure doesn't switch stop is enabled",46);
				break;
			}
			systemPara.isAutoSongEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_4_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_AUTOSONG_EN_ADDR, 1);
		}
		break;
#if 0
	case 5:
		if(systemPara.isCylinderSensorEnable)
		{
			systemPara.isCylinderSensorEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_5_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_CYLINDERSSOR_EN_ADDR, 0);
			if(systemPara.Language == 0)
			{
//				//字体
				GUI_showTextIsolate(SIGNAL_NAME_3_ADDR,CONF_INNER_CTRL_NAME_03,CONF_INNER_CTRL_NAME_3_LEN);
				GUI_showTextIsolate(INPUT_08_NAME_ADDR,CONF_INPUT_008_NAME,CONF_INPUT_008_NAME_LEN);
				GUI_showTextIsolate(INPUT_09_NAME_ADDR,CONF_INPUT_009_NAME,CONF_INPUT_009_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(SIGNAL_NAME_3_ADDR,EN_CONF_INNER_CTRL_NAME_03,CONF_INNER_CTRL_NAME_3_LEN);
				GUI_showTextIsolate(INPUT_08_NAME_ADDR,EN_CONF_INPUT_008_NAME,EN_CONF_INPUT_008_NAME_LEN);
				GUI_showTextIsolate(INPUT_09_NAME_ADDR,EN_CONF_INPUT_009_NAME,EN_CONF_INPUT_009_NAME_LEN);
			}
		}
		else
		{
			if(systemPara.isUpSensorIsRealtime)
			{
				if(systemPara.Language == 0)
					GUI_showMessage("上光纤实时输出有料信号已开启！",30);
				else if(systemPara.Language == 1)
					GUI_showMessage("Optical fiber real-time output signal is ON.",43);
				break;
			}
			systemPara.isCylinderSensorEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_5_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_CYLINDERSSOR_EN_ADDR, 1);
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(SIGNAL_NAME_3_ADDR,CONF_INNER_CTRL_NAME_3,CONF_INNER_CTRL_NAME_3_LEN);
				GUI_showTextIsolate(INPUT_08_NAME_ADDR,CONF_INPUT_08_NAME,CONF_INPUT_08_NAME_LEN);
				GUI_showTextIsolate(INPUT_09_NAME_ADDR,CONF_INPUT_09_NAME,CONF_INPUT_09_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(SIGNAL_NAME_3_ADDR,EN_CONF_INNER_CTRL_NAME_3,EN_CONF_INNER_CTRL_NAME_3_LEN);
				GUI_showTextIsolate(INPUT_08_NAME_ADDR,EN_CONF_INPUT_08_NAME,EN_CONF_INPUT_08_NAME_LEN);
				GUI_showTextIsolate(INPUT_09_NAME_ADDR,EN_CONF_INPUT_09_NAME,EN_CONF_INPUT_09_NAME_LEN);
			}
		}
		break;
//	case 6:
//		if(systemPara.isUpShouSensorEnable)
//		{
//			systemPara.isUpShouSensorEnable = 0;
//			GUI_sendWord(FUNCTION_CHOSE_6_ADDR, 0, 0);
//			AT24CXX_WriteOneByte(STORAGE_UPSSOR_EN_ADDR, 0);
//			if(systemPara.Language == 0)
//			{
//				GUI_showTextIsolate(INPUT_04_NAME_ADDR,CONF_INPUT_004_NAME,CONF_INPUT_004_NAME_LEN);
//				GUI_showTextIsolate(INPUT_05_NAME_ADDR,CONF_INPUT_005_NAME,CONF_INPUT_005_NAME_LEN);
//			}
//			else if(systemPara.Language == 1)
//			{
//				GUI_showTextIsolate(INPUT_04_NAME_ADDR,EN_CONF_INPUT_004_NAME,EN_CONF_INPUT_004_NAME_LEN);
//				GUI_showTextIsolate(INPUT_05_NAME_ADDR,EN_CONF_INPUT_005_NAME,EN_CONF_INPUT_005_NAME_LEN);
//			}
//		}
//		else
//		{
//			if(!systemPara.isUpShouEnable)
//			{
//				if(systemPara.Language == 0)
//					GUI_showMessage("收料功能未启用！",24);
//				else if(systemPara.Language == 1)
//					GUI_showMessage("Recycle film feature not enabled.", 33);
//				break;
//			}
//			systemPara.isUpShouSensorEnable = 1;
//			GUI_sendWord(FUNCTION_CHOSE_6_ADDR, 0, 1);
//			AT24CXX_WriteOneByte(STORAGE_UPSSOR_EN_ADDR, 1);
//			if(systemPara.Language == 0)
//			{
//				GUI_showTextIsolate(INPUT_04_NAME_ADDR,CONF_INPUT_04_NAME,CONF_INPUT_04_NAME_LEN);
//				GUI_showTextIsolate(INPUT_05_NAME_ADDR,CONF_INPUT_05_NAME,CONF_INPUT_05_NAME_LEN);
//			}
//			else if(systemPara.Language == 1)
//			{
//				GUI_showTextIsolate(INPUT_04_NAME_ADDR,EN_CONF_INPUT_04_NAME,EN_CONF_INPUT_04_NAME_LEN);
//				GUI_showTextIsolate(INPUT_05_NAME_ADDR,EN_CONF_INPUT_05_NAME,EN_CONF_INPUT_05_NAME_LEN);
//			}
//		}
//		break;
	case 7:
		if(systemPara.isRSTAutoSongEnable)
		{
			systemPara.isRSTAutoSongEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_7_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_RST_AUTOSONG_EN_ADDR, 0);
		}
		else
		{
			if(systemPara.sensorChosen == 2 || systemPara.sensorChosen == 3)
			{
				if(systemPara.Language == 0)
					GUI_showMessage("当前感应器模式不可选此模式！",28);
				else if(systemPara.Language == 1)
					GUI_showMessage("Feed mode does not support this feature.",40);
				break;
			}
			if(systemPara.isGivenEndNoAlarmEnable)
			{
				//Feeding failure does not enter the stop state is enabled
				if(systemPara.Language == 0)
					GUI_showMessage("送料失败不切换停止已开启",24);
				else if(systemPara.Language == 1)
					GUI_showMessage("Feeding failure doesn't switch stop is enabled",46);
				break;
			}
			systemPara.isRSTAutoSongEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_7_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_RST_AUTOSONG_EN_ADDR, 1);

			if(systemPara.Language == 0)
				GUI_showMessage("注意：开启此模式有一定的危险！",30);
			else if(systemPara.Language == 1)
				GUI_showMessage("Note:Turning this feature on is dangerous.",42);

		}
		break;
	case 8:
		if(systemPara.isCloseHCEnable)
		{
			systemPara.isCloseHCEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_8_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_CLOSEHC_EN_ADDR, 0);
		}
		else
		{
			systemPara.isCloseHCEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_8_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_CLOSEHC_EN_ADDR, 1);
		}
		break;
	case 9:
		if(systemPara.isLackMaterral_NC)
		{
			systemPara.isLackMaterral_NC = 0;
			GUI_sendWord(FUNCTION_CHOSE_9_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_LACKMATERRAL_NC_EN_ADDR, 0);
		}
		else
		{
			systemPara.isLackMaterral_NC = 1;
			GUI_sendWord(FUNCTION_CHOSE_9_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_LACKMATERRAL_NC_EN_ADDR, 1);
		}
		break;
	case 10:
		if(systemPara.isUpSensorIsRealtime)
		{
			systemPara.isUpSensorIsRealtime = 0;
			GUI_sendWord(FUNCTION_CHOSE_10_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_UPSENSORREAL_EN_ADDR, 0);
		}
		else
		{
			if(systemPara.isLowspeedSensorEnable)
			{
				if(systemPara.Language == 0)
					GUI_showMessage("传感器减速功能已开启，请关闭！",30);
				else if(systemPara.Language == 1)
					GUI_showMessage("Please turn off sensor deceleration!",36);

				break;
			}
			if(systemPara.isCylinderSensorEnable)
			{

				if(systemPara.Language == 0)
					GUI_showMessage("传感器气缸功能已开启，请关闭！",30);
				else if(systemPara.Language == 1)
					GUI_showMessage("Please turn off the sensor cylinder function!",45);
				break;
			}
			systemPara.isUpSensorIsRealtime = 1;
			GUI_sendWord(FUNCTION_CHOSE_10_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_UPSENSORREAL_EN_ADDR, 1);
		}
		break;
	case 11:
		if(systemPara.isLowspeedSensorEnable)
		{
			systemPara.isLowspeedSensorEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_11_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_LOWSPEEDSENSOR_EN_ADDR, 0);
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_01_NAME_ADDR,CONF_INPUT_01_NAME,CONF_INPUT_01_NAME_LEN);
				GUI_showTextIsolate(INPUT_02_NAME_ADDR,CONF_INPUT_02_NAME,CONF_INPUT_02_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_01_NAME_ADDR,EN_CONF_INPUT_01_NAME,EN_CONF_INPUT_01_NAME_LEN);
				GUI_showTextIsolate(INPUT_02_NAME_ADDR,EN_CONF_INPUT_02_NAME,EN_CONF_INPUT_02_NAME_LEN);
			}
		}
		else
		{
			if(systemPara.isUpSensorIsRealtime)
			{
				if(systemPara.Language == 0)
					GUI_showMessage("上光纤实时输出有料信号已开启！",30);
				else if(systemPara.Language == 1)
					GUI_showMessage("Optical fiber real-time output signal is ON!",44);
				break;
			}
        	if(systemPara.sensorChosen == 2)
        	{
				if(systemPara.Language == 0)
					GUI_showMessage("当前感应器模式不可选此模式！",28);
				else if(systemPara.Language == 1)
					GUI_showMessage("Feed mode does not support this feature.",40);
        		break;
        	}
			systemPara.isLowspeedSensorEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_11_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_LOWSPEEDSENSOR_EN_ADDR, 1);
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_01_NAME_ADDR,CONF_INPUT_001_NAME,CONF_INPUT_001_NAME_LEN);
				GUI_showTextIsolate(INPUT_02_NAME_ADDR,CONF_INPUT_002_NAME,CONF_INPUT_002_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_01_NAME_ADDR,EN_CONF_INPUT_001_NAME,EN_CONF_INPUT_001_NAME_LEN);
				GUI_showTextIsolate(INPUT_02_NAME_ADDR,EN_CONF_INPUT_002_NAME,EN_CONF_INPUT_002_NAME_LEN);
			}
		}
		break;
	case 12:
		if(systemPara.isGivenEndNoAlarmEnable)
		{
			systemPara.isGivenEndNoAlarmEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_12_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_GIVENIENDISNOALARM_EN_ADDR, 0);
		}
		else
		{
			if(systemPara.isAutoSongEnable || systemPara.isRSTAutoSongEnable)
			{

				if(systemPara.Language == 0)
					GUI_showMessage("自动送料功能已开启，请关闭！",28);
				else if(systemPara.Language == 1)
					GUI_showMessage("Please close the automatic feeding function!",44);
				break;
			}
			systemPara.isGivenEndNoAlarmEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_12_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_GIVENIENDISNOALARM_EN_ADDR, 1);
		}
		break;
	case 13:
		if(systemPara.isInEmergencyStopEnable)
		{
			systemPara.isInEmergencyStopEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_13_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_EM_STOP_EN_ADDR, 0);
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_11_NAME_ADDR,CONF_INPUT_11_NAME,CONF_INPUT_11_NAME_LEN);
			}
			else if(systemPara.Language ==  1)
			{
				GUI_showTextIsolate(INPUT_11_NAME_ADDR,EN_CONF_INPUT_11_NAME,EN_CONF_INPUT_11_NAME_LEN);
			}
		}
		else
		{
			systemPara.isInEmergencyStopEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_13_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_EM_STOP_EN_ADDR, 1);
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_11_NAME_ADDR,CONF_INPUT_011_NAME,CONF_INPUT_011_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_11_NAME_ADDR,EN_CONF_INPUT_011_NAME,EN_CONF_INPUT_011_NAME_LEN);
			}
		}
		break;
	case 14:
		if(systemPara.isUpSensorDownWichoutEnable)
		{
			systemPara.isUpSensorDownWichoutEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_14_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_UPSENSORDOWNWITHOUT_EN_ADDR, 0);
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_01_NAME_ADDR,CONF_INPUT_01_NAME,CONF_INPUT_01_NAME_LEN);
				GUI_showTextIsolate(INPUT_02_NAME_ADDR,CONF_INPUT_02_NAME,CONF_INPUT_02_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_01_NAME_ADDR,EN_CONF_INPUT_01_NAME,EN_CONF_INPUT_01_NAME_LEN);
				GUI_showTextIsolate(INPUT_02_NAME_ADDR,EN_CONF_INPUT_02_NAME,EN_CONF_INPUT_02_NAME_LEN);
			}
		}
		else
		{
			if(systemPara.sensorChosen == 1)
			{//下光纤
				if(systemPara.Language == 0)
					GUI_showMessage("当前感应器模式不可选此模式！",28);
				else if(systemPara.Language == 1)
					GUI_showMessage("Feed mode does not support this feature.",40);
				break;
			}
			if(systemPara.isLowspeedSensorEnable == 1)
			{//关闭传感器减速功能
				systemPara.isLowspeedSensorEnable = 0;
				GUI_sendWord(FUNCTION_CHOSE_11_ADDR, 0, 0);
				AT24CXX_WriteOneByte(STORAGE_LOWSPEEDSENSOR_EN_ADDR, 0);
			}

			systemPara.isUpSensorDownWichoutEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_14_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_UPSENSORDOWNWITHOUT_EN_ADDR, 1);
			if(systemPara.Language == 0)
			{
				GUI_showTextIsolate(INPUT_01_NAME_ADDR,CONF_INPUT_0001_NAME,CONF_INPUT_0001_NAME_LEN);
				GUI_showTextIsolate(INPUT_02_NAME_ADDR,CONF_INPUT_0002_NAME,CONF_INPUT_0002_NAME_LEN);
			}
			else if(systemPara.Language == 1)
			{
				GUI_showTextIsolate(INPUT_01_NAME_ADDR,EN_CONF_INPUT_0001_NAME,EN_CONF_INPUT_0001_NAME_LEN);
				GUI_showTextIsolate(INPUT_02_NAME_ADDR,EN_CONF_INPUT_0002_NAME,EN_CONF_INPUT_0002_NAME_LEN);
			}
		}
		break;
	case 15:
		if(systemPara.isInitSwOnLINEEnable)
		{
			systemPara.isInitSwOnLINEEnable = 0;
			GUI_sendWord(FUNCTION_CHOSE_15_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_INITSWONLINE_EN_ADDR, 0);
		}
		else
		{
			systemPara.isInitSwOnLINEEnable = 1;
			GUI_sendWord(FUNCTION_CHOSE_15_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_INITSWONLINE_EN_ADDR, 1);
		}
		break;
//	case 16://双感应器
//		if(systemPara.isDownShouSensorEnable)
//		{
//			systemPara.isDownShouSensorEnable = 0;
//			GUI_sendWord(FUNCTION_CHOSE_16_ADDR, 0, 0);
//			AT24CXX_WriteOneByte(STORAGE_DOWNSENSORREAL_EN_ADDR, 0);
//			if(systemPara.Language == 0)
//			{
//				GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_006_NAME,CONF_INPUT_006_NAME_LEN);
//				GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_007_NAME,CONF_INPUT_007_NAME_LEN);
//			}
//			else if(systemPara.Language == 1)
//			{
//				GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_006_NAME,EN_CONF_INPUT_006_NAME_LEN);
//				GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_007_NAME,EN_CONF_INPUT_007_NAME_LEN);
//			}
//		}
//		else
//		{
//			if(!systemPara.isDownShouEnable)
//			{
//				if(systemPara.Language == 0)
//					GUI_showMessage("下收料功能未启用！",26);
//				else if(systemPara.Language == 1)
//					GUI_showMessage("Lower film feature not enabled.", 31);
//				break;
//			}
//			systemPara.isDownShouSensorEnable = 1;
//			GUI_sendWord(FUNCTION_CHOSE_16_ADDR, 0, 1);
//			AT24CXX_WriteOneByte(STORAGE_DOWNSENSORREAL_EN_ADDR, 1);
//			if(systemPara.Language == 0)
//			{
//				GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_06_NAME,CONF_INPUT_06_NAME_LEN);
//				GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_07_NAME,CONF_INPUT_07_NAME_LEN);
//			}
//			else if(systemPara.Language == 1)
//			{
//				GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_06_NAME,EN_CONF_INPUT_06_NAME_LEN);
//				GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_07_NAME,EN_CONF_INPUT_07_NAME_LEN);
//			}
//		}
//		break;
//	case 17:
//		if(systemPara.isShouStopSensorEnable)
//		{
//			systemPara.isShouStopSensorEnable = 0;
//			GUI_sendWord(FUNCTION_CHOSE_17_ADDR, 0, 0);
//			AT24CXX_WriteOneByte(STORAGE_SHOUSTEEPSENSORREAL_EN_ADDR, 0);
////			if(systemPara.Language == 0)
////			{
////				GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_006_NAME,CONF_INPUT_006_NAME_LEN);
////				GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_007_NAME,CONF_INPUT_007_NAME_LEN);
////			}
////			else if(systemPara.Language == 1)
////			{
////				GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_006_NAME,EN_CONF_INPUT_006_NAME_LEN);
////				GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_007_NAME,EN_CONF_INPUT_007_NAME_LEN);
////			}
//		}
//		else
//		{
//			if(!systemPara.isDownShouEnable)
//			{
//				if(systemPara.Language == 0)
//					GUI_showMessage("下收料功能未启用！",26);
//				else if(systemPara.Language == 1)
//					GUI_showMessage("Lower film feature not enabled.", 31);
//				break;
//			}
//			systemPara.isDownShouSensorEnable = 1;
//			GUI_sendWord(FUNCTION_CHOSE_17_ADDR, 0, 1);
//			AT24CXX_WriteOneByte(STORAGE_SHOUSTEEPSENSORREAL_EN_ADDR, 1);
////			if(systemPara.Language == 0)
////			{
////				GUI_showTextIsolate(INPUT_06_NAME_ADDR,CONF_INPUT_06_NAME,CONF_INPUT_06_NAME_LEN);
////				GUI_showTextIsolate(INPUT_07_NAME_ADDR,CONF_INPUT_07_NAME,CONF_INPUT_07_NAME_LEN);
////			}
////			else if(systemPara.Language == 1)
////			{
////				GUI_showTextIsolate(INPUT_06_NAME_ADDR,EN_CONF_INPUT_06_NAME,EN_CONF_INPUT_06_NAME_LEN);
////				GUI_showTextIsolate(INPUT_07_NAME_ADDR,EN_CONF_INPUT_07_NAME,EN_CONF_INPUT_07_NAME_LEN);
////			}
//		}
//		break;
#endif
	case 20:
		if(systemPara.MotorGivenDir)
		{
			systemPara.MotorGivenDir = 0;
			GUI_sendWord(MOTOR1DIR_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_GIVEN_DIR_ADDR, 0);
		}
		else
		{
			systemPara.MotorGivenDir = 1;
			GUI_sendWord(MOTOR1DIR_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_GIVEN_DIR_ADDR, 1);
		}
		break;
	case 21:
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
	case 22:
		if(systemPara.MotorBoDir)
		{
			systemPara.MotorBoDir = 0;
			GUI_sendWord(MOTOR3DIR_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_BO_DIR_ADDR, 0);
		}
		else
		{
			systemPara.MotorBoDir = 1;
			GUI_sendWord(MOTOR3DIR_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_BO_DIR_ADDR, 1);
		}
		break;
	case 23:
		if(systemPara.MotorShouDir)
		{
			systemPara.MotorShouDir = 0;
			GUI_sendWord(MOTOR4DIR_ADDR, 0, 0);
			AT24CXX_WriteOneByte(STORAGE_SHOU_DIR_ADDR, 0);
		}
		else
		{
			systemPara.MotorShouDir = 1;
			GUI_sendWord(MOTOR4DIR_ADDR, 0, 1);
			AT24CXX_WriteOneByte(STORAGE_SHOU_DIR_ADDR, 1);
		}
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
//            AT24CXX_WriteOneByte(STORAGE_OFFSETSELECT_ADDR,1);
            break;

        case 2:
            GUI_sendWord(SINGAL_SELECT_1_ADDR,0,0);
            GUI_sendWord(SINGAL_SELECT_2_ADDR,0,1);

            MOTOR_offsetOrigin = 2;
//            AT24CXX_WriteOneByte(STORAGE_OFFSETSELECT_ADDR,2);
            break;
    }

    ADDR_6000_L[40] = 0;
}




