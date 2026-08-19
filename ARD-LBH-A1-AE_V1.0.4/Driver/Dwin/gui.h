/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-02     ylj       the first version
 */
#ifndef DRIVERS_DWIN_GUI_H_
#define DRIVERS_DWIN_GUI_H_

#include "stm32f1xx_hal.h"
#include "sysConfig/sysconfig.h"

void GUI_reset(void);
void GUI_init(void);
void GUI_switchPage(uint16_t pageNo);
void GUI_showText(uint16_t addr,char *data,uint8_t len);
void GUI_showMessage(char * message,uint8_t len);
void GUI_iconShow(uint16_t addr,uint8_t no);
void GUI_sendWord(uint16_t addr,uint8_t high,uint8_t low);
void GUI_initDisplay(void);
void GUI_switchModeDisp(uint8_t mode);
//void GUI_send(char *data,uint8_t length);
void GUI_switchPageIsolate(uint16_t pageNo);
void GUI_showTextIsolate(uint16_t addr,char *data,uint8_t len);
void GUI_sendWordIsolate(uint16_t addr,uint8_t high,uint8_t low);
void GUI_runStatusDisp(uint8_t isRun);
void GUI_sensorSelectDisp(uint8_t downSensor);
void GUI_givenModeSelectDisp(uint8_t no);
void GUI_mainMessageDisp(char * message,uint8_t len);
void GUI_mainMessageDispIsolate(char * message,uint8_t len);
void GUI_setOutputSignalColorDisp(uint8_t outputNo,uint8_t color);

void GUI_task(void * pv);
void GUI_sendTask(void *pv);
void GUI_send(char *data,uint8_t length);

void GUI_handler(void);

#define LANGUAGE_DISP_ADDR          0x7200      ///<language
#define USERNAME_DISP_ADDR          0x7500      ///<
#define PASSWORD_DISP_ADDR          0X7505      ///<
#define LOGGINSTATE_DISP_ADDR       0x7610      ///<
#define INFO_DISP_ADDR              0x7508      ///<
#define NPASS_DISP_ADDR             0x7520      ///<
#define NCPASS_DISP_ADDR            0x7523      ///<

#define SIGNAL_NAME_1_ADDR          0x7615      
#define SIGNAL_NAME_2_ADDR          0x761d
#define SIGNAL_NAME_3_ADDR          0x7625
#define SIGNAL_NAME_4_ADDR          0x762d
#define SIGNAL_NAME_5_ADDR          0x7635
#define SIGNAL_NAME_6_ADDR          0x763d
#define SIGNAL_NAME_7_ADDR          0x7645
#define SIGNAL_NAME_8_ADDR          0x764d

#define SINGAL_SELECT_1_ADDR        0x766c
#define SINGAL_SELECT_2_ADDR        0x766d
#define SINGAL_SENSOR_DOWN_ADDR     0x7699
#define SINGAL_SENSOR_UP_ADDR       0x769A
#define SINGAL_SENSOR_NO_ADDR       0x769D
#define SINGAL_GIVENMODE_N          0x769B
#define SINGAL_GIVENMODE_Y          0x769C

#define CTNSL_FLAG_ADDR    			0x7910		///< 连续送料
#define CTNFL_FLAG_ADDR    			0x7911		///< 连续放料
#define CTNUPSL_FLAG_ADDR    		0x7912		///< 连续上收料
#define CTNDOWNS_FLAGL_ADDR    		0x7913		///< 连续下收料
#define CTNTL_FLAG_ADDR    			0x7914		///< 连续退料

#define MANUALPRODUCT_FALG_ADDR     0x7695
#define AUTOPRODUCT_FLAG_ADDR       0x7694
#define FREERUN_FLAG_ADDR           0x7696
#define INIT_FLAG_ADDR              0x7697
#define STOP_FLAG_ADDR              0x7698

#define INFOR_DEVICE_NAME_ADDR      0X7700
#define INFOR_DEVICE_NO_ADDR        0X7710
#define INFOR_HARDWARE_VER_ADDR     0X7720
#define INFOR_SOFTWARE_VER_ADDR     0X7730

#define INPUT_00_NAME_ADDR          0X7740
#define INPUT_01_NAME_ADDR          0X7750
#define INPUT_02_NAME_ADDR          0X7760
#define INPUT_03_NAME_ADDR          0X7770
#define INPUT_04_NAME_ADDR          0X7780
#define INPUT_05_NAME_ADDR          0X7790
#define INPUT_06_NAME_ADDR          0X77A0
#define INPUT_07_NAME_ADDR          0X77B0
#define INPUT_08_NAME_ADDR          0X77C0
#define INPUT_09_NAME_ADDR          0X77D0
#define INPUT_10_NAME_ADDR          0X77E0
#define INPUT_11_NAME_ADDR          0X77F0

#define EXINPUT_00_NAME_ADDR		0X7830
#define EXINPUT_01_NAME_ADDR		0X7840
#define EXINPUT_02_NAME_ADDR		0X7850
#define EXINPUT_03_NAME_ADDR		0X7860
#define EXINPUT_04_NAME_ADDR		0X7870
#define EXINPUT_05_NAME_ADDR		0X7880
#define EXINPUT_06_NAME_ADDR		0X7890
#define EXINPUT_07_NAME_ADDR		0X78A0

#define OUTPUT_1_NAME_ADDR          0x766e
#define OUTPUT_2_NAME_ADDR          0x7676
#define OUTPUT_3_NAME_ADDR          0X767E
#define OUTPUT_4_NAME_ADDR          0X7686
#define OUTPUT_5_NAME_ADDR			0x78b0
#define OUTPUT_6_NAME_ADDR			0X78B8
#define OUTPUT_7_NAME_ADDR			0X78C0
#define OUTPUT_8_NAME_ADDR			0X78C8

#define MAINPAGE_MESSAGE_ADDR       0x7800

#define	FUNCTION_CHOSE_1_ADDR		0x7920		///< 启用放料功能
#define	FUNCTION_CHOSE_2_ADDR		0x7921		///< 启用上收料功能
#define	FUNCTION_CHOSE_3_ADDR		0x7922		///< 启用下收料功能
#define	FUNCTION_CHOSE_4_ADDR		0x7923		///< 启用自动送料功能
#define	FUNCTION_CHOSE_5_ADDR		0x7924		///< 启用传感器气缸功能
#define	FUNCTION_CHOSE_6_ADDR		0x7925		///< 启用上收料传感器
#define	FUNCTION_CHOSE_7_ADDR		0x7926		///< 启用自动复位同步送料
#define	FUNCTION_CHOSE_8_ADDR		0x7927		///< 屏蔽后撤功能
#define	FUNCTION_CHOSE_9_ADDR		0x7928		///< 缺料常开
#define	FUNCTION_CHOSE_10_ADDR		0x7929		///< 上光纤实时有料信号
#define	FUNCTION_CHOSE_11_ADDR		0x792A		///< 启用传感器减速功能
#define	FUNCTION_CHOSE_12_ADDR		0x792B		///< 启用送料限位走完不停止功能
#define	FUNCTION_CHOSE_13_ADDR		0x792C		///< 启用内部急停
#define	FUNCTION_CHOSE_14_ADDR		0x792D		///< 启用上光纤定位，下光纤检测有无
#define	FUNCTION_CHOSE_15_ADDR		0x792E		///< 启用外部初始化切换联机模式
#define	FUNCTION_CHOSE_16_ADDR		0x792F		///< 启用下收料传感器

#define	FUNCTION_CHOSE_17_ADDR		0x7930		///< 启用下收料传感器

#if		CONF_LET_VER == 0
	#define	MOTOR1DIR_ADDR				0x7940		///< 电机1方向
	#define	MOTOR2DIR_ADDR				0x7941		///< 电机2方向
	#define	MOTOR3DIR_ADDR				0x7942		///< 电机3方向
	#define	MOTOR4DIR_ADDR				0x7943		///< 电机4方向
#else
	#define	MOTOR1DIR_ADDR				0x7930		///< 电机1方向
	#define	MOTOR2DIR_ADDR				0x7931		///< 电机2方向
	#define	MOTOR3DIR_ADDR				0x7932		///< 电机3方向
	#define	MOTOR4DIR_ADDR				0x7933		///< 电机4方向
#endif

#define SINGAL_DUAL_SENSOR_DOWN_ADDR     	0x7950			///< 上收料双感应器
#define SINGAL_SINGLE_SENSOR_DOWN_ADDR     	0x7951			///< 上收料单感应器
#define SINGAL_NOT_SENSOR_DOWN_ADDR     	0x7952			///< 上收料无感应器

#define SINGAL_DUAL_SENSOR_UP_ADDR       	0x7958			///< 下收料双感应器
#define SINGAL_SINGLE_SENSOR_UP_ADDR       	0x7959			///< 下收料单感应器
#define SINGAL_NOT_SENSOR_UP_ADDR       	0x795A			///< 下收料无感应器


#define PAGE_MAIN_NO                0           ///< ��ҳ����        

#define MAINPAGE_MESAGE_CLEAR       "                                                  ",64

extern uint16_t pageNoBeforMessage;
extern uint32_t logout_count1;	//	自动登出
extern uint8_t resetFeederflag;	//	恢复出厂设置计时标志位
extern uint64_t resetFeederTime;	//	恢复出厂设置计时
extern uint8_t changeoverflag; // 换料计时标志
extern uint64_t ChangeoverTime;// 换料计时

#endif /* DRIVERS_DWIN_GUI_H_ */
