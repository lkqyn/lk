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
#define USERNAME_DISP_ADDR          0x7500      ///< �û�����ʾ
#define PASSWORD_DISP_ADDR          0X7505      ///< ������ʾ��*��
#define LOGGINSTATE_DISP_ADDR       0x7610      ///< ��ʾ��¼״̬
#define INFO_DISP_ADDR              0x7508      ///< ��Ϣ��ַ
#define NPASS_DISP_ADDR             0x7520      ///< ��������ʾ��*����ַ
#define NCPASS_DISP_ADDR            0x7523      ///< ȷ����������ʾ��*����ַ

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

#define LIANXUSONGLIAO_FLAG_ADDR    0x7690
#define LIANXUSHOULIAO_FLAG_ADDR    0x7693

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
#define	FUNCTION_CHOSE_2_ADDR		0x7921		///< 启用收料功能
#define	FUNCTION_CHOSE_3_ADDR		0x7922		///< 启用收料传感器
#define	FUNCTION_CHOSE_4_ADDR		0x7923		///< 启用送料到位检测，不选为限位传感
#define	FUNCTION_CHOSE_5_ADDR		0x7924		///< 缺料常开
#define	FUNCTION_CHOSE_6_ADDR		0x7925		///< 启用内部急停
#define	FUNCTION_CHOSE_7_ADDR		0x7926		///< 启用外部初始化切换联机模式


#define PAGE_MAIN_NO                0           ///< ��ҳ����        

#define MAINPAGE_MESAGE_CLEAR       "                                        ",64

extern uint16_t pageNoBeforMessage;
extern uint32_t logout_count1;	//	自动登出

#endif /* DRIVERS_DWIN_GUI_H_ */
