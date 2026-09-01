/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-07     ylj       the first version
 */
#ifndef APPLICATIONS_STORAGE_STORAGE_H_
#define APPLICATIONS_STORAGE_STORAGE_H_

#include "stm32f1xx_hal.h"
#include "sysconfig/sysconfig.h"

#define     STORAGE_CHECK_ADDR              0x00    ///< �洢�����

#define     STORAGE_INITED_ADDR             0x01    ///< ��ʼ����־λ
#define     STORAGE_USERPS_ADDR             0x02    ///< �û�����
#define     STORAGE_VENDORPS_ADDR           0x08    ///< ��Ӧ������

#define     STORAGE_GIVENSPEED_ADDR         0x10    ///< �����ٶ�
#define     STORAGE_GIVENOFFSET_ADDR        0x12    ///< ����ƫ��
#define     STORAGE_LETSPEED_ADDR           0x14    ///< �����ٶ�
#define     STORAGE_LETOFFSET_ADDR          0x16    ///< ����ƫ��
#define     STORAGE_BOSPEED_ADDR            0x18    ///< �����ٶ�
#define     STORAGE_BODISTANCE_ADDR         0x1A    ///< ���Ϻ��г�
#define     STORAGE_SHOUSPEED_ADDR          0x1C    ///< �����ٶ�
#define     STORAGE_SHOUDISTANCE_ADDR       0x1E    ///< �����г�
#define     STORAGE_ORISPEED_ADDR           0x20    ///< ��ԭ���ٶ�
#define     STORAGE_ORIOFFSET_ADDR          0x22    ///< ԭ��ƫ����
#define     STORAGE_GIVENCUR_ADDR           0x24    ///< ���ϵ������
#define     STORAGE_GIVENMS_ADDR            0x26    ///< ���ϵ��ϸ��
#define     STORAGE_LETCUR_ADDR             0x28    ///< ���ϵ������
#define     STORAGE_LETMS_ADDR              0x2A    ///< ���ϵ��ϸ��
#define     STORAGE_BOCUR_ADDR              0x2C    ///< ���ϵ������
#define     STORAGE_BOMS_ADDR               0x2E    ///< ���ϵ��ϸ��
#define     STORAGE_SHOUCUR_ADDR            0x30    ///< ���ϵ������
#define     STORAGE_SHOUMS_ADDR             0x32    ///< ���ϵ��ϸ��

#define     STORAGE_HCD2_ADDR             	0x34    ///< ���ϵ��ϸ��
#define     STORAGE_HCD2SPEED_ADDR          0x36    ///< ���ϵ��ϸ��
#define     STORAGE_CHANGEOVER_ADDR         0x38    ///< ���ϵ��ϸ��

//#define 	STORAGE_OFFSETSELECT_ADDR		0x43	///< 原点偏移方向
//#define 	STORAGE_SENSORSELECT_ADDR		0x44	///< 选择传感器模式
#define 	STORAGE_LABLEPOSITION_ADDR		0x44	// 贴标位置选择
#define 	STORAGE_GIVENMODESELECT_ADDR	0x45	///< 选择有料是否可送模式
#define 	STORAGE_WORKMODE_ADDR			0x46
#define 	STORAGE_LANGUAGE_ADDR			0x48	//language 47

#define 	STORAGE_LET_EN_ADDR				0X70
#define 	STORAGE_UP_EN_ADDR				0X71
#define 	STORAGE_DOWN_EN_ADDR			0x72
#define 	STORAGE_FEEDINPLASE_ADDR		0x73
#define		STORAGE_LACKMATERRAL_NC_EN_ADDR 0X74
#define		STORAGE_EM_STOP_EN_ADDR			0X75 //isInEmergencyStopEnable
#define		STORAGE_INITSWONLINE_EN_ADDR	0X76

#define 	STORAGE_GIVEN_DIR_ADDR			0x80
#define 	STORAGE_LET_DIR_ADDR			0x81
#define 	STORAGE_UpShou_DIR_ADDR			0x82
#define 	STORAGE_DownShow_DIR_ADDR		0x83

uint8_t STORAGE_checkStorage(void);
uint8_t STORAGE_checkInited(void);
void STORAGE_setAllParameter(void);
void STORAGE_saveAll(void);
void STORAGE_init(void);
void STORAGE_getPassword(uint8_t which,char *password);
void STORAGE_setDefaultParameter(void);
void STORAGE_saveSingalParameter(uint8_t addr, uint16_t para);
void STORAGE_resetFeederParam(void);
void STORAGE_updataPassword(uint8_t which);

// ���ٶ� ��λ: Hz/(10ms)
#define     DEFAULT_GM_ACC          CONF_GM_ACC
#define     DEFAULT_GM_DEC          CONF_GM_DEC

#define     DEFAULT_LM_ACC          CONF_LM_ACC
#define     DEFAULT_LM_DEC          CONF_LM_DEC

#define     DEFAULT_BM_ACC          CONF_BM_ACC   
#define     DEFAULT_BM_DEC          CONF_BM_DEC

#define     DEFAULT_SM_ACC          CONF_SM_ACC
#define     DEFAULT_SM_DEC          CONF_SM_DEC

#endif /* APPLICATIONS_STORAGE_STORAGE_H_ */
