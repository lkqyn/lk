/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-07     ylj       the first version
 */

#include "storage/storage.h"
#include "at24cxx/at24cxx.h"
#include "paraManager/paramanager.h"
#include "motorctrl/motorctrl.h"
#include "sysconfig/sysconfig.h"

static void STORAGE_savePassword(uint8_t which,char * password);
static void defualt_acc(void);

static const uint16_t defaultPara[] = {CONF_GM_SPEED,CONF_GM_OFFSET,  
                                       CONF_LM_SPEED,CONF_LM_OFFSET,  
                                       CONF_BM_SPEED,CONF_BM_OFFSET, 
                                       CONF_SM_SPEED,CONF_SM_DISTANCE,  
                                       CONF_GOZERO_SPEED,CONF_ZERO_OFFSET,      
                                       CONF_GM_AMP,CONF_GM_MS,     
                                       CONF_LM_AMP,CONF_LM_MS,        
                                       CONF_BM_AMP,CONF_BM_MS,        
                                       CONF_SM_AMP,CONF_SM_MS,        
									   CONF_HC_D2,CONF_BS_POINT,		// 第二次后撤距离, 变速点
									   CONF_HL_DISTANCE,
};                                    
                                    

/**
 * @brief 检查存储器是否正常
 * @return
 */
uint8_t STORAGE_checkStorage(void)
{
    uint8_t rtl;

    AT24CXX_WriteOneByte(STORAGE_CHECK_ADDR,0x5a);
    if(0x5a == AT24CXX_ReadOneByte(STORAGE_CHECK_ADDR))
    {
        rtl = 1;
    }
    else
    {
        rtl = 0;
    }

    return rtl;
}

/**
 * @brief 
 * @return
 */
uint8_t STORAGE_checkInited(void)
{
    uint8_t rtl;

    if(0x37 == AT24CXX_ReadOneByte(STORAGE_INITED_ADDR))
    {
        rtl = 1;
    }
    else
    {
        rtl = 0;
    }

    return rtl;
}

/**
 * @brief 读取所以参数
 */
void STORAGE_setAllParameter(void)
{
    uint16_t paraIndex = 0x5000;
    uint8_t storageIndex = 0x10;
    uint8_t i;

    STORAGE_getPassword(0,systemPara.userPassword);
    STORAGE_getPassword(1,systemPara.vendorPassword);

    for(;storageIndex<=0x38;storageIndex+=2)
    {
        PARA_writeParameter8bit(paraIndex, AT24CXX_ReadOneByte(storageIndex+1), AT24CXX_ReadOneByte(storageIndex));
        paraIndex ++;
    }
    
    for(i=0;i<4;i++)
    {
        motor_ch[i].acc.inc = (AT24CXX_ReadOneByte(0x51+(i<<2))<<8) | AT24CXX_ReadOneByte(0x50+(i<<2));
        motor_ch[i].acc.dec = (AT24CXX_ReadOneByte(0x53+(i<<2))<<8) | AT24CXX_ReadOneByte(0x52+(i<<2));        
    }
    
//    MOTOR_offsetOrigin = AT24CXX_ReadOneByte(STORAGE_OFFSETSELECT_ADDR); //��ȡƫ�Ʒ���
    
    //systemPara.sensorChosen = AT24CXX_ReadOneByte(STORAGE_SENSORSELECT_ADDR); // ��ȡ��λ��Ӧ��ѡ��
    systemPara.Lableposition = AT24CXX_ReadOneByte(STORAGE_LABLEPOSITION_ADDR);
    systemPara.givenMode = AT24CXX_ReadOneByte(STORAGE_GIVENMODESELECT_ADDR); // ��ȡ����ģʽѡ��
    
    systemPara.status = AT24CXX_ReadOneByte(STORAGE_WORKMODE_ADDR);

    systemPara.Language = AT24CXX_ReadOneByte(STORAGE_LANGUAGE_ADDR);

   if(systemPara.Language != 0 || systemPara.Language != 1)
   {
	   systemPara.Language = 0;
	   AT24CXX_WriteOneByte(STORAGE_LANGUAGE_ADDR,systemPara.Language);
   }

    systemPara.isAutoLetMetalEnable = AT24CXX_ReadOneByte(STORAGE_LET_EN_ADDR);

    systemPara.isUpShouEnable = AT24CXX_ReadOneByte(STORAGE_UP_EN_ADDR);
    systemPara.isDownShouEnable = AT24CXX_ReadOneByte(STORAGE_DOWN_EN_ADDR);


    systemPara.isFeedInPlaceEnable = AT24CXX_ReadOneByte(STORAGE_FEEDINPLASE_ADDR);

    systemPara.isLackMaterral_NC = AT24CXX_ReadOneByte(STORAGE_LACKMATERRAL_NC_EN_ADDR);
//    systemPara.isDownShouEnable = AT24CXX_ReadOneByte(STORAGE_DOWN_EN_ADDR);

    systemPara.isInEmergencyStopEnable = AT24CXX_ReadOneByte(STORAGE_EM_STOP_EN_ADDR);
    systemPara.isInitSwOnLINEEnable = AT24CXX_ReadOneByte(STORAGE_INITSWONLINE_EN_ADDR);

    systemPara.MotorGivenDir = AT24CXX_ReadOneByte(STORAGE_GIVEN_DIR_ADDR);
	systemPara.MotorLetDir = AT24CXX_ReadOneByte(STORAGE_LET_DIR_ADDR);
	systemPara.MotorUpShouDir = AT24CXX_ReadOneByte(STORAGE_UpShou_DIR_ADDR);
	systemPara.MotorDownShouDir = AT24CXX_ReadOneByte(STORAGE_DownShow_DIR_ADDR);

    PARA_writeParameter8bit(0x5012, AT24CXX_ReadOneByte(0x35), AT24CXX_ReadOneByte(0x34));
}

/**
 * @brief 保存所有参数
 * @note
 */
void STORAGE_saveAll(void)
{
    uint8_t storageAddr = 0x10;
    uint8_t i;

    for(i=0;i<ADDR5000_NUM;i++)
    {
        AT24CXX_WriteOneByte(storageAddr, ADDR_5000_L[i]);
        storageAddr++;
        AT24CXX_WriteOneByte(storageAddr, ADDR_5000_H[i]);
        storageAddr++;
    }
    
    for(i=0;i<4;i++)
    {
        AT24CXX_WriteOneByte(0x50+(i<<2),motor_ch[i].acc.inc & 0xff);
        AT24CXX_WriteOneByte(0x51+(i<<2),((motor_ch[i].acc.inc >> 8) & 0xff));
        AT24CXX_WriteOneByte(0x52+(i<<2),motor_ch[i].acc.dec & 0xff);
        AT24CXX_WriteOneByte(0x53+(i<<2),((motor_ch[i].acc.dec >> 8) & 0xff));
    }
    
    AT24CXX_WriteOneByte(0x43,ADDR_5000_L[12]);
    
    AT24CXX_WriteOneByte(STORAGE_LANGUAGE_ADDR,systemPara.Language);

//    AT24CXX_WriteOneByte(STORAGE_SENSORSELECT_ADDR,systemPara.sensorChosen);
    AT24CXX_WriteOneByte(STORAGE_LABLEPOSITION_ADDR,systemPara.Lableposition);
    AT24CXX_WriteOneByte(STORAGE_GIVENMODESELECT_ADDR,systemPara.givenMode);
    
    AT24CXX_WriteOneByte(STORAGE_LET_EN_ADDR, systemPara.isAutoLetMetalEnable);
    AT24CXX_WriteOneByte(STORAGE_UP_EN_ADDR, systemPara.isUpShouEnable);
//    AT24CXX_WriteOneByte(STORAGE_DOWN_EN_ADDR, systemPara.isUpShouSensorEnable);
    AT24CXX_WriteOneByte(STORAGE_DOWN_EN_ADDR, systemPara.isDownShouEnable);

    AT24CXX_WriteOneByte(STORAGE_FEEDINPLASE_ADDR, systemPara.isFeedInPlaceEnable);
    AT24CXX_WriteOneByte(STORAGE_LACKMATERRAL_NC_EN_ADDR, systemPara.isInitSwOnLINEEnable);

    AT24CXX_WriteOneByte(STORAGE_EM_STOP_EN_ADDR, systemPara.isInEmergencyStopEnable);
    AT24CXX_WriteOneByte(STORAGE_INITSWONLINE_EN_ADDR, systemPara.isInitSwOnLINEEnable);


    AT24CXX_WriteOneByte(STORAGE_GIVEN_DIR_ADDR, systemPara.MotorGivenDir);
    AT24CXX_WriteOneByte(STORAGE_LET_DIR_ADDR, systemPara.MotorLetDir);
    AT24CXX_WriteOneByte(STORAGE_UpShou_DIR_ADDR, systemPara.MotorUpShouDir);
    AT24CXX_WriteOneByte(STORAGE_DownShow_DIR_ADDR, systemPara.MotorDownShouDir);

    AT24CXX_WriteOneByte(STORAGE_INITED_ADDR, 0x37);
}

/**
 * @brief 写入默认参数
 * @note
 */
void STORAGE_setDefaultParameter(void)
{
    uint8_t i;
    
    for(i=0;i<ADDR5000_NUM;i++)
    {
        PARA_writeParameter(0x5000+i,defaultPara[i]);
    }
    
    MOTOR_offsetOrigin = 1; // Ĭ��������ƫ��
    systemPara.status = 2;
    systemPara.givenMode = 1; // Ĭ��ƽ̨���ϲ�����
    systemPara.sensorChosen = 1; // Ĭ���¹��˸�Ӧ��
    systemPara.Lableposition = 1;
    systemPara.Language = 0;

    systemPara.isAutoLetMetalEnable = 0;
    systemPara.isUpShouEnable = 0;
    systemPara.isUpShouSensorEnable = 0;
    systemPara.isDownShouEnable = 1;
    systemPara.isFeedInPlaceEnable = 1;
    systemPara.isLackMaterral_NC = 0;
    systemPara.isInEmergencyStopEnable = 0;
    systemPara.isInitSwOnLINEEnable = 0;
    systemPara.MotorGivenDir = 0;			///< 送料电机方向设置
	systemPara.MotorLetDir = 0;				///< 放料电机方向设置
	systemPara.MotorUpShouDir = 0;			///< 上收料电机方向设置
	systemPara.MotorDownShouDir = 0;		///< 下收料电机方向设置
    
//    if(AT24CXX_ReadOneByte(0x02) > '9')
    {
        PARA_setPassword(0,CONF_USER_PASSWORD);
        PARA_setPassword(1,CONF_VENDOR_PASSWORD);
        STORAGE_savePassword(0,CONF_USER_PASSWORD);
        STORAGE_savePassword(1,CONF_VENDOR_PASSWORD);

//        AT24CXX_WriteOneByte(STORAGE_LET_EN_ADDR, 1);
//        AT24CXX_WriteOneByte(STORAGE_UP_EN_ADDR, 1);
//        AT24CXX_WriteOneByte(STORAGE_DOWN_EN_ADDR, 1);
    }
    
    defualt_acc();
    
    STORAGE_saveAll();
}

/**
 * @brief 存储器初始化
 *
 */
void STORAGE_init(void)
{
    PARA_setPassword(2,CONF_SUPER_PASSWORD);
    
    AT24CXX_Init();
//    AT24CXX_WriteOneByte(STORAGE_INITED_ADDR,0xff);
//    STORAGE_savePassword(0,"000000");
//    STORAGE_savePassword(1,"654321");
//    while(1);
    
    if(0 == STORAGE_checkStorage())
    {//存储器故障或无存储器
        STORAGE_setDefaultParameter();
    }
    else
    {
        if(0 == STORAGE_checkInited())
        {//新存储器
            STORAGE_setDefaultParameter();
        }
        else
        {//已初始化存储器
            STORAGE_setAllParameter();
        }
    }

    defualt_acc();
}

/**
 * @brief ��ȡ����
 * @param which ��ȡ�ĸ�����
 *          @arg 0 user
 *          @arg 1 vendor
 * @param password
 */
void STORAGE_getPassword(uint8_t which,char *password)
{
    uint8_t i;
    uint8_t addr;

    switch(which)
    {
    case 0://user password
        addr = STORAGE_USERPS_ADDR;
        break;
    case 1://vendor password
        addr = STORAGE_VENDORPS_ADDR;
        break;
    }

    for(i=0;i<6;i++)
    {
        password[i] = AT24CXX_ReadOneByte(addr+i);
    }
}

static void STORAGE_savePassword(uint8_t which,char * password)
{
    uint8_t i;
    uint8_t addr;

    switch(which)
    {
    case 0://user password
        addr = STORAGE_USERPS_ADDR;
        break;
    case 1://vendor password
        addr = STORAGE_VENDORPS_ADDR;
        break;
    }

    for(i=0;i<6;i++)
    {
        AT24CXX_WriteOneByte(addr+i, password[i]);
    }

}

/**
 * 使用systemPara参数更新密码保存
 * @param which -0- user
 * 				-1- vendor
 */
void STORAGE_updataPassword(uint8_t which)
{
	uint8_t i;

	switch(which)
	{
	case 0://user password
		for(i=0;i<6;i++)
		{
			AT24CXX_WriteOneByte(STORAGE_USERPS_ADDR+i, systemPara.userPassword[i]);
		}
		break;
	case 1://vendor password
		for(i=0;i<6;i++)
		{
			AT24CXX_WriteOneByte(STORAGE_VENDORPS_ADDR+i, systemPara.vendorPassword[i]);
		}
		break;
	}
}

void STORAGE_saveSingalParameter(uint8_t addr, uint16_t para)
{
    uint8_t temp;
    
    temp = para & 0xff;
    AT24CXX_WriteOneByte(addr,temp);
    
    temp = (para >> 8) & 0xff;
    AT24CXX_WriteOneByte(addr+1,temp);
}

static void defualt_acc(void)
{
    motor_ch[GIVEN_MOTOR].acc.inc = DEFAULT_GM_ACC;
    motor_ch[GIVEN_MOTOR].acc.dec = DEFAULT_GM_DEC;
    
    motor_ch[LET_MOTOR].acc.inc = DEFAULT_LM_ACC;
    motor_ch[LET_MOTOR].acc.dec = DEFAULT_LM_DEC;
    
    motor_ch[UP_MOTOR].acc.inc = DEFAULT_BM_ACC;
    motor_ch[UP_MOTOR].acc.dec = DEFAULT_BM_DEC;
    
    motor_ch[DOWN_MOTOR].acc.inc = DEFAULT_SM_ACC;
    motor_ch[DOWN_MOTOR].acc.dec = DEFAULT_SM_DEC;
}

/**
 �ָ�����������
*/
void STORAGE_resetFeederParam(void)
{
    uint8_t i;
    uint8_t addr = 0x10;
    
    for(i=0;i<8;i++)
    {
        STORAGE_saveSingalParameter(addr,defaultPara[i]);
        addr += 2;
    }
    
    addr = 0x34;//把地址设置为送料补偿地址

    for(i=0;i<ADDR5000_NUM;i++)//不恢复电机控制参数
    {
        STORAGE_saveSingalParameter(addr,defaultPara[i + 18]);
        addr += 2;
    }

    PARA_setPassword(0,CONF_USER_PASSWORD);
	STORAGE_savePassword(0,CONF_USER_PASSWORD);

	//AT24CXX_WriteOneByte(STORAGE_SENSORSELECT_ADDR,1); //
	AT24CXX_WriteOneByte(STORAGE_LABLEPOSITION_ADDR,1);
	AT24CXX_WriteOneByte(STORAGE_GIVENMODESELECT_ADDR,1); //
    
    systemPara.status = STATUS_AUTO;
    systemPara.givenMode = 1;
    AT24CXX_WriteOneByte(STORAGE_WORKMODE_ADDR,systemPara.status);
    AT24CXX_WriteOneByte(STORAGE_GIVENMODESELECT_ADDR,systemPara.givenMode);
}
