/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-04     ylj       the first version
 */
#ifndef APPLICATIONS_PARAMANAGER_PARAMANAGER_H_
#define APPLICATIONS_PARAMANAGER_PARAMANAGER_H_

#include "stm32f1xx_hal.h"

uint16_t PARA_readParameter(uint16_t addr);
void PARA_writeParameter(uint16_t addr,uint16_t data);
float PARA_read(uint8_t addr);
void PARA_writeParameter8bit(uint16_t addr,uint8_t high,uint8_t low);
void PARA_setPassword(uint8_t which,char *pw);
void PARA_setPasswordInt(uint8_t which,uint32_t pw);

#define ADDR5000_NUM        25
#define ADDR6000_NUM        54
#define ADDR7000_NUM        6

typedef struct _systemPara
{
    uint8_t logginStatus;   ///< 0-未登录; 1-尝试User登录; 2-尝试Vendor登录; 11-User登录成功; 12-Vendor登录成功; 22-Super登录
    uint8_t passwordStatus; ///< 输入密码状态0未输入
    char userPassword[6];
    char vendorPassword[6];
    char superPassword[6];  ///< 超级密码
    uint8_t status;         ///< 运行状态 0-; 1-生产模式; 2-调试模式; 3- 空跑模式
    uint8_t Language;		///	< 语言：0 中文  1 英文
    uint8_t Initstatus;         ///< 运行状态 0-未初始化; 1-初始化完成;
    uint8_t RunStatus;		///< 0空闲; 1初始化;2送料;3后撤4剥刀复位
    uint8_t isWorkTaskRun;  ///< 工作线程是否在运行. 0-未运行; 1-在运行.
    uint8_t sensorChosen;   ///< 感应器选择 1-下光纤感应器; 2-上光纤感应器
    uint8_t givenMode;      ///< 送料模式 1-有料不可送; 2-有料可送
    uint8_t givenOK;      ///< ������� 1-; 2-����δ���
    uint8_t CUTOK;      ///< ������� 1-; 2-����δ���
    uint8_t RSTOK;      ///< ������� 1-; 2-����δ���
    uint8_t SLOK;      ///< ������� 1-; 2-����δ���
    uint8_t RSTAlarm; 	//电机复位报警标志位
    uint8_t Motor_orientation;
    uint8_t givenOnceTriggerByUIorIO;///< 触发一次送料,1：下光纤，2：上光纤
    uint8_t CUTOnceTriggerByUIorIO;	///< 触发一次切料
    uint8_t RSTOnceTriggerByUIorIO;	///< 触发一次剥刀复位
    uint8_t STOPOnceTriggerByUIorIO;///< 触发一次停止
    uint8_t letOnceTrigger;			///< 触发一次放料
    uint8_t refreshMatalUI;			///< 更新有无料UI
    uint8_t isAutoLetMetalEnable;	///< 自动放料功能是否使能
    uint8_t isUpShouEnable;			///< 自动上收料是否使能
    uint8_t isUpShouSensorEnable;
    uint8_t isDownShouEnable;		///< 自动下收料是否使能
	uint8_t isLackMaterral_NC;		///< 1，缺料使用常开感应器
    uint8_t isInEmergencyStopEnable;	///< 使用内部急停
    uint8_t isInitSwOnLINEEnable;	///< 使用外部初始化信号切换联机模式
    uint8_t isFeedInPlaceEnable;	///< 使用送料到位，否则为后限位
    uint8_t MotorGivenDir;			///< 送料电机方向设置
	uint8_t MotorLetDir;			///< 放料电机方向设置
	uint8_t MotorUpShouDir;			///< 上收料电机方向设置
	uint8_t MotorDownShouDir;		///< 下收料电机方向设置

	uint8_t Lableposition;			// 贴标位置选择 1，位置1  2，位置2
	uint8_t doInit;					///< 执行初始化

    uint8_t doGohome;					///< 执行回原点
	uint8_t doGohomeOK;				//回原点完成

	uint8_t doLabeling;				///< 执行贴标动作
	uint8_t doLabelingOK;			///< 执行贴标动作完成

	uint8_t doTaking;				///< 执行取标动作
	uint8_t doTakingOK;				///< 执行取标动作完成


    uint8_t doLabelPosi;			///< 执行去贴标动作
	uint8_t doLabelposiOK;			///< 执行去贴标动作完成

    uint8_t doTakePosi;				///< 执行去取标动作
	uint8_t doTakePosiOK;			///< 执行去取标动作完成
	uint8_t GoBackTakePosi;			///< 贴标位置回到取标位置动作
    uint8_t isMetalOnDeck;			///< 是否有料
    uint8_t bodaoGoBack;			///< 剥刀换料后撤标志位
    uint8_t AlarmFlag;				///< 报警代码
    uint8_t Err;
    uint8_t Page;					//页面序号



    uint8_t UpStatus;				//上收料状态，0 停止，1，正转启动，2反转启动
    uint8_t DownStatus;				//下收料状态，0 停止，1，启动
    uint8_t FLStatus;				//放料料状态，0 停止，1，启动
	uint8_t SLmode;					//0,距离模式，1传感模式
	uint8_t FLmode;					//0,距离模式，1传感模式
	uint8_t UPSLmode;				//0,距离模式，1传感模式
	uint8_t DOWNSLmode;				//0,距离模式，1传感模式
	uint8_t LackMaterral;			//0，缺料 1有料，
//	uint8_t isMaterialStateOn;		//感应器亮
//	uint8_t isMaterialStateOff;		//感应器灭
	uint8_t MaterStateOld;
	uint8_t MaterStateNew;
	uint8_t isChangeCount;          //感应器变化次数
    volatile uint8_t fiberNo;				///< 光纤编号
}SystemPara;

#define     STATUS_UNINIT           0
#define     STATUS_ONLINE           1
#define     STATUS_AUTO             2
#define     STATUS_FREERUN          3
#define     STATUS_HUANLIAO         4

/*
typedef struct _controlPara
{
    float givenSpeed;           ///< 送料速度
    float givenOffset;          ///< 送料偏移
    float letSpeed;             ///< 放料速度
    float letOffset;            ///< 放料偏移
    float boSpeed;              ///< 剥料速度
    float boDistance;           ///< 剥料回撤行程
    float shouSpeed;            ///< 收料速度
    float shouDistance;         ///< 收料行程
    float oriSpeed;             ///< 回原点速度
    float oriOffset;            ///< 原点偏移

    float givenCur;             ///< 送料电机电流
    float givenMS;              ///< 送料电机细分
    float letCur;               ///< 放料电机电流
    float letMS;                ///< 放料电机细分
    float boCur;                ///< 剥料电机电流
    float boMS;                 ///< 剥料电机细分
    float shouCur;              ///< 收料电机电流
    float shouMS;               ///< 收料电机细分
}ControlPara;
*/

#define     PARA_GIVENSPEED     0
#define     PARA_GIVENOFFSET    1
#define     PARA_LETSPEED       2
#define     PARA_LETOFFSET      3
#define     PARA_BOSPEED        4
#define     PARA_BODISTANCE     5
#define     PARA_SHOUSPEED      6
#define     PARA_SHOUDISTANCE   7
#define     PARA_ORISPEED       8
#define     PARA_ORIOFFSET      9
#define     PARA_GIVENCUR       10
#define     PARA_GIVENMS        11
#define     PARA_LETCUR         12
#define     PARA_LETMS          13
#define     PARA_BOCUR          14
#define     PARA_BOMS           15
#define     PARA_SHOUCUR        16
#define     PARA_SHOUMS         17

extern volatile SystemPara systemPara;
extern uint8_t ADDR_5000_L[ADDR5000_NUM];
extern uint8_t ADDR_5000_H[ADDR5000_NUM];
extern uint8_t ADDR_6000_L[ADDR6000_NUM];
extern uint8_t ADDR_6000_H[ADDR6000_NUM];
extern uint8_t ADDR_7000_L[ADDR7000_NUM];
extern uint8_t ADDR_7000_H[ADDR7000_NUM];

#endif /* APPLICATIONS_PARAMANAGER_PARAMANAGER_H_ */
