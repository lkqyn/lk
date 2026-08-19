#ifndef _MOTORCTRL_H_
#define _MOTORCTRL_H_

#include "stm32f1xx_hal.h"
#include "lv8731v/lv8731v.h"
#include "sysconfig/sysconfig.h"

typedef struct _accelerate
{
    uint16_t inc;
    uint16_t dec;
}Accelerate;

typedef enum _MotorRunStatus
{
    Motor_Stop,             	///< 停止
    Motor_incSpeed,         	///< 电机加速
    Motor_Run,              	///< 运行
    Motor_decSpeed,         	///< 减速
    Motor_runForever,       	///< 连续运行
    Motor_run2Stop,         	///< 运行到停止
    Motor_runWithOffset,    	///< 带偏移运行
}MotorRunStatus;

enum MotorStopEvent // 电机停止事件
{
	EVENT_none,
	EVENT_exceedPulseCountLimit,
	EVENT_letMotorExceedPulseCountLimit,
};


typedef struct _pulseCount
{
    uint32_t switchRun;     ///< 电机开始运行的脉冲计数
    uint32_t switchDec;     ///< 电机开始减速的脉冲计数
    uint32_t switchStop;    ///< 停止运动的脉冲计数
}PulseCount;

typedef struct _motorPara
{
    uint16_t amp;                   ///< 电机的电流限制, 单位2mA
    uint8_t ms;                     ///< 电机的微步细分
    Accelerate acc;                 ///< 电机的加速度信息，包含两个字段inc和dec，分别表示加速度和减速度。
    uint32_t aimSpeed;              ///< 电机的目标速度。
    uint16_t distance;              ///< 电机的行程距离。
    uint16_t curSpeed;              ///< 电机的当前速度。
    PulseCount pulseCount;          ///< 电机的脉冲计数信息。
    MotorRunStatus status;          ///< 电机的运行状态
    uint16_t mm2pulse;              ///< 电机的脉冲数与毫米数的转换比例，单位为2个脉冲数/毫米
    uint16_t arrTable[CONF_S_LEN];  ///< TIM ARR值的数组，用于生成PWM波形
    //uint16_t index;               ///< 锟斤拷锟节诧拷锟斤拷锟斤拷锟斤拷锟斤拷
    int16_t index;                	///< 电机内部用于计数的索引
	int32_t pulseCountLimit;        ///< 电机的脉冲计数限制，用于run2Stop模式
    uint8_t beginRunOffset;         ///< 开始运行时的偏移标志
    uint32_t offsetPulseCount;      ///< 偏移脉冲计数
    enum MotorStopEvent stopEvent;  ///< 电机的停止事件
    uint8_t motorHadRun;
    uint8_t motorDir;				///< 电机的方向
    uint8_t motorRunStatus;			///< 电机运行状态
}MotorPara;

void MC_config(void);
void MC_motorMoveForever(uint8_t motorNo, uint32_t speed, uint8_t cw);
void MC_motorMoveForeverWithPulseCountLimited(uint8_t motorNo,uint8_t cw,int32_t pulseCountLimited);
void MC_motorStopAll(void);
void MC_motorStop(uint8_t motorNo);
void MC_motorStopStatus(uint8_t motorNo);
void MC_motorStopDec(uint8_t motorNo);
void MC_motorStopAllEmergency(void);
void MC_setMicroStep(uint8_t motorNo,uint8_t msCode);
void MC_debug(void *pv);
void MC_5000toPara(uint8_t addr);
void MC_AmpDecay(uint8_t motorNo);
void MC_cmd(uint8_t motorNo, uint8_t cmd);
void MC_motorMovePulse(uint8_t motorNo,uint8_t cw,uint32_t pulseCount);
void MC_motorMoveDistance(uint8_t motorNo, uint32_t speed, uint8_t cw, uint32_t mm);
void MC_motorMoveDistance1(uint8_t motorNo,uint8_t cw, uint32_t mm);
uint16_t MC_pulse2mm(uint8_t motorNo);
void MC_GMOffsetRunDistance(uint8_t dir, uint32_t Distance);
void MC_BMOffsetRunDistance(uint8_t dir, uint32_t Distance);
void MC_GMOffsetRun(void);
void MC_BMOffsetRun(void);
uint32_t MC_mm2pulse(uint8_t motorNo,float mm);
uint32_t MC_mm2pulse1(uint8_t motorNo,float mm);
void MC_motorMoveDistanceNoAcc(uint8_t motorNo,uint8_t cw,uint16_t mm);
void MC_motorContinueMoveDistanceNoDec(uint8_t motorNo,uint16_t mm);
void MC_motorContinueMoveDistance(uint8_t motorNo,uint16_t mm);
void MC_motorContinueMovePulseNoDec(uint8_t motorNo,uint32_t pulseCount);
void MC_refreshAllParameter(void);
void MC_motorRunAndOffsetByEXTI(uint8_t motorNo, uint32_t speed, uint8_t cw,uint32_t offsetPulse,uint32_t limitPulse);

void MC_motorMoveForeverS(uint8_t motorNo,uint8_t cw);
void MC_calculateFactor4exp(void);

#define     GIVEN_MOTOR         CONF_GM_NO       ///< 送料电机
#define     LET_MOTOR           CONF_LM_NO       ///< 放料电机
#define     BO_MOTOR            CONF_BM_NO       ///< 剥料电机
#define     SHOU_MOTOR          CONF_SM_NO       ///< 收料电机

#define     GM_MM_PULSE         CONF_GM_MM_PULSE     ///< 送料电机运动1mm所需要的脉冲数, 全步时
#define     LM_MM_PULSE         CONF_LM_MM_PULSE     ///< 放料电机运动1mm所需要的脉冲数,全步时
#define     BM_MM_PULSE         CONF_BM_MM_PULSE     ///< 剥料电机运动1mm所需要的脉冲数,全步时
#define     SM_MM_PULSE         CONF_SM_MM_PULSE     ///< 送料电机运动1mm所需要的脉冲数,全步时

#define     DOWN_MOTOR          3//CONF_BM_NO	// 下收料电机
#define     UP_MOTOR          	CONF_SM_NO		//上收料电机

extern MotorPara motor_ch[4];
extern uint8_t MOTOR_offsetOrigin;

#endif
