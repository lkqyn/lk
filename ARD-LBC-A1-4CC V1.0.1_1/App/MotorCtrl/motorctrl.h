#ifndef _MOTORCTRL_H_
#define _MOTORCTRL_H_

#include "stm32f1xx_hal.h"
#include "lv8731v/lv8731v.h"
#include "sysconfig/sysconfig.h"
//#include "signal_event/signal_event.h"

typedef struct _accelerate
{
    uint16_t inc;
    uint16_t dec;
}Accelerate;

typedef enum _MotorRunStatus
{
    Motor_Stop,             ///< ���ͣ�
    Motor_incSpeed,         ///< �������
    Motor_Run,              ///< �����������
    Motor_decSpeed,          ///< �������
    Motor_runForever,        ///< ��ʱ������
    Motor_run2Stop,          ///< ����ֱ�ӵ�ֹͣ״̬, ���ڲ���Ҫ�Ӽ��ٵ�����
    Motor_runWithOffset,    ///< ���ⲿ�жϲ���֮ǰһֱ����, һ����Ӧ�ⲿ�жϲ�����ʼ��ƫ��
}MotorRunStatus;

enum MotorStopEvent
{
	EVENT_none,
	EVENT_exceedPulseCountLimit,
	EVENT_letMotorExceedPulseCountLimit,
};


typedef struct _pulseCount
{
    uint32_t switchRun;     ///< �л�������״̬ʱ��������
    uint32_t switchDec;     ///< �л�������״̬ʱ��������
    uint32_t switchStop;    ///< �л���ֹͣ״̬ʱ��������
}PulseCount;

typedef struct _motorPara
{
    uint16_t amp;                   ///< ����ֵ, 2λС��, ����
    uint8_t ms;                     ///< ϸ��
    Accelerate acc;                 ///< ���ٶ�/���ٶ�
    uint16_t aimSpeed;              ///< Ŀ���ٶ�
    uint16_t distance;              ///< �г�
    uint16_t curSpeed;              ///< ��ǰ�ٶ�
    PulseCount pulseCount;          ///< 3���ؼ�������
    MotorRunStatus status;            ///< �������״�
    uint16_t mm2pulse;              ///< �������1mm����������, 2λС��, ����
    uint16_t arrTable[CONF_S_LEN];   ///< TIM ARR��
    uint16_t index;                 ///< ���ڲ���������
    int32_t pulseCountLimit;        ///< ����������, ����run2Stopģʽ
    uint8_t beginRunOffset;         ///< ��ʼ��ƫ�Ʊ�־λ
    uint32_t offsetPulseCount;      ///< ƫ����������
    enum MotorStopEvent stopEvent;  ///< ֹͣ�¼�
    uint8_t motorHadRun;
}MotorPara;

void MC_config(void);
void MC_motorMoveForever(uint8_t motorNo,uint8_t cw);
void MC_motorMoveForeverWithPulseCountLimited(uint8_t motorNo,uint8_t cw,int32_t pulseCountLimited);
void MC_motorStopAll(void);
void MC_motorStop(uint8_t motorNo);
void MC_motorStopAllEmergency(void);
void MC_setMicroStep(uint8_t motorNo,uint8_t msCode);
void MC_debug(void *pv);
void MC_5000toPara(uint8_t addr);
void MC_AmpDecay(uint8_t motorNo);
void MC_cmd(uint8_t motorNo, uint8_t cmd);
void MC_motorMovePulse(uint8_t motorNo,uint8_t cw,uint32_t pulseCount);
void MC_motorMoveDistance(uint8_t motorNo,uint8_t cw, uint32_t mm);
uint16_t MC_pulse2mm(uint8_t motorNo);
void MC_GMOffsetRun(void);
uint32_t MC_mm2pulse(uint8_t motorNo,uint16_t mm);
void MC_motorMoveDistanceNoAcc(uint8_t motorNo,uint8_t cw,uint16_t mm);
void MC_motorContinueMoveDistanceNoDec(uint8_t motorNo,uint16_t mm);
void MC_motorContinueMoveDistance(uint8_t motorNo,uint16_t mm);
void MC_motorContinueMovePulseNoDec(uint8_t motorNo,uint32_t pulseCount);

void MC_motorRunAndOffsetByEXTI(uint8_t motorNo,uint8_t cw,uint32_t offsetPulse,uint32_t limitPulse);

void MC_motorMoveForeverS(uint8_t motorNo,uint8_t cw);
void MC_calculateFactor4exp(void);
void MC_motorMovePulseWithoutIncDec(uint8_t motorNo, uint8_t cw,uint32_t pulseCount);
void MC_motorStopDec(uint8_t motorNo);
void MC_defaultArrTable(void);
void MC_motorMovePulseWithoutCalculateARR(uint8_t motorNo,uint8_t cw,uint32_t pulseCount);
void MC_motorMoveDistanceWithoutCalculateARR(uint8_t motorNo,uint8_t cw, uint32_t mm);
void MC_motorMovePulseWithoutIncDec(uint8_t motorNo, uint8_t cw,uint32_t pulseCount);
void MC_motorMoveDistanceWithoutIncDec(uint8_t motorNo,uint8_t cw, uint32_t mm);
/* 
 ���ֵ����ͨ���Ŷ��, ���޸�����.
*/
#define     GIVEN_MOTOR         CONF_GM_NO       ///< ���ϵ��
#define     LET_MOTOR           CONF_LM_NO       ///< ���ϵ��
#define     BO_MOTOR            CONF_BM_NO       ///< ���ϵ��
#define     SHOU_MOTOR          CONF_SM_NO       ///< ���ϵ��

#define     GM_MM_PULSE         CONF_GM_MM_PULSE     ///< ���ϵ���˶�1mm��Ҫ������, ȫ��ʱ, �迼�Ǽ��ٻ�
#define     LM_MM_PULSE         CONF_LM_MM_PULSE     ///< ���ϵ���˶�1mm��Ҫ������, ȫ��ʱ, �迼�Ǽ��ٻ�
#define     BM_MM_PULSE         CONF_BM_MM_PULSE     ///< ���ϵ���˶�1mm��Ҫ������, ȫ��ʱ, �迼�Ǽ��ٻ�
#define     SM_MM_PULSE         CONF_SM_MM_PULSE     ///< ���ϵ���˶�1mm��Ҫ������, ȫ��ʱ, �迼�Ǽ��ٻ�

#define     UP_MOTOR            CONF_SM_NO          ///< �����ϵ��
#define     DOWN_MOTOR          CONF_LM_NO          ///< �����ϵ��

extern MotorPara motor_ch[4];
extern uint8_t MOTOR_offsetOrigin;      ///< ԭ��ƫ�Ʒ���

#endif
