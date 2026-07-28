/**
 @file
*/
#include "motorctrl/motorctrl.h"
#include "pwm1/pwm1.h"
#include "pwm2/pwm2.h"
#include "pwm3/pwm3.h"
#include "pwm4/pwm4.h"
#include "lv8731v/lv8731v.h"
//#include "tim5/tim5.h"
#include "mcp4728/mcp4728.h"
#include "paramanager/paramanager.h"
#include "motorctrl/s_func_factor.h"
#include "userfunc/userfunc.h"
//#include "givenevent/givenevent.h"
//#include "signal_event/signal_event.h"
#include "dwin/gui.h"
#include "inboardio/inboardio.h"

typedef void (*PWMCmd)(void);
typedef void (*SetFrequent)(uint32_t );

static const PWMCmd pwmStop[] = {PWM1_stop,PWM2_stop,PWM3_stop,PWM4_stop};
static const PWMCmd pwmStart[] = {PWM1_start,PWM2_start,PWM3_start,PWM4_start};
//static const SetFrequent setFrequent[] = {PWM1_setFrequent,PWM2_setFrequent,PWM3_setFrequent,PWM4_setFrequent};
MotorPara motor_ch[4] = {0};

// ʱ�䳣��, �ó�����ֵӦ��TIM5�ж�ʱ��һ��, ����ÿ�θı��ٶȵ�ʱ��һ��
#define TIME_CONST_K        0.002f

uint8_t SsorLowCnt1 = 0,SsorLowCnt2 = 0,SsorLowCnt3 = 0,SsorLowCnt4 = 0,SsorLowCnt5 = 0;
uint8_t SsorLowCnt11 = 0;
/**
 ԭ��ƫ�Ʒ���. 1-��ƫ��; 2-��ƫ��
*/
uint8_t MOTOR_offsetOrigin;

void MC_motorMovePulseCount(uint8_t motorNo, uint32_t pulseCount);
static void MC_refreshParameter(void);
static uint8_t calculatePulseRatio(uint8_t motorNo);
void MC_refreshAmp(uint8_t motorNo);
void MC_genArrTable(uint8_t motorNo,uint16_t aimFreq);
static void MC_motorMovePulseCountWithoutIncDec(uint8_t motorNo, uint32_t pulseCount);


/**
 @brief ������Ƴ�ʼ��
 ����LV8731V, PWM1, PWM2, PWM3, PWM4, TIM5���г�ʼ��.
*/
void MC_config(void)
{
    uint8_t i;
    
    for(i=0;i<4;i++)
    {
        motor_ch[i].status = Motor_Stop;
    }
    
    LV8731V_config();
    PWM1_config();
    PWM2_config();
    PWM3_config();
    PWM4_config();
    //TIM5_config();
    MCP4728_config();
    
//    motor_ch[GIVEN_MOTOR].mm2pulse = GM_MM_PULSE;
    motor_ch[LET_MOTOR].mm2pulse = LM_MM_PULSE;
    motor_ch[GIVEN_MOTOR].mm2pulse = motor_ch[GIVEN_MOTOR].amp;
    //motor_ch[BO_MOTOR].mm2pulse = BM_MM_PULSE;
    motor_ch[BO_MOTOR].mm2pulse = motor_ch[BO_MOTOR].amp;
    motor_ch[SHOU_MOTOR].mm2pulse = SM_MM_PULSE; 
    
    //ʹ��4ͨ�����
//    MC_cmd(0,1);
//    MC_cmd(1,1);
//    MC_cmd(2,1);
//    MC_cmd(3,1);
    
    // ����˥��
    MC_AmpDecay(0);
    MC_AmpDecay(1);
    MC_AmpDecay(2);
    MC_AmpDecay(3);
    
    MC_motorStop(0);
    MC_motorStop(1);
    MC_motorStop(2);
    MC_motorStop(3);
}

/**
 @brief ָ������˶�ָ��������
 @param -motorNo- ָ�����
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4 
 @param -cw- ���з���
    @arg -0- ˳ʱ�뷽��
    @arg -1- ��ʱ�뷽��
 @param -pulseCount- ������
*/
void MC_motorMovePulse(uint8_t motorNo,uint8_t cw,uint32_t pulseCount)
{
    MC_refreshParameter();
    LV8731V_setCW(motorNo,cw);
    LV8731V_setMicroStep(motorNo,motor_ch[motorNo].ms);
    MC_refreshAmp(motorNo);
    MC_motorMovePulseCount(motorNo,pulseCount);
}

/**
 @brief ָ������˶�ָ��������
 @param -motorNo- ָ�����
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4 
 @param -pulseCount- ������
*/
void MC_motorMovePulseCount(uint8_t motorNo, uint32_t pulseCount)
{
    uint16_t tempFreq;
    
    pwmStop[motorNo]();
    
    motor_ch[motorNo].pulseCount.switchStop = pulseCount;
    
    if(pulseCount < 100)
    {// �˶�����ܶ�, ����Ҫ��������, ��5mm/s���ٶ�����
    	tempFreq = CONF_S_TIMER_FREQ / (5 * motor_ch[motorNo].mm2pulse * calculatePulseRatio(motorNo));
        motor_ch[motorNo].status = Motor_run2Stop;
    }
    else
    {
        MC_genArrTable(motorNo,motor_ch[motorNo].aimSpeed);
        tempFreq = motor_ch[motorNo].arrTable[0];
        
        motor_ch[motorNo].pulseCount.switchRun = 100;
        motor_ch[motorNo].pulseCount.switchDec = motor_ch[motorNo].pulseCount.switchStop - 100;
        
        motor_ch[motorNo].status = Motor_incSpeed;
        motor_ch[motorNo].index = 0;
    }

    switch(motorNo)
    {
        case 0:
            PWM1_pulseCount=0;
            TIM1->ARR = tempFreq;
            TIM1->CCR1 = TIM1->ARR >> 1;
        break;
        case 1:
            TIM2->ARR = tempFreq;
            TIM2->CCR1 = TIM2->ARR >> 1;
            PWM2_pulseCount=0;
        break;
        case 2:
            TIM3->ARR = tempFreq;
            TIM3->CCR3 = TIM3->ARR >> 1;
            PWM3_pulseCount=0;
        break;
        case 3:
            TIM4->ARR = tempFreq;
            TIM4->CCR1 = TIM4->ARR >> 1;
            PWM4_pulseCount=0;
        break;
    }
    
    motor_ch[motorNo].pulseCount.switchStop = pulseCount;

    LV8731V_cmd(motorNo,1);
    pwmStart[motorNo]();
}

/**
 * 移动指定脉冲数.
 */
static void MC_motorMovePulseCountWithoutIncDec(uint8_t motorNo, uint32_t pulseCount)
{
    uint16_t tempFreq;

    pwmStop[motorNo]();

    motor_ch[motorNo].pulseCount.switchStop = pulseCount;

    motor_ch[motorNo].status = Motor_run2Stop;
    tempFreq = CONF_S_TIMER_FREQ / (CONF_S_SPEED_MIN + (motor_ch[motorNo].aimSpeed - CONF_S_SPEED_MIN) * expFactor[CONF_S_FACTOR_NO][CONF_S_LEN]);

    switch(motorNo)
    {
        case 0:
            PWM1_pulseCount=0;
            TIM1->ARR = tempFreq;
            TIM1->CCR1 = TIM1->ARR >> 1;
        break;
        case 1:
            TIM2->ARR = tempFreq;
            TIM2->CCR1 = TIM2->ARR >> 1;
            PWM2_pulseCount=0;
        break;
        case 2:
            TIM3->ARR = tempFreq;
            TIM3->CCR3 = TIM3->ARR >> 1;
            PWM3_pulseCount=0;
        break;
        case 3:
            TIM4->ARR = tempFreq;
            TIM4->CCR1 = TIM4->ARR >> 1;
            PWM4_pulseCount=0;
        break;
    }

    motor_ch[motorNo].pulseCount.switchStop = pulseCount;

    LV8731V_cmd(motorNo,1);
    pwmStart[motorNo]();
}

/**
 @brief ����˶�ָ������
 @param -motorNo- ָ�����
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4 
 @param -cw- ����˶�����
    @arg -0- ˳ʱ��
    @arg -1- ��ʱ��
 @param -mm- �˶�����, ��λmm. 2λ����С��, ��100�൱��1mm
 @note ʹ�ø÷�������� GM_MM_PULSE �Ⱥ���ж���.
*/ 
void MC_motorMoveDistance(uint8_t motorNo,uint8_t cw, uint32_t mm)
{
    uint32_t pulseCount;
    uint8_t pulseRatio;     ///< ��������ϸ����������Ĳ���.
    
    if(mm == 0)return;
    MC_refreshParameter();
    
    pulseRatio = calculatePulseRatio(motorNo);
    pulseCount = (mm / 100.0f) * motor_ch[motorNo].mm2pulse * pulseRatio;
    MC_motorMovePulse(motorNo,cw,pulseCount);
}

/**
 * 移动脉冲数不带加减速
 */
void MC_motorMovePulseWithoutIncDec(uint8_t motorNo, uint8_t cw,uint32_t pulseCount)
{
	MC_refreshParameter();
	LV8731V_setCW(motorNo,cw);
	LV8731V_setMicroStep(motorNo,motor_ch[motorNo].ms);
	MC_refreshAmp(motorNo);
	MC_motorMovePulseCountWithoutIncDec(motorNo,pulseCount);
}

/**
 * 电机移动指定距离且不带加减速
 */
void MC_motorMoveDistanceWithoutIncDec(uint8_t motorNo,uint8_t cw, uint32_t mm)
{
    uint32_t pulseCount;
    uint8_t pulseRatio;     ///< ��������ϸ����������Ĳ���.

    if(mm == 0)return;
    MC_refreshParameter();

    pulseRatio = calculatePulseRatio(motorNo);
    pulseCount = (mm / 100.0f) * motor_ch[motorNo].mm2pulse * pulseRatio;
    MC_motorMovePulseWithoutIncDec(motorNo,cw,pulseCount);
}

/**
 * 使用4通道的目标速度生成各自加速表
 */
void MC_defaultArrTable(void)
{
	uint8_t motorNo;
	uint16_t i;
	uint16_t factorMaxMin;

	for(motorNo=0;motorNo<4;motorNo++)
	{
		if(motor_ch[motorNo].aimSpeed > CONF_S_SPEED_MIN)
		{
			factorMaxMin = motor_ch[motorNo].aimSpeed - CONF_S_SPEED_MIN;

			for(i=0;i<CONF_S_LEN;i++)
			{
				motor_ch[motorNo].arrTable[i] = CONF_S_TIMER_FREQ / (CONF_S_SPEED_MIN + factorMaxMin * expFactor[CONF_S_FACTOR_NO][i]);
			}
		}

		else
		{
			for(i=0;i<CONF_S_LEN;i++)
			{
				motor_ch[motorNo].arrTable[i] = CONF_S_TIMER_FREQ / motor_ch[motorNo].aimSpeed;
			}
		}
	}

}

/**
 @brief ʹ�ܵ��
 @param -motorNo- ָ�����
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4
 @param -cmd- ʹ��ָ��
    @arg 0- ʧ��
    @arg 1- ʹ��
*/
void MC_cmd(uint8_t motorNo, uint8_t cmd)
{
    LV8731V_cmd(motorNo,cmd);
}

/**
 @brief ֹͣ���е��
*/
void MC_motorStopAll(void)
{
    uint8_t i;
    for(i=0;i<4;i++)
    {
        pwmStop[i]();
        motor_ch[i].status = Motor_Stop;
        MC_AmpDecay(i);
    }
}

/**
 * 用于紧急停止的停止所有电机, 会关闭通道使能.
 */
void MC_motorStopAllEmergency(void)
{
	uint8_t i;
	for (i = 0; i < 4; i++)
	{
		pwmStop[i]();
		motor_ch[i].status = Motor_Stop;
		MC_AmpDecay(i);
		LV8731V_cmd(i, 0);
	}
}

uint16_t MC_pulse2mm(uint8_t motorNo)
{
    uint32_t pulseCount;
    uint8_t pulseRatio;
    
    switch(motorNo)
    {
        case 0:pulseCount = PWM1_pulseCount;
            break;
        case 1:pulseCount = PWM2_pulseCount;
            break;
        case 2:pulseCount = PWM3_pulseCount;
            break;
        case 3:pulseCount = PWM4_pulseCount;
            break;
    }
    
    pulseRatio = calculatePulseRatio(motorNo);
    
    return pulseCount * 1.0f / motor_ch[motorNo].mm2pulse / pulseRatio * 100;
}

/**
 @brief ָֹͣ�����
 @param -motorNo- ָ�����
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4    
*/
void MC_motorStop(uint8_t motorNo)
{
    pwmStop[motorNo]();
    motor_ch[motorNo].status = Motor_Stop;
    MC_AmpDecay(motorNo);
}

/**
 * 带减速停止电机
 */
void MC_motorStopDec(uint8_t motorNo)
{
	motor_ch[motorNo].status = Motor_decSpeed;
}


/**
 @brief ����ϸ��
 @param -motorNo- ָ�����
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4
 @param -msCode- ϸ��
    @arg Full_Step- ȫ��
    @arg Half_Step- 2ϸ��
    @arg Quarter_Step- 4ϸ��
    @arg MS_16- 16ϸ��
*/
void MC_setMicroStep(uint8_t motorNo,uint8_t msCode)
{
    LV8731V_setMicroStep(motorNo,msCode);
}

void MC_debug(void *pv)
{
    MC_motorMovePulseCount(1,15000);
}

void MC_refreshMicroStep(uint8_t motorNo)
{
    LV8731V_setMicroStep(motorNo,motor_ch[motorNo].ms);
}

void MC_refreshAmp(uint8_t motorNo)
{
    float V;

    V = motor_ch[motorNo].amp * 0.011f; // cur * 0.01 * 1.1
    if(V > 3.25f)
    {
        V = 3.25f;
    }
    MCP4728_setOutput(motorNo,(uint16_t)(V / 0.0008));
}

void MC_AmpDecay(uint8_t motorNo)
{
    float V;

    if(motorNo == LET_MOTOR)
    {
    	V = (motor_ch[motorNo].amp * 0.6f) * 0.011f; // cur * 0.01 * 1.1
    }
    else
    {
    	V = (motor_ch[motorNo].amp * 0.4f) * 0.011f; // cur * 0.01 * 1.1
    }
    if(V > 3.25f)
    {
        V = 3.25f;
    }
    MCP4728_setOutput(motorNo,(uint16_t)(V / 0.0008));
}

void MC_refreshSpeed(uint8_t motorNo)
{
    
}

void MC_refreshAllParameter(uint8_t motorNo)
{
    MC_refreshMicroStep(motorNo);
    MC_refreshAmp(motorNo);
}

/**
 @brief ��������Ʋ�����0x5000��ַ��motor_ch
*/
void MC_5000toPara(uint8_t addr)
{
    switch(addr)
    {
        case 0xa:
            motor_ch[GIVEN_MOTOR].amp = PARA_readParameter(addr + 0x5000);
            MC_refreshAmp(GIVEN_MOTOR);
            break;
        case 0xb:
            motor_ch[GIVEN_MOTOR].ms = PARA_readParameter(addr + 0x5000);
            MC_refreshMicroStep(GIVEN_MOTOR);
            break;
        case 0x0c:
            motor_ch[LET_MOTOR].amp = PARA_readParameter(addr + 0x5000);
            MC_refreshAmp(LET_MOTOR);
            break;
        case 0x0d:
            motor_ch[LET_MOTOR].ms = PARA_readParameter(addr + 0x5000);
            MC_refreshMicroStep(LET_MOTOR);
            break;
        case 0x0e:
            motor_ch[BO_MOTOR].amp = PARA_readParameter(addr + 0x5000);
            MC_refreshAmp(BO_MOTOR);
            break;
        case 0x0f:
            motor_ch[BO_MOTOR].ms = PARA_readParameter(addr + 0x5000);
            MC_refreshMicroStep(BO_MOTOR);
            break;
        case 0x10:
            motor_ch[SHOU_MOTOR].amp = PARA_readParameter(addr + 0x5000);
            MC_refreshAmp(SHOU_MOTOR);
            break;
        case 0x11:
            motor_ch[SHOU_MOTOR].ms = PARA_readParameter(addr + 0x5000);
            MC_refreshMicroStep(SHOU_MOTOR);
            break;
    }
}

/**
 @brief ���µ�����в���
*/
static void MC_refreshParameter(void)
{
    // ����ϸ��
    motor_ch[GIVEN_MOTOR].ms = PARA_readParameter(0x500b);
    motor_ch[LET_MOTOR].ms = PARA_readParameter(0x500d);
    motor_ch[BO_MOTOR].ms = PARA_readParameter(0x500F);
    motor_ch[SHOU_MOTOR].ms = PARA_readParameter(0x5011);


    // ���µ���
    motor_ch[GIVEN_MOTOR].amp = PARA_readParameter(0x500a);
    motor_ch[LET_MOTOR].amp = PARA_readParameter(0x500C);
    motor_ch[BO_MOTOR].amp = PARA_readParameter(0x500E);
    motor_ch[SHOU_MOTOR].amp = PARA_readParameter(0x5010);

    motor_ch[GIVEN_MOTOR].mm2pulse = motor_ch[GIVEN_MOTOR].amp;
    motor_ch[BO_MOTOR].mm2pulse = motor_ch[BO_MOTOR].amp;
    // ����Ŀ���ٶ�
    // motor_ch[motorNo].mm2pulse * PARA_readParameter(0x5000) * 0.01;
    motor_ch[GIVEN_MOTOR].aimSpeed = 1.2f * PARA_readParameter(0x5000) * motor_ch[GIVEN_MOTOR].mm2pulse * 0.01f * calculatePulseRatio(GIVEN_MOTOR);
    motor_ch[LET_MOTOR].aimSpeed = PARA_readParameter(0x5002) * motor_ch[LET_MOTOR].mm2pulse * 0.01f * calculatePulseRatio(LET_MOTOR);
    motor_ch[BO_MOTOR].aimSpeed =  PARA_readParameter(0x5013) * motor_ch[BO_MOTOR].mm2pulse * 0.01f * calculatePulseRatio(BO_MOTOR);
    motor_ch[SHOU_MOTOR].aimSpeed = PARA_readParameter(0x5004) * motor_ch[SHOU_MOTOR].mm2pulse * 0.01f * calculatePulseRatio(SHOU_MOTOR);
}

static uint8_t calculatePulseRatio(uint8_t motorNo)
{
    uint8_t pulseRatio;
    
    switch(motor_ch[motorNo].ms)
    {
        case 1:pulseRatio = 1;break;
        case 2:pulseRatio = 2;break;
        case 3:pulseRatio = 4;break;
        case 4:pulseRatio = 16;break;
        default:pulseRatio=1;break;
    }
    
    return pulseRatio;
}

/**
 �������ϵ��ƫ��set����
*/
void MC_GMOffsetRun(void)
{
    uint8_t speed_h,speed_l;
    
    speed_h = ADDR_5000_H[0];
    speed_l = ADDR_5000_L[0];
    
    ADDR_5000_H[0] = CONF_GM_OFFSET_SETSPEED >> 8;
    ADDR_5000_L[0] = CONF_GM_OFFSET_SETSPEED & 0xff;
    
    MC_motorMoveForever(0,0);
    
    ADDR_5000_H[0] = speed_h;
    ADDR_5000_L[0] = speed_l;
}

/**
 ����ת����������
 @param -motorNo- ָ�����
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4
 @param -mm- ����. ��λmm. 2λ����С��
*/
uint32_t MC_mm2pulse(uint8_t motorNo,uint16_t mm)
{
    uint8_t pulseRatio;
    uint32_t pulseCount;
    
    if(mm == 0)return 0;
    MC_refreshParameter();
    
    pulseRatio = calculatePulseRatio(motorNo);
    pulseCount = (mm / 100.0) * motor_ch[motorNo].mm2pulse * pulseRatio;
    
    return pulseCount;
}

/**
 ��MC_motorMoveForever֮���������pulseCount������.
*/
void MC_motorContinueMovePulseNoDec(uint8_t motorNo,uint32_t pulseCount)
{
    if(pulseCount == 0)
    {
        pwmStop[motorNo]();
        motor_ch[motorNo].status = Motor_Stop;
        return;
    }
    
    motor_ch[motorNo].pulseCount.switchStop = pulseCount;
    
    switch(motorNo)
    {
        case 0:
            PWM1_pulseCount=0;
        break;
        case 1:
            PWM2_pulseCount=0;
        break;
        case 2:
            PWM3_pulseCount=0;
        break;
        case 3:
            PWM4_pulseCount=0;
        break;
    }
    
    motor_ch[motorNo].status = Motor_run2Stop;
}

/**
 ��������˶�ָ������
*/
void MC_motorContinueMoveDistance(uint8_t motorNo,uint16_t mm)
{
    if(mm == 0)
    {
        pwmStop[motorNo]();
        motor_ch[motorNo].status = Motor_Stop;
        return;
    }
    
    motor_ch[motorNo].pulseCount.switchStop = MC_mm2pulse(motorNo,mm);
    
    if(motor_ch[motorNo].pulseCount.switchStop >= CONF_S_LEN)
    {//����˶����������ڼ������ߵ���, ��ִ�м�������
        
        motor_ch[motorNo].pulseCount.switchDec = motor_ch[motorNo].pulseCount.switchStop - CONF_S_LEN;
        
        switch(motorNo)
        {
            case 0:
                PWM1_pulseCount=0;
            break;
            case 1:
                PWM2_pulseCount=0;
            break;
            case 2:
                PWM3_pulseCount=0;
            break;
            case 3:
                PWM4_pulseCount=0;
            break;
        }
    
        motor_ch[motorNo].index = 100;
        
        if(motor_ch[motorNo].pulseCount.switchDec > 0)
        {
            motor_ch[motorNo].status = Motor_Run;
        }
        else
        {
            motor_ch[motorNo].status = Motor_decSpeed;
        }
        
    }
    else
    {
        MC_motorContinueMoveDistanceNoDec(motorNo,mm);
    }
}

/**
 ��������ָ�������޼��ٹ���
 @note ͨ������MC_motorRunForever֮��
*/
void MC_motorContinueMoveDistanceNoDec(uint8_t motorNo,uint16_t mm)
{   
    if(mm == 0)
    {
        pwmStop[motorNo]();
        motor_ch[motorNo].status = Motor_Stop;
        return;
    }
    
    motor_ch[motorNo].pulseCount.switchStop = MC_mm2pulse(motorNo,mm);
    
    switch(motorNo)
    {
        case 0:
            PWM1_pulseCount=0;
        break;
        case 1:
            PWM2_pulseCount=0;
        break;
        case 2:
            PWM3_pulseCount=0;
        break;
        case 3:
            PWM4_pulseCount=0;
        break;
    }
    
    motor_ch[motorNo].status = Motor_run2Stop;
}

/**
 ֱ������ָ������, �޼Ӽ��ٹ���
 @param -motorNo- ������
 @param -cw- ���з���
 @param -speed- �ٶ�
 @param -mm- ���о���
*/
void MC_motorMoveDistanceNoAcc(uint8_t motorNo,uint8_t cw,uint16_t mm)
{
    uint32_t pulseCount;
    uint16_t tempFreq;
    
    if(mm == 0)
    {
        pwmStop[motorNo]();
        motor_ch[motorNo].status = Motor_Stop;
        return;
    }
    
    MC_refreshParameter();
    
//    pwmStop[motorNo]();
    
    LV8731V_setCW(motorNo,cw);
    LV8731V_setMicroStep(motorNo,motor_ch[motorNo].ms);
    MC_refreshAmp(motorNo);
    
    pulseCount = MC_mm2pulse(motorNo,mm);

    motor_ch[motorNo].pulseCount.switchStop = pulseCount;
    
    tempFreq = CONF_S_TIMER_FREQ / motor_ch[motorNo].aimSpeed;

    motor_ch[motorNo].status = Motor_run2Stop;

    switch(motorNo)
    {
        case 0:
            TIM1->ARR = tempFreq;
            TIM1->CCR1 = TIM1->ARR >> 1;
            PWM1_pulseCount=0;
        break;
        case 1:
            TIM2->ARR = tempFreq;
            TIM2->CCR1 = TIM2->ARR >> 1;
            PWM2_pulseCount=0;
        break;
        case 2:
            TIM3->ARR = tempFreq;
            TIM3->CCR3 = TIM3->ARR >> 1;
            PWM3_pulseCount=0;
        break;
        case 3:
            TIM4->ARR = tempFreq;
            TIM4->CCR1 = TIM4->ARR >> 1;
            PWM4_pulseCount=0;
        break;
    }

    LV8731V_cmd(motorNo,1);
    pwmStart[motorNo]();
}

void MC_motorMovePulseCountWithoutCalculateARR(uint8_t motorNo, uint32_t pulseCount)
{

    pwmStop[motorNo]();

    motor_ch[motorNo].pulseCount.switchStop = pulseCount;

    switch(motorNo)
    {
        case 0:
            PWM1_pulseCount=0;
            TIM1->ARR = motor_ch[motorNo].arrTable[0];
            TIM1->CCR1 = TIM1->ARR >> 1;
        break;
        case 1:
            TIM2->ARR = motor_ch[motorNo].arrTable[0];
            TIM2->CCR1 = TIM2->ARR >> 1;
            PWM2_pulseCount=0;
        break;
        case 2:
            TIM3->ARR = motor_ch[motorNo].arrTable[0];
            TIM3->CCR3 = TIM3->ARR >> 1;
            PWM3_pulseCount=0;
        break;
        case 3:
            TIM4->ARR = motor_ch[motorNo].arrTable[0];
            TIM4->CCR1 = TIM4->ARR >> 1;
            PWM4_pulseCount=0;
        break;
    }

    motor_ch[motorNo].pulseCount.switchStop = pulseCount;

    LV8731V_cmd(motorNo,1);
    pwmStart[motorNo]();
}

/**
 * 电机移动指定脉冲数且不计算加减速曲线
 */
void MC_motorMovePulseWithoutCalculateARR(uint8_t motorNo,uint8_t cw,uint32_t pulseCount)
{
    MC_refreshParameter();
    LV8731V_setCW(motorNo,cw);
    LV8731V_setMicroStep(motorNo,motor_ch[motorNo].ms);
    MC_refreshAmp(motorNo);
    MC_motorMovePulseCountWithoutCalculateARR(motorNo,pulseCount);
}

/**
 * 电机运动指定距离且不计算加减速曲线
 */
void MC_motorMoveDistanceWithoutCalculateARR(uint8_t motorNo,uint8_t cw, uint32_t mm)
{
    uint32_t pulseCount;
    uint8_t pulseRatio;     ///< ��������ϸ����������Ĳ���.

    if(mm == 0)return;
    MC_refreshParameter();

    pulseRatio = calculatePulseRatio(motorNo);
    pulseCount = (mm / 100.0f) * motor_ch[motorNo].mm2pulse * pulseRatio;
    MC_motorMovePulseWithoutCalculateARR(motorNo,cw,pulseCount);
}


#ifdef CONF_S_TYPE_ACC

/**
 ���ɶ�ʱ��ARR��
*/
void MC_genArrTable(uint8_t motorNo,uint16_t aimFreq)
{
    uint16_t i;
    uint16_t factorMaxMin;
    
    if(aimFreq > CONF_S_SPEED_MIN)
    {
        factorMaxMin = aimFreq - CONF_S_SPEED_MIN;
    
        for(i=0;i<CONF_S_LEN;i++)
        {
            motor_ch[motorNo].arrTable[i] = CONF_S_TIMER_FREQ / (CONF_S_SPEED_MIN + factorMaxMin * expFactor[CONF_S_FACTOR_NO][i]);
        }
    }
    
    else
    {
        for(i=0;i<CONF_S_LEN;i++)
        {
            motor_ch[motorNo].arrTable[i] = CONF_S_TIMER_FREQ / aimFreq;
        }
    }
}

/**
 ʹ��S���ٵ���������
*/
void MC_motorMoveForever(uint8_t motorNo,uint8_t cw)
{    
    MC_refreshParameter();
    LV8731V_setCW(motorNo,cw);
    LV8731V_setMicroStep(motorNo,motor_ch[motorNo].ms);
    MC_refreshAmp(motorNo);
    
    MC_genArrTable(motorNo,motor_ch[motorNo].aimSpeed);
    
    switch(motorNo)
    {
        case 0:
            PWM1_pulseCount=0;
            TIM1->ARR = motor_ch[motorNo].arrTable[0];
            TIM1->CCR1 = TIM1->ARR >> 1;
        break;
        case 1:
            TIM2->ARR = motor_ch[motorNo].arrTable[0];
            TIM2->CCR1 = TIM2->ARR >> 1;
            PWM2_pulseCount=0;
        break;
        case 2:
            TIM3->ARR = motor_ch[motorNo].arrTable[0];
            TIM3->CCR3 = TIM3->ARR >> 1;
            PWM3_pulseCount=0;
        break;
        case 3:
            TIM4->ARR = motor_ch[motorNo].arrTable[0];
            TIM4->CCR1 = TIM4->ARR >> 1;
            PWM4_pulseCount=0;
        break;
    }

    motor_ch[motorNo].index = 0;
    motor_ch[motorNo].pulseCount.switchDec = 4294967295;
    motor_ch[motorNo].status = Motor_runForever;
    motor_ch[motorNo].pulseCountLimit = -1;
    
    LV8731V_cmd(motorNo,1);

    pwmStart[motorNo]();
}

/**
 �����������Ƶĳ�ʱ������.
 @note Ŀǰ�������ϵ����Ч.
*/
void MC_motorMoveForeverWithPulseCountLimited(uint8_t motorNo,uint8_t cw,int32_t pulseCountLimited)
{
   
    MC_refreshParameter();
    LV8731V_setCW(motorNo,cw);
    LV8731V_setMicroStep(motorNo,motor_ch[motorNo].ms);
    MC_refreshAmp(motorNo);
    
    MC_genArrTable(motorNo,motor_ch[motorNo].aimSpeed);
    
    switch(motorNo)
    {
        case 0:
            PWM1_pulseCount=0;
            TIM1->ARR = motor_ch[motorNo].arrTable[0];
            TIM1->CCR1 = TIM1->ARR >> 1;
        break;
        case 1:
            TIM2->ARR = motor_ch[motorNo].arrTable[0];
            TIM2->CCR1 = TIM2->ARR >> 1;
            PWM2_pulseCount=0;
        break;
        case 2:
            TIM3->ARR = motor_ch[motorNo].arrTable[0];
            TIM3->CCR3 = TIM3->ARR >> 1;
            PWM3_pulseCount=0;
        break;
        case 3:
            TIM4->ARR = motor_ch[motorNo].arrTable[0];
            TIM4->CCR1 = TIM4->ARR >> 1;
            PWM4_pulseCount=0;
        break;
    }

    motor_ch[motorNo].stopEvent = EVENT_none;
    motor_ch[motorNo].index = 0;
    motor_ch[motorNo].pulseCount.switchDec = 4294967295;
    motor_ch[motorNo].status = Motor_runForever;
    motor_ch[motorNo].pulseCountLimit = pulseCountLimited;
    
    LV8731V_cmd(motorNo,1);

    pwmStart[motorNo]();
}

#endif

/**
 ����˶�, ������Ӧ�ⲿ�ж�ʱ, ���Ͽ�ʼ��ƫ��, ֱ��ƫ������
*/
void MC_motorRunAndOffsetByEXTI(uint8_t motorNo,uint8_t cw,uint32_t offsetPulse,uint32_t limitPulse)
{
        
    MC_refreshParameter();
    LV8731V_setMicroStep(motorNo,motor_ch[motorNo].ms);
    MC_refreshAmp(motorNo);
    LV8731V_setCW(motorNo,cw);
    MC_genArrTable(motorNo,motor_ch[motorNo].aimSpeed);

//	LV8731V_setMicroStep(motorNo, 2);
//	motor_ch[motorNo].amp = 250;
//	MC_refreshAmp(motorNo);
//	LV8731V_setCW(motorNo, cw);
//	MC_genArrTable(motorNo, 4480);
    
    switch(motorNo)
    {
        case 0:
            PWM1_pulseCount=0;
            TIM1->ARR = motor_ch[motorNo].arrTable[0];
            TIM1->CCR1 = TIM1->ARR >> 1;
        break;
        case 1:
            TIM2->ARR = motor_ch[motorNo].arrTable[0];
            TIM2->CCR1 = TIM2->ARR >> 1;
            PWM2_pulseCount=0;
        break;
        case 2:
            TIM3->ARR = motor_ch[motorNo].arrTable[0];
            TIM3->CCR3 = TIM3->ARR >> 1;
            PWM3_pulseCount=0;
        break;
        case 3:
            TIM4->ARR = motor_ch[motorNo].arrTable[0];
            TIM4->CCR1 = TIM4->ARR >> 1;
            PWM4_pulseCount=0;
        break;
    }

    motor_ch[motorNo].stopEvent = EVENT_none;
    motor_ch[motorNo].index = 0;
    motor_ch[motorNo].status = Motor_runWithOffset;
    motor_ch[motorNo].pulseCountLimit = limitPulse;
    motor_ch[motorNo].beginRunOffset = 0;
    motor_ch[motorNo].offsetPulseCount = offsetPulse;
    
    LV8731V_cmd(motorNo,1);

    pwmStart[motorNo]();
}


//void TIM1_UP_IRQHandler(void)
void TIM1_CC_IRQHandler(void)
{
    
//    rt_interrupt_enter();
    
//    if((TIM1->SR & TIM_SR_UIF) == TIM_SR_UIF)
	if((TIM1->SR&TIM_SR_CC1IF) == TIM_SR_CC1IF)
    {
        PWM1_pulseCount ++;
#if 0
        if(PWM1_pulseCount > 100)
		{
			 if(!systemPara.isLackMaterral_NC)
			 {
					 if(IBIO_getInput(6) == 0)
					 {
						 if(++SsorLowCnt5 == 20)
						{
							SsorLowCnt5 = 0;
							systemPara.LackMaterral = 0;
						}
					 }
			 }
			 else
			 {
				 if(IBIO_getInput(6) == 1)
					 {
						 if(++SsorLowCnt5 == 20)
						{
							SsorLowCnt5 = 0;
							systemPara.LackMaterral = 0;
						}
					 }
			 }
		}
        if(systemPara.SLmode != 1)
        {//非回原点时，检测后限位
        	if(IBIO_getInput(2) == 0 )
			{
				if(++SsorLowCnt11 == 2)
				{
					SsorLowCnt11 = 0;
					systemPara.SLmode = 10;
					motor_ch[0].status = Motor_Stop;
				}
			}
        }
#endif

        if(systemPara.isFeedInPlaceEnable == 0 && systemPara.SLmode != 1 && IBIO_getInput(2) == 0)
        {
			if(++SsorLowCnt1 == 5)
			{
				motor_ch[0].status = Motor_Stop;
				GUI_mainMessageDisp("报警信息：触发限位！", 20);
				Alarm(0);
				return;
			}
        }
        if(systemPara.SLmode == 1)
        {
			if(IBIO_getInput(1) == 0 )
			{
				if(++SsorLowCnt1 == 5)
				{
					SsorLowCnt1 = 0;
					systemPara.SLmode = 0;
					motor_ch[0].beginRunOffset = 1;
					motor_ch[0].offsetPulseCount += PWM1_pulseCount;
				}
			}
        }
        else if(systemPara.SLmode == 2)
        {
			if(IBIO_getInput(1) == 1 )
			{
				if(++SsorLowCnt1 == 5)
				{
					SsorLowCnt1 = 0;
					systemPara.SLmode = 0;
					motor_ch[0].beginRunOffset = 1;
					motor_ch[0].offsetPulseCount += PWM1_pulseCount;
				}
			}
        }

        switch(motor_ch[0].status)
        {
            case Motor_Stop:
                break;
            case Motor_incSpeed:
                TIM1->ARR = motor_ch[0].arrTable[motor_ch[0].index];
                TIM1->CCR1 = TIM1->ARR >> 1;
                motor_ch[0].index ++;
                if(motor_ch[0].index > 99)
                {
                    motor_ch[0].status = Motor_Run;
                }
                break;
            case Motor_decSpeed:
            	motor_ch[0].index --;
                TIM1->ARR = motor_ch[0].arrTable[motor_ch[0].index];
                TIM1->CCR1 = TIM1->ARR >> 1;
                if(motor_ch[0].index < 1)
                {
                    motor_ch[0].index = 0;
                    motor_ch[0].status = Motor_Stop;
                    pwmStop[0]();
                    MC_AmpDecay(0);//���ֹͣ����˥��50%
//                    if(systemPara.RunStatus == 2 && systemPara.status == STATUS_ONLINE)
//					{
//						//收料异常报警
//						if(++SsorLowCnt5 == 5)
//						{
//							SsorLowCnt5 = 0;
////							if(systemPara.DOWNSLmode != 1)
////							{
////								LetMotorExceedPulseLimitAlert();
////							}
//							if(FLflag1 == 1)
//							{
//								LetMotorExceedPulseLimitAlert();
//							}
//						}
//
//					}
                }
                break;
            case Motor_Run:
                if(PWM1_pulseCount >= motor_ch[0].pulseCount.switchDec)
                {
                    motor_ch[0].status = Motor_decSpeed;
                }
                break;
            case Motor_runForever:
                if(motor_ch[0].index <= 99)
                {
                    TIM1->ARR = motor_ch[0].arrTable[motor_ch[0].index];
                    TIM1->CCR1 = TIM1->ARR >> 1;
                    motor_ch[0].index ++;
                }
                
                if((motor_ch[0].pulseCountLimit > 0) && (PWM1_pulseCount > motor_ch[0].pulseCountLimit))
                {
//                    GEVT_exceedPulseCount();
//                    EVT_givenMotorExceedPulseCount();
                    MC_motorStop(0);
//                    SIG_releaseGivenMotorStopSigFromExceedPulseCount();
                    
                }
                
                break;
                
            case Motor_run2Stop:
                if(PWM1_pulseCount >= motor_ch[0].pulseCount.switchStop)
                {
                    pwmStop[0]();
                    MC_AmpDecay(0);//���ֹͣ����˥��50%
                    motor_ch[0].status = Motor_Stop;
                }
                break;
                
            case Motor_runWithOffset:
                if(motor_ch[0].index <= 99)
                {
                    TIM1->ARR = motor_ch[0].arrTable[motor_ch[0].index];
                    TIM1->CCR1 = TIM1->ARR >> 1;
                    motor_ch[0].index ++;
                }
                
                if(motor_ch[0].beginRunOffset)
                {  
                    if(PWM1_pulseCount >= motor_ch[0].offsetPulseCount)
                    {
//                        MC_motorStop(0);
                    	motor_ch[0].status = Motor_decSpeed;//减速停止
                        motor_ch[0].beginRunOffset = 0;


                    }
                }
                
                if(((motor_ch[0].pulseCountLimit > 0) && (PWM1_pulseCount > motor_ch[0].pulseCountLimit)  && IBIO_getInput(1) == 1))
                {
                    motor_ch[0].stopEvent = EVENT_exceedPulseCountLimit;
                    MC_motorStop(0);

                }
                break;
        }
        
        TIM1->SR &= ~TIM_SR_CC1IF;
    }
    
//    rt_interrupt_leave();
}

void TIM2_IRQHandler(void)
{
    
    if((TIM2->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        PWM2_pulseCount ++;

        if(systemPara.FLmode == 1)
        {
			if(IBIO_getInput(10) == 0)
			{
				if(++SsorLowCnt2 == 5)
				{
					SsorLowCnt2 = 0;
					systemPara.FLmode = 0;
					motor_ch[1].status = Motor_decSpeed;
				}
			}
        }
        switch(motor_ch[1].status)
        {
            case Motor_Stop:
                break;
            case Motor_incSpeed:
                TIM2->ARR = motor_ch[1].arrTable[motor_ch[1].index];
                TIM2->CCR1 = TIM2->ARR >> 1;
                motor_ch[1].index ++;
                if(motor_ch[1].index > 99)
                {
                    motor_ch[1].status = Motor_Run;
                }
                break;
            case Motor_decSpeed:
            	 motor_ch[1].index --;
                TIM2->ARR = motor_ch[1].arrTable[motor_ch[1].index];
                TIM2->CCR1 = TIM2->ARR >> 1;
                if(motor_ch[1].index < 1)
                {
                    motor_ch[1].index = 0;
                    motor_ch[1].status = Motor_Stop;
                    pwmStop[1]();
                    MC_AmpDecay(1);//���ֹͣ����˥��50%

//                    if(systemPara.status == STATUS_ONLINE)
//                    {
//                    	// 放料自然停止时说明超过了放料限位.
//                    	LetMotorExceedPulseLimitAlert();
//                    }
                }
                break;
            case Motor_Run:
                if(PWM2_pulseCount >= motor_ch[1].pulseCount.switchDec)
                {
                    motor_ch[1].status = Motor_decSpeed;
                }
                break;
            case Motor_runForever:
                if(motor_ch[1].index <= 99)
                {
                    TIM2->ARR = motor_ch[1].arrTable[motor_ch[1].index];
                    TIM2->CCR1 = TIM2->ARR >> 1;
                    motor_ch[1].index ++;
                }
                
                if((motor_ch[1].pulseCountLimit > 0) && (PWM2_pulseCount > motor_ch[1].pulseCountLimit))
                {
//                    GEVT_exceedPulseCount();
//                    EVT_letMotorExceedPulseCount();
//                    SIG_releaseLetMotorStopSigFromExceedPulseCount();
                }
                
                break;
                
            case Motor_run2Stop:
                if(PWM2_pulseCount >= motor_ch[1].pulseCount.switchStop)
                {
                    pwmStop[1]();
                    MC_AmpDecay(1);//���ֹͣ����˥��50%
                    motor_ch[1].status = Motor_Stop;
                }
                break;
                
            case Motor_runWithOffset:
                break;
        }
        
        TIM2->SR &= ~TIM_SR_UIF;
    }
    

}

void TIM3_IRQHandler(void)
{
    
    if((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        PWM3_pulseCount ++;

//        if(systemPara.RunStatus == 4)//放松料带
//        {
//			if(IBIO_getInput(3) == 0)
//			{
//        		if(++SsorLowCnt3 == 5)
//				{
//        			SsorLowCnt3 = 0;
//
//					motor_ch[2].status = Motor_decSpeed;
//				}
//			}
//        }
//        else
//        {
//			if(systemPara.UPSLmode == 1)
//        	{
//	        	if(IBIO_getInput(4) == 0)
//				{
//	        		if(++SsorLowCnt3 == 5)
//					{
//	        			SsorLowCnt3 = 0;
//	        			systemPara.UPSLmode = 0;
//						motor_ch[2].status = Motor_decSpeed;
//					}
//				}
//			}
//        }
        if(systemPara.UPSLmode == 1)
		{
			if(IBIO_getInput(4) == 0 )
			{
				if(++SsorLowCnt3 == 5)
				{
					SsorLowCnt3 = 0;
					systemPara.UPSLmode = 0;
					motor_ch[2].status = Motor_decSpeed;
//					motor_ch[2].beginRunOffset = 1;
//					motor_ch[2].offsetPulseCount += PWM3_pulseCount;
				}
			}
		}
		else if(systemPara.UPSLmode == 2)
		{
			if(IBIO_getInput(4) == 1 )
			{
				if(++SsorLowCnt3 == 5)
				{
					SsorLowCnt3 = 0;
					systemPara.UPSLmode = 0;
					motor_ch[2].status = Motor_decSpeed;
//					motor_ch[2].beginRunOffset = 1;
//					motor_ch[2].offsetPulseCount += PWM3_pulseCount;
				}
			}
		}


        switch(motor_ch[2].status)
        {
            case Motor_Stop:
                break;
            case Motor_incSpeed:
                TIM3->ARR = motor_ch[2].arrTable[motor_ch[2].index];
                TIM3->CCR3 = TIM3->ARR >> 1;
                motor_ch[2].index ++;
                if(motor_ch[2].index > 99)
                {
                    motor_ch[2].status = Motor_Run;
                }
                break;
            case Motor_decSpeed:
            	 motor_ch[2].index --;
                TIM3->ARR = motor_ch[2].arrTable[motor_ch[2].index];
                TIM3->CCR3 = TIM3->ARR >> 1;
                if(motor_ch[2].index < 1)
                {
                    motor_ch[2].index = 0;
                    motor_ch[2].status = Motor_Stop;
                    pwmStop[2]();
                    MC_AmpDecay(2);//���ֹͣ����˥��50%
                }
                break;
            case Motor_Run:
                if(PWM3_pulseCount >= motor_ch[2].pulseCount.switchDec)
                {
                    motor_ch[2].status = Motor_decSpeed;
                }
                break;
            case Motor_runForever:
                if(motor_ch[2].index <= 99)
                {
                    TIM3->ARR = motor_ch[2].arrTable[motor_ch[2].index];
                    TIM3->CCR3 = TIM3->ARR >> 1;
                    motor_ch[2].index ++;
                }
                
                break;
                
            case Motor_run2Stop:
                if(PWM3_pulseCount >= motor_ch[2].pulseCount.switchStop)
                {
                    pwmStop[2]();
                    MC_AmpDecay(2);//���ֹͣ����˥��50%
                    motor_ch[2].status = Motor_Stop;
                }
                break;
                
            case Motor_runWithOffset:
				if (motor_ch[2].index <= 99)
				{
					TIM3->ARR = motor_ch[2].arrTable[motor_ch[2].index];
					TIM3->CCR3 = TIM3->ARR >> 1;
					motor_ch[2].index++;
				}

				if (motor_ch[2].beginRunOffset)
				{
					if (PWM3_pulseCount >= motor_ch[2].offsetPulseCount)
					{
						//MC_motorStop(BO_MOTOR);
						motor_ch[2].status = Motor_decSpeed;
						motor_ch[2].beginRunOffset = 0;
					}
				}
			break;
        }
        
        TIM3->SR &= ~TIM_SR_UIF;
    }
}

void TIM4_IRQHandler(void)
{
    if((TIM4->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        PWM4_pulseCount ++;

        if(systemPara.UPSLmode == 1)
		{
			if(IBIO_getInput(8) == 0)
			{
				if(++SsorLowCnt4 == 5)
				{
					SsorLowCnt4 = 0;
					systemPara.UPSLmode = 0;
					motor_ch[3].status = Motor_decSpeed;
				}
			}
		}
        else if(systemPara.UPSLmode == 2)
		{
			if(IBIO_getInput(7) == 0)
			{
				if(++SsorLowCnt4 == 5)
				{
					SsorLowCnt4 = 0;
					systemPara.UPSLmode = 0;
					motor_ch[3].status = Motor_decSpeed;
				}
			}
		}
        switch(motor_ch[3].status)
        {
            case Motor_Stop:
                break;
            case Motor_incSpeed:
                TIM4->ARR = motor_ch[3].arrTable[motor_ch[3].index];
                TIM4->CCR1 = TIM4->ARR >> 1;
                motor_ch[3].index ++;
                if(motor_ch[3].index > 99)
                {
                    motor_ch[3].status = Motor_Run;
                }
                break;
            case Motor_decSpeed:
            	 motor_ch[3].index --;
                TIM4->ARR = motor_ch[3].arrTable[motor_ch[3].index];
                TIM4->CCR1 = TIM4->ARR >> 1;
                if(motor_ch[3].index < 1)
                {
                    motor_ch[3].index = 0;
//                    motor_ch[3].status = Motor_Stop;
//                    pwmStop[3]();
                    MC_motorStop(3);
                    MC_AmpDecay(3);//���ֹͣ����˥��50%

//                    if(systemPara.status != STATUS_FREERUN)
//                    {
//                    	ShouMotorExceedPulseLimitAlert();
//                    }

                }
                break;
            case Motor_Run:
                if(PWM4_pulseCount >= motor_ch[3].pulseCount.switchDec)
                {
                    motor_ch[3].status = Motor_decSpeed;
                }
                break;
            case Motor_runForever:
                if(motor_ch[3].index <= 99)
                {
                    TIM4->ARR = motor_ch[3].arrTable[motor_ch[3].index];
                    TIM4->CCR1 = TIM4->ARR >> 1;
                    motor_ch[3].index ++;
                }
                
                break;
                
            case Motor_run2Stop:
                if(PWM4_pulseCount >= motor_ch[3].pulseCount.switchStop)
                {
                    pwmStop[3]();
                    MC_AmpDecay(3);//���ֹͣ����˥��50%
                    motor_ch[3].status = Motor_Stop;
                }
                break;
                
            case Motor_runWithOffset:
                break;
        }
        
        TIM4->SR &= ~TIM_SR_UIF;
    }
}

