/**
 @file
*/
#include "motorctrl/motorctrl.h"
#include "pwm1/pwm1.h"
#include "pwm2/pwm2.h"
#include "pwm3/pwm3.h"
#include "pwm4/pwm4.h"
#include "lv8731v/lv8731v.h"
#include "DAC/dac.h"
#include "MdriverAlarmIO/mdriveralarmio.h"
#include "paramanager/paramanager.h"
#include "eventHandler/eventHandler.h"
#include "motorctrl/s_func_factor.h"
#include "userfunc/userfunc.h"
#include "dwin/gui.h"
#include "inboardio/inboardio.h"
#include "exio/exio.h"

typedef void (*PWMCmd)(void);
typedef void (*SetFrequent)(uint32_t );

static const PWMCmd pwmStop[] = {PWM1_stop,PWM2_stop,PWM3_stop,PWM4_stop};
static const PWMCmd pwmStart[] = {PWM1_start,PWM2_start,PWM3_start,PWM4_start};
MotorPara motor_ch[4] = {0};

#define TIME_CONST_K        0.002f

uint8_t MOTOR_offsetOrigin;

void MC_motorMovePulseCount(uint8_t motorNo, uint32_t pulseCount);
//static void MC_refreshParameter(void);
static uint8_t calculatePulseRatio(uint8_t motorNo);
void MC_refreshAmp(uint8_t motorNo);
void MC_genArrTable(uint8_t motorNo,uint32_t aimFreq);
//static void MC_motorMovePulseCountWithoutIncDec(uint8_t motorNo, uint32_t pulseCount);
void MC_motorMovePulseWithoutCalculateARR(uint8_t motorNo,uint8_t cw,uint32_t pulseCount);
void MC_motorMovePulseCountWithoutCalculateARR(uint8_t motorNo, uint32_t pulseCount);
void MC_refreshSpeed(uint8_t motorNo, uint32_t speed);
void MC_refreshEXMParameter(uint8_t motorNo);

/**
 @brief 电机初始化
	LV8731V, PWM1, PWM2, PWM3, PWM4, TIM5.
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

    // 初始化DAC模块
    MX_DAC_Init();
    // 配置MDAIO
    MDAIO_config();
    
    motor_ch[0].mm2pulse = motor_ch[0].amp;
    motor_ch[1].mm2pulse = LM_MM_PULSE;
    motor_ch[2].mm2pulse = motor_ch[2].amp;
    motor_ch[3].mm2pulse = SM_MM_PULSE;
    
//    MC_AmpDecay(0);
    MC_AmpDecay(1);
//    MC_AmpDecay(2);
    MC_AmpDecay(3);
    
    MC_motorStop(0);
    MC_motorStop(1);
    MC_motorStop(2);
    MC_motorStop(3);
}

// 电机移动固定距离
void MC_motorMoveDistance(uint8_t motorNo, uint32_t speed, uint8_t cw, uint32_t mm)
{
    uint32_t pulseCount=0;
    uint8_t pulseRatio;

    if(mm == 0)return;

    MC_refreshSpeed(motorNo, speed);

    if(motorNo == 1 || motorNo == 3)
    {
		MC_refreshEXMParameter(motorNo);
    }
    LV8731V_setCW(motorNo,cw);

    pulseRatio = calculatePulseRatio(motorNo);

    pulseCount = mm  * motor_ch[motorNo].mm2pulse * pulseRatio;

    MC_genArrTable(motorNo,motor_ch[motorNo].aimSpeed);

    MC_motorMovePulse(motorNo,cw,pulseCount);
}

// 电机移动脉冲
void MC_motorMovePulse(uint8_t motorNo,uint8_t cw,uint32_t pulseCount)
{
    LV8731V_setCW(motorNo,cw);
    LV8731V_setMicroStep(motorNo,motor_ch[motorNo].ms);
    MC_refreshAmp(motorNo);
    MC_motorMovePulseCount(motorNo,pulseCount);
}

// 电机移动脉冲数
void MC_motorMovePulseCount(uint8_t motorNo, uint32_t pulseCount)
{
    uint16_t tempFreq;
    uint32_t Dec_pulseCount;

    if(motorNo == 0)
    {
    	Dec_pulseCount = CONF_S_LEN / CONF_S_OFFSET_GIVEN;
    }
    else if(motorNo == 1)
    {
    	Dec_pulseCount = CONF_S_LEN / CONF_S_OFFSET_LET;
    }
    else if(motorNo == 2)
    {
    	Dec_pulseCount = CONF_S_LEN / CONF_S_OFFSET_BO;
    }
    else if(motorNo == 3)
    {
    	Dec_pulseCount = CONF_S_LEN / CONF_S_OFFSET_SHOU;
    }

	if((pulseCount <= Dec_pulseCount * 2)){
		Dec_pulseCount = (pulseCount / 2) - 2;//当减速脉冲小于最小减速脉冲时会多两个脉冲
	}

	motor_ch[motorNo].pulseCount.switchStop = pulseCount;

	LV8731V_cmd(motorNo,1);

	tempFreq = motor_ch[motorNo].arrTable[0];

	motor_ch[motorNo].pulseCount.switchRun = Dec_pulseCount;
	motor_ch[motorNo].pulseCount.switchDec = motor_ch[motorNo].pulseCount.switchStop - motor_ch[motorNo].pulseCount.switchRun;

	motor_ch[motorNo].status = Motor_incSpeed;
	motor_ch[motorNo].index = 0;


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

    pwmStart[motorNo]();
}

void MC_cmd(uint8_t motorNo, uint8_t cmd)
{
    LV8731V_cmd(motorNo,cmd);
}

/**
 @brief 停止锟斤拷锟叫碉拷锟??
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
/**
 * 使用4通道的目标速度生成各自加速表
 */
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

void MC_motorStop(uint8_t motorNo)
{
    pwmStop[motorNo]();
    motor_ch[motorNo].status = Motor_Stop;
    MC_AmpDecay(motorNo);
}

// 电机减速停止
void MC_motorStopDec(uint8_t motorNo)
{
	motor_ch[motorNo].status = Motor_decSpeed;
}

//更新电机微步
void MC_refreshMicroStep(uint8_t motorNo)
{
    LV8731V_setMicroStep(motorNo,motor_ch[motorNo].ms);
}

// 更新电机电流值
void MC_refreshAmp(uint8_t motorNo)
{
    float V;

    V = motor_ch[motorNo].amp * 0.011f; // cur * 0.01 * 1.1
    if(V > 3.25f)
    {
        V = 3.25f;
    }
//    MCP4728_setOutput(motorNo,(uint16_t)(V / 0.0008));
    DacSetValue(motorNo, V);
}

void MC_AmpDecay(uint8_t motorNo)
{
    float V;

    V = (motor_ch[motorNo].amp * 0.5f) * 0.011f; // cur * 0.01 * 1.1
    if(V > 3.25f)
    {
        V = 3.25f;
    }
//    MCP4728_setOutput(motorNo,(uint16_t)(V / 0.0008));
    DacSetValue(motorNo, V);
}

// 更新电机速度
void MC_refreshSpeed(uint8_t motorNo, uint32_t speed)
{
	if(motorNo == 2)
		motor_ch[motorNo].aimSpeed = 1.5f * speed * motor_ch[motorNo].mm2pulse * calculatePulseRatio(motorNo);
	else
		motor_ch[motorNo].aimSpeed = speed * motor_ch[motorNo].mm2pulse * calculatePulseRatio(motorNo);
}

// 更新电机所有参数
void MC_refreshAllParameter(void)
{
	//读取电机的细分设定
	motor_ch[0].ms = PARA_readParameter(0x500b);
	motor_ch[1].ms = PARA_readParameter(0x500d);
	motor_ch[2].ms = PARA_readParameter(0x500F);
	motor_ch[3].ms = PARA_readParameter(0x5011);
	//读取电机的电流设定
	motor_ch[0].amp = PARA_readParameter(0x500a);
	motor_ch[1].amp = PARA_readParameter(0x500C);
	motor_ch[2].amp = PARA_readParameter(0x500E);
	motor_ch[3].amp = PARA_readParameter(0x5010);

	//motor_ch[0].mm2pulse = GM_MM_PULSE;
	motor_ch[0].mm2pulse = motor_ch[0].amp;
	motor_ch[1].mm2pulse = LM_MM_PULSE;
//    motor_ch[2].mm2pulse = BM_MM_PULSE;
	motor_ch[2].mm2pulse = motor_ch[2].amp;
	motor_ch[3].mm2pulse = SM_MM_PULSE;

	//读取电机各项参数数据
	controlPara.givenSpeed = PARA_readParameter(0x5000) * 0.01f;
	controlPara.givenDistance = PARA_readParameter(0x5001) * 0.1f;
	controlPara.givenDistancePulseCount = MC_mm2pulse(0, controlPara.givenDistance);
	controlPara.givenLowSpeed = PARA_readParameter(0x5017) * 0.01f;
	controlPara.givenLowDistance = PARA_readParameter(0x5016) * 0.01f;
	controlPara.givenLowDistancePulseCount = MC_mm2pulse(0, controlPara.givenLowDistance);
	controlPara.givenOffset = PARA_readParameter(0x5012) * 0.01f;
	controlPara.givenOffsetPulseCount = MC_mm2pulse(0, controlPara.givenOffset);

	controlPara.letSpeed = PARA_readParameter(0x5002) * 0.01f;
	controlPara.letDistance = PARA_readParameter(0x5003) * 0.1f;

	controlPara.boSpeed = PARA_readParameter(0x5004) * 0.01f;
	controlPara.boDistance = PARA_readParameter(0x5005) * 0.01f;
	controlPara.boLowSpeed = PARA_readParameter(0x5019) * 0.01f;
	controlPara.boaRstSpeed = PARA_readParameter(0x5015) * 0.01f;
	controlPara.boDistance2 = PARA_readParameter(0x5013) * 0.01f;
	controlPara.boLowDistance = PARA_readParameter(0x5018) * 0.01f;

	controlPara.shouSpeed = PARA_readParameter(0x5006) * 0.01f;
	controlPara.shouDistance = PARA_readParameter(0x5007) * 0.1f;

	controlPara.oriSpeed = PARA_readParameter(0x5008) * 0.01f;
	controlPara.oriOffset = PARA_readParameter(0x5009) * 0.01f;
	controlPara.oriOffsetPulseCount = MC_mm2pulse(2, controlPara.oriOffset);

	controlPara.HLDistance = PARA_readParameter(0x5014) * 0.01f;
}

// 更新电机 微步，电流值
void MC_refreshEXMParameter(uint8_t motorNo)
{
	LV8731V_setMicroStep(motorNo,motor_ch[motorNo].ms);
	MC_refreshAmp(motorNo);
}

void MC_5000toPara(uint8_t addr)
{
    switch(addr)
    {
        case 0xa:
            motor_ch[0].amp = PARA_readParameter(addr + 0x5000);
            MC_refreshAmp(0);
            break;
        case 0xb:
            motor_ch[0].ms = PARA_readParameter(addr + 0x5000);
            MC_refreshMicroStep(0);
            break;
        case 0x0c:
            motor_ch[1].amp = PARA_readParameter(addr + 0x5000);
            MC_refreshAmp(1);
            break;
        case 0x0d:
            motor_ch[1].ms = PARA_readParameter(addr + 0x5000);
            MC_refreshMicroStep(1);
            break;
        case 0x0e:
            motor_ch[2].amp = PARA_readParameter(addr + 0x5000);
            MC_refreshAmp(2);
            break;
        case 0x0f:
            motor_ch[2].ms = PARA_readParameter(addr + 0x5000);
            MC_refreshMicroStep(2);
            break;
        case 0x10:
            motor_ch[3].amp = PARA_readParameter(addr + 0x5000);
            MC_refreshAmp(3);
            break;
        case 0x11:
            motor_ch[3].ms = PARA_readParameter(addr + 0x5000);
            MC_refreshMicroStep(3);
            break;
    }
}

static uint8_t calculatePulseRatio(uint8_t motorNo)
{
    uint8_t pulseRatio;
    
    switch(motor_ch[motorNo].ms)
    {
        case 1:pulseRatio = 1;break;
        case 2:pulseRatio = 2;break;
        case 3:pulseRatio = 4;break;
        case 4:
        	if(motorNo == 0 || motorNo == 2 )
        		pulseRatio = 8;
        	else
        		pulseRatio = 16;
        	break;
        default:pulseRatio=1;break;
    }
    
    return pulseRatio;
}

void MC_GMOffsetRun(void)
{
	if(EmergencyStop() == 1)
		return;

    if(!IBIO_getOutput(1))
	{
//		GUI_mainMessageDisp("压料气缸缩回.", 13);
		YL_Action(1);
	}
	if(IBIO_getOutput(2))
	{
//		GUI_mainMessageDisp("夹料气缸缩回.", 13);
		JL_Action(1);
	}

    if(motor_ch[0].status == Motor_Stop)
    	MC_motorMoveForever(0,CONF_GM_JOGSPEED,systemPara.MotorGivenDir);
}

void MC_BMOffsetRun(void)
{
	if(EmergencyStop() == 1)
		return;

	if(systemPara.status == STATUS_FREERUN && systemPara.Initstatus == 1)
	{
		if(systemPara.Language == 0)
			POP_UP_INFO_AGING_RUN();
		else if(systemPara.Language == 1)
			EN_POP_UP_INFO_AGING_RUN();
		return;
	}

	if(systemPara.status != STATUS_AUTO && systemPara.Page == 0)
	{
		if(systemPara.status ==  STATUS_ONLINE )
		{
			if(systemPara.Language == 0)
				CANT_SWITCH_PAGE_MESSAGE1();
			else if(systemPara.Language == 1)
				EN_CANT_SWITCH_PAGE_MESSAGE1();
			return;
		}
	}

    if(systemPara.Initstatus == 1)
    {
		GUI_runStatusDisp(0);
		systemPara.Initstatus = 0;
		systemPara.isWorkTaskRun = 0;
    }
    if(!IBIO_getOutput(1))
	{
//		GUI_mainMessageDisp("压料气缸缩回.", 13);
		YL_Action(1);
	}
	if(IBIO_getOutput(2))
	{
//		GUI_mainMessageDisp("夹料气缸缩回.", 13);
		JL_Action(1);
	}

    if(motor_ch[2].status == Motor_Stop)
    	MC_motorMoveForever(2,CONF_BM_JOGSPEED, systemPara.MotorBoDir);
}

void MC_GMOffsetRunDistance(uint8_t dir, uint32_t Distance)
{
	if(EmergencyStop() == 1)
		return;

    if(motor_ch[0].status == Motor_Stop)
    	MC_motorMoveDistance(0,CONF_GM_JOGSPEED,dir,Distance);
}

void MC_BMOffsetRunDistance(uint8_t dir, uint32_t Distance)
{
	if(EmergencyStop() == 1)
		return;

    if(motor_ch[2].status == Motor_Stop)
    	MC_motorMoveDistance(2,CONF_BM_JOGSPEED,dir,Distance);
}
// mm转换成脉冲数
uint32_t MC_mm2pulse(uint8_t motorNo,float mm)
{
    uint8_t pulseRatio;
    uint32_t pulseCount;
    
    if(mm == 0)return 0;
    
    pulseRatio = calculatePulseRatio(motorNo);
    pulseCount = mm  * motor_ch[motorNo].mm2pulse * pulseRatio;
    
    return pulseCount;
}

/**
 * mm转换成脉冲数
 @param -motorNo- 电机序号1
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4
 @param -mm- 电机移动的距离. 单位mm. 2位小数
*/
uint32_t MC_mm2pulse1(uint8_t motorNo,float mm)
{
    uint8_t pulseRatio;
    uint32_t pulseCount;

    if(mm == 0)return 0;

    pulseRatio = calculatePulseRatio(motorNo);
    pulseCount = (mm * 0.01)  * motor_ch[motorNo].mm2pulse * pulseRatio;

    return pulseCount;
}

#ifdef CONF_S_TYPE_ACC

// 根据目标速度获取加速表
void MC_genArrTable(uint8_t motorNo,uint32_t aimFreq)
{
    uint16_t i;
    uint32_t factorMaxMin;
    
    if(aimFreq > CONF_S_SPEED_MIN)
    {
        factorMaxMin = aimFreq - CONF_S_SPEED_MIN;

        for(i=0;i<CONF_S_LEN - 1;i++)
        {
#if CONF_S_100
            motor_ch[motorNo].arrTable[i] = CONF_S_TIMER_FREQ / (CONF_S_SPEED_MIN + factorMaxMin * expFactor[CONF_S_FACTOR_NO][i]);
#else
        	motor_ch[motorNo].arrTable[i] = CONF_S_TIMER_FREQ / (CONF_S_SPEED_MIN + factorMaxMin * expFactor_1000[i]);
#endif
        }
    }
    else
    {
        for(i=0;i<CONF_S_LEN -1 ;i++)
        {
            motor_ch[motorNo].arrTable[i] = CONF_S_TIMER_FREQ / aimFreq;
        }
    }
}

// 电机一直转动
void MC_motorMoveForever(uint8_t motorNo, uint32_t speed, uint8_t cw)
{    
	MC_refreshSpeed(motorNo, speed);

    if(motorNo == 1 || motorNo == 3)
    {
		MC_refreshEXMParameter(motorNo);
    }
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
#endif

void MC_motorRunAndOffsetByEXTI(uint8_t motorNo, uint32_t speed, uint8_t cw,uint32_t offsetPulse,uint32_t limitPulse)
{
	uint32_t Dec_pulseCount;
	MC_refreshSpeed(motorNo, speed);

    LV8731V_setCW(motorNo,cw);
	LV8731V_cmd(motorNo,1);
    MC_genArrTable(motorNo,motor_ch[motorNo].aimSpeed);

	if(motorNo == 1 || motorNo == 3)
    {
		MC_refreshEXMParameter(motorNo);
    }
	if(motorNo == 0)
	{
		Dec_pulseCount = CONF_S_LEN / CONF_S_OFFSET_GIVEN;
	}
	else if(motorNo == 1)
	{
		Dec_pulseCount = CONF_S_LEN / CONF_S_OFFSET_LET;
	}
	else if(motorNo == 2)
	{
		Dec_pulseCount = CONF_S_LEN / CONF_S_OFFSET_BO;
	}
	else if(motorNo == 3)
	{
		Dec_pulseCount = CONF_S_LEN / CONF_S_OFFSET_SHOU;
	}

	if(limitPulse <= Dec_pulseCount){
		Dec_pulseCount = limitPulse / 2;
	}

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

	motor_ch[motorNo].pulseCount.switchRun = Dec_pulseCount;
	motor_ch[motorNo].pulseCount.switchDec = motor_ch[motorNo].pulseCountLimit - motor_ch[motorNo].pulseCount.switchRun;
    motor_ch[motorNo].beginRunOffset = 0;
    motor_ch[motorNo].offsetPulseCount = offsetPulse;
    
    pwmStart[motorNo]();
}


void TIM1_CC_IRQHandler(void)
{
	static uint8_t SsorLowCnt = 0,SsorLowCnt1 = 0,SsorLowCnt2 = 0,Estopcount = 0;

	if((TIM1->SR&TIM_SR_CC1IF) == TIM_SR_CC1IF)
    {
        PWM1_pulseCount ++;

        if(systemPara.isInEmergencyStopEnable && ++Estopcount >= 5 && IBIO_getInput(12))
        {
        	MC_motorStopAllEmergency();
        }

		if(PWM1_pulseCount > 100)
		{//缺料预警检测
#if 0
			 if(!systemPara.isLackMaterral_NC)
			 {
				if((++SsorLowCnt >= 20) && (!IBIO_getInput(4)))
				{
					SsorLowCnt = 0;
					systemPara.LackMaterral = 0;
				}
			 }
			 else
			{
				 if((++SsorLowCnt >= 20) && (IBIO_getInput(4)))
				 {
					SsorLowCnt = 0;
					systemPara.LackMaterral = 0;
				 }
			}
#else
			 if(systemPara.isLackMaterral_NC)
			 {//开启缺料检测，未检测到物料进行输出缺料信号
				if((++SsorLowCnt >= 20) && (IBIO_getInput(4)))
				{
					SsorLowCnt = 0;
					systemPara.LackMaterral = 0;
				}
			 }
#endif
		}

        if(systemPara.SLmode == 1 || systemPara.SLmode == 10)
        {//物料定位检测
#if CONF_Fiber_VER == 0
			if((++SsorLowCnt1 >= 2) && (systemPara.sensorChosen == 1) && (!IBIO_getInput(2)))//下光纤
#else
        	if((++SsorLowCnt1 >= 2) && (systemPara.sensorChosen == 1) && ((!IBIO_getInput(2)) ||(!IBIO_getInput(3))))//下光纤
#endif
			{//下光纤
				SsorLowCnt1 = 0;
				systemPara.SLmode = 0;
				motor_ch[0].beginRunOffset = 1;
				motor_ch[0].status = Motor_runWithOffset;
				motor_ch[0].stopEvent = EVENT_none;
			}

			else if(systemPara.sensorChosen == 2)
			{//上光纤
				if(systemPara.ChkSensorLevel)
				{//检测有料，检测高电平
					if((++SsorLowCnt1 >= 50) && IBIO_getInput(3))
					{
						SsorLowCnt1 = 0;
						systemPara.ChkSensorLevel = 0;
						return;
					}
					else
					{
						if(SsorLowCnt1 > 1)
							SsorLowCnt1 --;
					}
				}
				else if((++SsorLowCnt1 >= 5) && (!IBIO_getInput(3)))
				{//检测有料，检测低电平
					SsorLowCnt1 = 0;
					systemPara.SLmode = 0;
					motor_ch[0].beginRunOffset = 1;
					//减速过程中检测到物料，重新切换到偏移模式
					motor_ch[0].status = Motor_runWithOffset;
					motor_ch[0].stopEvent = EVENT_none;
					//motor_ch[0].offsetPulseCount += PWM1_pulseCount;
				}
			}
        }
        else if(systemPara.SLmode == 2)
        {//检测感应器减速停
        	if((++SsorLowCnt1 >= 5) && (!IBIO_getInput(3)))
			{
				SsorLowCnt1 = 0;
				systemPara.SLmode = 0;
				systemPara.LowSpeedTrigger = 1;
				motor_ch[0].status = Motor_decSpeed;
			}
#if CONF_Fiber_VER == 0
			if((++SsorLowCnt2 >= 5) && (!IBIO_getInput(2)))
#else
			if((++SsorLowCnt2 >= 5) && ((!IBIO_getInput(2)) || (!IBIO_getInput(3))))
#endif
			{//如果先检测到物料到位传感器
				SsorLowCnt2 = 0;
				systemPara.SLmode = 0;
				systemPara.LowSpeedTrigger = 2;
				motor_ch[0].beginRunOffset = 1;
			}
        }

        switch(motor_ch[0].status)
        {
            case Motor_Stop:
                break;
            case Motor_incSpeed:
                if(motor_ch[0].index < (motor_ch[0].pulseCount.switchRun * CONF_S_OFFSET_GIVEN))//CONF_S_LEN)
                {
                    TIM1->ARR = motor_ch[0].arrTable[motor_ch[0].index];
					TIM1->CCR1 = TIM1->ARR >> 1;
                	motor_ch[0].index += CONF_S_OFFSET_GIVEN;
                }
                else
                {
                	motor_ch[0].status = Motor_Run;
                }
                break;
            case Motor_decSpeed:
            	if(PWM1_pulseCount < motor_ch[0].pulseCount.switchStop)
            	{
					if(motor_ch[0].index > CONF_S_OFFSET_GIVEN)
					{
						if(motor_ch[0].index > CONF_S_LEN)
							motor_ch[0].index = CONF_S_LEN;
						motor_ch[0].index -= CONF_S_OFFSET_GIVEN;
						TIM1->ARR = motor_ch[0].arrTable[motor_ch[0].index];
						TIM1->CCR1 = TIM1->ARR >> 1;
					}
            	}
            	else
            	{
					MC_motorStop(0);
            		motor_ch[0].index = 0;
            	}
                break;
            case Motor_Run:
                if(PWM1_pulseCount > motor_ch[0].pulseCount.switchDec)
                {
                    motor_ch[0].status = Motor_decSpeed;
                }
                break;
            case Motor_runForever:
            	if(motor_ch[0].index < CONF_S_LEN - CONF_S_OFFSET_GIVEN)
                {
                    TIM1->ARR = motor_ch[0].arrTable[motor_ch[0].index];
                    TIM1->CCR1 = TIM1->ARR >> 1;
                    motor_ch[0].index += CONF_S_OFFSET_GIVEN;
                }
                break;
            case Motor_run2Stop:
                if(PWM1_pulseCount >= motor_ch[0].pulseCount.switchStop)
                {
					MC_motorStop(0);
                }
                break;
            case Motor_runWithOffset:
            	if(motor_ch[0].index < ((motor_ch[0].pulseCount.switchRun * CONF_S_OFFSET_GIVEN)))//CONF_S_LEN)
                {
                    TIM1->ARR = motor_ch[0].arrTable[motor_ch[0].index];
                    TIM1->CCR1 = TIM1->ARR >> 1;
                    motor_ch[0].index += CONF_S_OFFSET_GIVEN;
                }

            	if(motor_ch[0].beginRunOffset)
                {  
					motor_ch[0].beginRunOffset = 0;
					//----------------------------------------------------
					if(motor_ch[0].offsetPulseCount >= CONF_S_LEN/CONF_S_OFFSET_GIVEN)
					{//补偿值大于或等于减速脉冲时，把补偿值加入减速脉冲中，加20个固定减速脉冲
						motor_ch[0].pulseCount.switchStop = (motor_ch[0].offsetPulseCount + 20) + PWM1_pulseCount;
						motor_ch[0].pulseCount.switchRun = CONF_S_LEN / CONF_S_OFFSET_GIVEN;;
						motor_ch[0].pulseCount.switchDec = motor_ch[0].pulseCount.switchStop - motor_ch[0].pulseCount.switchRun;
						motor_ch[0].status = Motor_Run;
					}
					else
					{//小于减速脉冲时，直接减速
						motor_ch[0].pulseCount.switchStop = (motor_ch[0].offsetPulseCount + 20) + PWM1_pulseCount;
//						motor_ch[0].pulseCount.switchRun = CONF_S_LEN / CONF_S_OFFSET_GIVEN;;
//						motor_ch[0].pulseCount.switchDec = motor_ch[0].pulseCount.switchStop - motor_ch[0].pulseCount.switchRun;
//						motor_ch[0].pulseCount.switchDec = (motor_ch[0].offsetPulseCount + 20) * CONF_S_OFFSET_GIVEN;
//	            		if(motor_ch[0].pulseCount.switchDec > CONF_S_LEN)
//							motor_ch[0].index = CONF_S_LEN;
//	            		else
//	            			motor_ch[0].index = motor_ch[0].pulseCount.switchDec;

						motor_ch[0].status = Motor_decSpeed;
					}
                }
                else if((motor_ch[0].pulseCountLimit > 0) && (PWM1_pulseCount >= motor_ch[0].pulseCount.switchDec))
                {
					motor_ch[0].stopEvent = EVENT_exceedPulseCountLimit;
					motor_ch[0].status = Motor_decSpeed;//减速停止
                }
                break;
        }
        
        TIM1->SR &= ~TIM_SR_CC1IF;
    }
}

void TIM2_IRQHandler(void)
{
	static uint8_t SsorLowCnt = 0, Estopcount = 0;

    if((TIM2->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        PWM2_pulseCount ++;

        if(systemPara.isInEmergencyStopEnable && ++Estopcount >= 5 && IBIO_getInput(12))
        {
        	MC_motorStopAllEmergency();
        }

        if(systemPara.FLmode == 1 && (++SsorLowCnt >= 5) && (!IBIO_getInput(8)))
        {
			SsorLowCnt = 0;
			systemPara.FLmode = 0;
			motor_ch[1].status = Motor_decSpeed;
        }
        switch(motor_ch[1].status)
		{
			case Motor_Stop:
				break;
			case Motor_incSpeed:
				if(motor_ch[1].index < ((motor_ch[1].pulseCount.switchRun * CONF_S_OFFSET_LET )))//CONF_S_LEN)
				{
					TIM2->ARR = motor_ch[1].arrTable[motor_ch[1].index];
					TIM2->CCR1 = TIM2->ARR >> 1;
					motor_ch[1].index += CONF_S_OFFSET_LET;
				}
				else
				{
					motor_ch[1].status = Motor_Run;
				}
				break;
			case Motor_decSpeed:
				if(motor_ch[1].index > CONF_S_OFFSET_LET)
				{
					motor_ch[1].index -= CONF_S_OFFSET_LET;
					TIM2->ARR = motor_ch[1].arrTable[motor_ch[1].index];
					TIM2->CCR1 = TIM2->ARR >> 1;
				}
				else
				{
					MC_motorStop(1);
					motor_ch[1].index = 0;
				}
				break;
			case Motor_Run:
				if(PWM2_pulseCount > motor_ch[1].pulseCount.switchDec)
				{
					motor_ch[1].status = Motor_decSpeed;
				}
				break;
			case Motor_runForever:
				if(motor_ch[1].index < CONF_S_LEN - CONF_S_OFFSET_LET)
				{
					TIM2->ARR = motor_ch[1].arrTable[motor_ch[1].index];
					TIM2->CCR1 = TIM2->ARR >> 1;
					motor_ch[1].index += CONF_S_OFFSET_LET;
				}
				break;
			case Motor_run2Stop:
				if(PWM2_pulseCount >= motor_ch[1].pulseCount.switchStop)
				{
					MC_motorStop(1);
				}
				break;

			case Motor_runWithOffset:
				if(motor_ch[1].index < ((motor_ch[1].pulseCount.switchRun * CONF_S_OFFSET_LET)))//CONF_S_LEN)
				{
					TIM2->ARR = motor_ch[1].arrTable[motor_ch[1].index];
					TIM2->CCR1 = TIM2->ARR >> 1;
					motor_ch[1].index += CONF_S_OFFSET_LET;
				}

				if(motor_ch[1].beginRunOffset && (PWM2_pulseCount >= motor_ch[1].offsetPulseCount))
				{
					if(motor_ch[2].index < CONF_S_LEN - CONF_S_OFFSET_LET)
					{//防止距离小，加速未完成时出现距离走少问题
						motor_ch[2].index = CONF_S_LEN - CONF_S_OFFSET_LET;
					}
                    motor_ch[1].status = Motor_decSpeed;//减速停止
					motor_ch[1].beginRunOffset = 0;
				}
				else if((motor_ch[1].pulseCountLimit > 0) && (PWM2_pulseCount >= motor_ch[1].pulseCountLimit))
				{
					motor_ch[1].stopEvent = EVENT_exceedPulseCountLimit;
					motor_ch[1].status = Motor_decSpeed;//减速停止
				}
				break;
		}
        TIM2->SR &= ~TIM_SR_UIF;
    }
}

void TIM3_IRQHandler(void)
{
	static uint8_t SsorLowCnt = 0, SsorLowCnt1 = 0, Estopcount = 0;

    if((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        PWM3_pulseCount ++;

        if(systemPara.isInEmergencyStopEnable && ++Estopcount >= 5 && IBIO_getInput(12))
        {
        	MC_motorStopAllEmergency();
        }

        if((systemPara.RSTmode == 1) && (++SsorLowCnt >= 5) && (!IBIO_getInput(1)))
        {//检测原点
			SsorLowCnt = 0;
			systemPara.RSTmode = 0;
			motor_ch[2].beginRunOffset = 1;
        }
        else if(systemPara.RSTmode == 2 && (++SsorLowCnt1 >= 5) && (IBIO_getInput(1)))
		{//避开原点2
        	SsorLowCnt1 = 0;
			systemPara.RSTmode = 0;
			motor_ch[2].status = Motor_decSpeed;
		}

        switch(motor_ch[2].status)
		{
			case Motor_Stop:
				break;

			case Motor_incSpeed:
				if(motor_ch[2].index < (motor_ch[2].pulseCount.switchRun * CONF_S_OFFSET_BO))
				{
					TIM3->ARR = motor_ch[2].arrTable[motor_ch[2].index];
					TIM3->CCR3 = TIM3->ARR >> 1;
					motor_ch[2].index += CONF_S_OFFSET_BO;
				}
				else
				{
					motor_ch[2].status = Motor_Run;
				}
				break;
			case Motor_decSpeed:
				if(motor_ch[2].index > CONF_S_OFFSET_BO)
				{
					motor_ch[2].index -= CONF_S_OFFSET_BO;
					TIM3->ARR = motor_ch[2].arrTable[motor_ch[2].index];
					TIM3->CCR3 = TIM3->ARR >> 1;
				}
				else
				{
					MC_motorStop(2);
					motor_ch[2].index = 0;
				}
				break;
			case Motor_Run:
				if(PWM3_pulseCount >= motor_ch[2].pulseCount.switchDec)
				{
					motor_ch[2].status = Motor_decSpeed;
				}
				break;
			case Motor_runForever:
				if(motor_ch[2].index >= CONF_S_LEN - CONF_S_OFFSET_BO)
				{
					TIM3->ARR = motor_ch[2].arrTable[motor_ch[2].index];
					TIM3->CCR3 = TIM3->ARR >> 1;
					motor_ch[2].index += CONF_S_OFFSET_BO;
				}
				break;
			case Motor_run2Stop:
				if(PWM3_pulseCount >= motor_ch[2].pulseCount.switchStop)
				{
					MC_motorStop(2);
				}
				break;
			case Motor_runWithOffset:
				if(motor_ch[2].index < ((motor_ch[2].pulseCount.switchRun * CONF_S_OFFSET_BO)))//CONF_S_LEN)
				{
					TIM3->ARR = motor_ch[2].arrTable[motor_ch[2].index];
					TIM3->CCR3 = TIM3->ARR >> 1;
					motor_ch[2].index += CONF_S_OFFSET_BO;
				}
            	if(motor_ch[2].beginRunOffset)
                {
					motor_ch[2].beginRunOffset = 0;
					//----------------------------------------------------

					motor_ch[2].pulseCount.switchDec = (motor_ch[2].offsetPulseCount) + PWM3_pulseCount;
					if(motor_ch[2].index < CONF_S_LEN - CONF_S_OFFSET_BO)
					{//防止距离小，加速未完成时出现距离走少问题
						motor_ch[2].index = CONF_S_LEN - CONF_S_OFFSET_BO;
					}
					motor_ch[2].status = Motor_Run;

                }
				else if((motor_ch[2].pulseCountLimit > 0) && (PWM3_pulseCount >= motor_ch[2].pulseCount.switchDec))
				{
					motor_ch[2].stopEvent = EVENT_exceedPulseCountLimit;
					motor_ch[2].status = Motor_decSpeed;
				}

			break;
		}
        TIM3->SR &= ~TIM_SR_UIF;
    }
}

void TIM4_IRQHandler(void)
{
	static uint8_t SsorLowCnt = 0, Estopcount = 0;

    if((TIM4->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        PWM4_pulseCount ++;

        if(systemPara.isInEmergencyStopEnable && ++Estopcount >= 5 && IBIO_getInput(12))
        {
        	MC_motorStopAllEmergency();
        }

        if(systemPara.UPSLmode == 1 && (++SsorLowCnt >= 5) && (!IBIO_getInput(6)))
        {
        	SsorLowCnt = 0;
			systemPara.UPSLmode = 0;
			motor_ch[3].status = Motor_decSpeed;
        }

        switch(motor_ch[3].status)
		{
			case Motor_Stop:
				break;
			case Motor_incSpeed:
				if(motor_ch[3].index < (motor_ch[3].pulseCount.switchRun * CONF_S_OFFSET_SHOU))
				{
					TIM4->ARR = motor_ch[3].arrTable[motor_ch[3].index];
					TIM4->CCR1 = TIM4->ARR >> 1;
					motor_ch[3].index += CONF_S_OFFSET_SHOU;
				}
				else
				{
					motor_ch[3].status = Motor_Run;
				}
				break;
			case Motor_decSpeed:
				if(motor_ch[3].index > CONF_S_OFFSET_SHOU)
				{
					motor_ch[3].index -= CONF_S_OFFSET_SHOU;
					TIM4->ARR = motor_ch[3].arrTable[motor_ch[3].index];
					TIM4->CCR1 = TIM4->ARR >> 1;
				}
				else
				{
					MC_motorStop(3);
					motor_ch[3].index = 0;
				}
				break;
			case Motor_Run:
				if(PWM4_pulseCount >= motor_ch[3].pulseCount.switchDec)
				{
					motor_ch[3].status = Motor_decSpeed;
				}
				break;
			case Motor_runForever:
				if(motor_ch[3].index < CONF_S_LEN - CONF_S_OFFSET_SHOU)
				{
					TIM4->ARR = motor_ch[3].arrTable[motor_ch[3].index];
					TIM4->CCR1 = TIM4->ARR >> 1;
					motor_ch[3].index += CONF_S_OFFSET_SHOU;
				}
				break;
			case Motor_run2Stop:
				if(PWM4_pulseCount >= motor_ch[3].pulseCount.switchStop)
				{
					MC_motorStop(3);
				}
				break;

			case Motor_runWithOffset:
				if(motor_ch[3].index < (motor_ch[3].pulseCount.switchRun * CONF_S_OFFSET_SHOU))//CONF_S_LEN)
				{
					TIM4->ARR = motor_ch[3].arrTable[motor_ch[3].index];
					TIM4->CCR1 = TIM4->ARR >> 1;
					motor_ch[3].index+= CONF_S_OFFSET_SHOU;
				}

				if (motor_ch[3].beginRunOffset && (PWM4_pulseCount >= motor_ch[3].offsetPulseCount))
				{
					if(motor_ch[2].index < CONF_S_LEN - CONF_S_OFFSET_SHOU)
					{//防止距离小，加速未完成时出现距离走少问题
						motor_ch[2].index = CONF_S_LEN - CONF_S_OFFSET_SHOU;
					}
					motor_ch[3].status = Motor_decSpeed;
					motor_ch[3].beginRunOffset = 0;
				}
				else if((motor_ch[3].pulseCountLimit > 0) && (PWM4_pulseCount >= motor_ch[3].pulseCount.switchDec))
				{
					motor_ch[3].stopEvent = EVENT_exceedPulseCountLimit;
					motor_ch[3].status = Motor_decSpeed;
				}
			break;
		}
        TIM4->SR &= ~TIM_SR_UIF;
    }
}

