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
#include "paramanager/paramanager.h"
#include "eventHandler/eventHandler.h"
#include "motorctrl/s_func_factor.h"
#include "userfunc/userfunc.h"
#include "dwin/gui.h"
#include "inboardio/inboardio.h"
#include "exio/exio.h"
#include "paraManager/paraManager.h"
#include "easyModbus/easyModbus.h"
#include "MdriverAlarmIO/mdriveralarmio.h"

typedef void (*PWMCmd)(void);
typedef void (*SetFrequent)(uint32_t );

static const PWMCmd pwmStop[] = {PWM1_stop,PWM2_stop,PWM3_stop,PWM4_stop};
static const PWMCmd pwmStart[] = {PWM1_start,PWM2_start,PWM3_start,PWM4_start};
//static const SetFrequent setFrequent[] = {PWM1_setFrequent,PWM2_setFrequent,PWM3_setFrequent,PWM4_setFrequent};
MotorPara motor_ch[4] = {0};

// й╠О©╫ДЁёО©╫О©╫, О©╫цЁО©╫О©╫О©╫О©╫О©╫ж╣с╕О©╫О©╫TIM5О©╫п╤О©╫й╠О©╫О©╫р╩О©╫О©╫, О©╫О©╫О©╫О©╫ц©О©╫н╦д╠О©╫О©╫ы╤х╣О©╫й╠О©╫О©╫р╩О©╫О©╫
#define TIME_CONST_K        0.002f

uint8_t SsorLowCnt1 = 0,SsorLowCnt2 = 0,SsorLowCnt3 = 0,SsorLowCnt4 = 0,SsorLowCnt5 = 0;
uint8_t SsorLowCnt11 = 0;
/**
 т╜О©╫О©╫ф╚О©╫ф╥О©╫О©╫О©╫. 1-О©╫О©╫ф╚О©╫О©╫; 2-О©╫О©╫ф╚О©╫О©╫
*/
uint8_t MOTOR_offsetOrigin;

void MC_motorMovePulseCount(uint8_t motorNo, uint32_t pulseCount);
static void MC_refreshParameter(void);
static uint8_t calculatePulseRatio(uint8_t motorNo);
void MC_refreshAmp(uint8_t motorNo);
void MC_genArrTable(uint8_t motorNo,uint16_t aimFreq);
static void MC_motorMovePulseCountWithoutIncDec(uint8_t motorNo, uint32_t pulseCount);


/**
 @brief Г■╣Ф°╨Е┬²Е╖▀Е▄?
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

    // Е┬²Е╖▀Е▄√DACФ╗║Е²≈
    MX_DAC_Init();
    // И┘█Г╫╝MDAIO
    MDAIO_config();

//    motor_ch[GIVEN_MOTOR].mm2pulse = GM_MM_PULSE;
    motor_ch[LET_MOTOR].mm2pulse = LM_MM_PULSE;
    motor_ch[GIVEN_MOTOR].mm2pulse = motor_ch[GIVEN_MOTOR].amp;
    motor_ch[UP_MOTOR].mm2pulse = UP_MM_PULSE;
    motor_ch[DOWN_MOTOR].mm2pulse = DOWN_MM_PULSE;
    
    //й╧О©╫О©╫4м╗О©╫О©╫О©╫О©╫О©?
//    MC_cmd(0,1);
//    MC_cmd(1,1);
//    MC_cmd(2,1);
//    MC_cmd(3,1);
    
    // О©╫О©╫О©╫О©╫к╔О©╫О©╫
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
 @brief ж╦О©╫О©╫О©╫О©╫О©╫О©╫к╤О©╫ж╦О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©?
 @param -motorNo- ж╦О©╫О©╫О©╫О©╫О©?
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4 
 @param -cw- О©╫О©╫О©╫п╥О©╫О©╫О©╫
    @arg -0- кЁй╠О©╫К╥╫О©╫О©╫
    @arg -1- О©╫О©╫й╠О©╫К╥╫О©╫О©╫
 @param -pulseCount- О©╫О©╫О©╫О©╫О©╫О©╫
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
 @brief ж╦О©╫О©╫О©╫О©╫О©╫О©╫к╤О©╫ж╦О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©?
 @param -motorNo- ж╦О©╫О©╫О©╫О©╫О©?
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4 
 @param -pulseCount- О©╫О©╫О©╫О©╫О©╫О©╫
*/
void MC_motorMovePulseCount(uint8_t motorNo, uint32_t pulseCount)
{
    uint16_t tempFreq;
    uint32_t mm_pulseCount;
    uint32_t Dec_pulseCount;
    uint8_t pulseRatio;

    motor_ch[motorNo].pulseCount.switchStop = pulseCount;
    pulseRatio = calculatePulseRatio(motorNo);
    mm_pulseCount = (motor_ch[motorNo].mm2pulse * pulseRatio);

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

	if((pulseCount < Dec_pulseCount * 2)){
		Dec_pulseCount = (pulseCount / 2) - 2;
	}

	MC_genArrTable(motorNo,motor_ch[motorNo].aimSpeed);
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
    
    motor_ch[motorNo].pulseCount.switchStop = pulseCount;

    LV8731V_cmd(motorNo,1);
    pwmStart[motorNo]();
}

/**
 * Г╖╩Е┼╗Ф▄┤Е╝ Х└┴Е├╡Ф∙?.
 */
static void MC_motorMovePulseCountWithoutIncDec(uint8_t motorNo, uint32_t pulseCount)
{
    uint16_t tempFreq;

    pwmStop[motorNo]();

    motor_ch[motorNo].pulseCount.switchStop = pulseCount;

    motor_ch[motorNo].status = Motor_run2Stop;
#if CONF_S_100
    tempFreq = CONF_S_TIMER_FREQ / (CONF_S_SPEED_MIN + (motor_ch[motorNo].aimSpeed - CONF_S_SPEED_MIN) * expFactor[CONF_S_FACTOR_NO][CONF_S_LEN]);
#else
    tempFreq = CONF_S_TIMER_FREQ / (CONF_S_SPEED_MIN + (motor_ch[motorNo].aimSpeed - CONF_S_SPEED_MIN) * expFactor_1000[CONF_S_LEN]);
#endif

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
 @brief О©╫О©╫О©╫О©╫к╤О©╫ж╦О©╫О©╫О©╫О©╫О©╫О©?
 @param -motorNo- ж╦О©╫О©╫О©╫О©╫О©?
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4 
 @param -cw- О©╫О©╫О©╫О©╫к╤О©╫О©╫О©╫О©╫О©?
    @arg -0- кЁй╠О©╫О©╫
    @arg -1- О©╫О©╫й╠О©╫О©╫
 @param -mm- О©╫к╤О©╫О©╫О©╫О©╫О©╫, О©╫О©╫н╩mm. 2н╩О©╫О©╫О©╫О©╫п║О©╫О©╫, О©╫О©╫100О©╫Ю╣╠О©╫О©╫1mm
 @note й╧О©╫ц╦ц╥О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©? GM_MM_PULSE О©╫х╨О©╫О©╫О©╫п╤О©╫О©╫О©?.
*/ 
void MC_motorMoveDistance(uint8_t motorNo,uint8_t cw, uint32_t mm)
{
    uint32_t pulseCount;
    uint8_t pulseRatio;     ///< О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫о╦О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫д╡О©╫О©╫О©?.
    
    if(mm == 0)return;
    MC_refreshParameter();
    
    pulseRatio = calculatePulseRatio(motorNo);
    pulseCount = (mm / 100.0f) * motor_ch[motorNo].mm2pulse * pulseRatio;
    MC_motorMovePulse(motorNo,cw,pulseCount);
}

/**
 * Г╖╩Е┼╗Х└┴Е├╡Ф∙╟Д╦█Е╦╕Е┼═Е┤▐И??
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
 * Г■╣Ф°╨Г╖╩Е┼╗Ф▄┤Е╝ Х╥²Г╕╩Д╦■Д╦█Е╦╕Е┼═Е┤▐И??
 */
void MC_motorMoveDistanceWithoutIncDec(uint8_t motorNo,uint8_t cw, uint32_t mm)
{
    uint32_t pulseCount;
    uint8_t pulseRatio;     ///< О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫о╦О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫д╡О©╫О©╫О©?.

    if(mm == 0)return;
    MC_refreshParameter();

    pulseRatio = calculatePulseRatio(motorNo);
    pulseCount = (mm / 100.0f) * motor_ch[motorNo].mm2pulse * pulseRatio;
    MC_motorMovePulseWithoutIncDec(motorNo,cw,pulseCount);
}

/**
 * Д╫©Г■╗4И─ И│⌠Г └Г⌡╝Ф═┤И?÷Е╨╕Г■÷Ф┬░Е░└Х┤╙Е┼═И?÷Х║╗
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
#if CONF_S_100
				motor_ch[motorNo].arrTable[i] = CONF_S_TIMER_FREQ / (CONF_S_SPEED_MIN + factorMaxMin * expFactor[CONF_S_FACTOR_NO][i]);
#else
				motor_ch[motorNo].arrTable[i] = CONF_S_TIMER_FREQ / (CONF_S_SPEED_MIN + factorMaxMin * expFactor_1000[i]);
#endif
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
 @brief й╧О©╫э╣О©╫О©?
 @param -motorNo- ж╦О©╫О©╫О©╫О©╫О©?
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4
 @param -cmd- й╧О©╫О©╫ж╦О©╫О©╫
    @arg 0- й╖О©╫О©╫
    @arg 1- й╧О©╫О©╫
*/
void MC_cmd(uint8_t motorNo, uint8_t cmd)
{
    LV8731V_cmd(motorNo,cmd);
}

/**
 @brief мёж╧О©╫О©╫О©╫п╣О©╫О©?
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
 * Г■╗Д╨▌Г╢╖Ф?╔Е│°Ф╜╒Г └Е│°Ф╜╒Ф┴?Ф°┴Г■╣Ф°?, Д╪ Е┘ЁИ≈╜И? И│⌠Д╫©Х┐╫.
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
 @brief мёж╧ж╦О©╫О©╫О©╫О©╫О©?
 @param -motorNo- ж╦О©╫О©╫О©╫О©╫О©?
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
 * Е╦╕Е┤▐И─÷Е│°Ф╜╒Г■╣Ф°?
 */
void MC_motorStopDec(uint8_t motorNo)
{
	motor_ch[motorNo].status = Motor_decSpeed;
}


/**
 @brief О©╫О©╫О©╫О©╫о╦О©╫О©╫
 @param -motorNo- ж╦О©╫О©╫О©╫О©╫О©?
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4
 @param -msCode- о╦О©╫О©╫
    @arg Full_Step- х╚О©╫О©╫
    @arg Half_Step- 2о╦О©╫О©╫
    @arg Quarter_Step- 4о╦О©╫О©╫
    @arg MS_16- 16о╦О©╫О©╫
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
    DacSetValue(motorNo, V);
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

    DacSetValue(motorNo, V);
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
 @brief О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫ф╡О©╫О©╫О©╫О©╫О©?0x5000О©╫О©╫ж╥О©╫О©╫motor_ch
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
            motor_ch[UP_MOTOR].amp = PARA_readParameter(addr + 0x5000);
            MC_refreshAmp(UP_MOTOR);
            break;
        case 0x0f:
            motor_ch[UP_MOTOR].ms = PARA_readParameter(addr + 0x5000);
            MC_refreshMicroStep(UP_MOTOR);
            break;
        case 0x10:
            motor_ch[DOWN_MOTOR].amp = PARA_readParameter(addr + 0x5000);
            MC_refreshAmp(DOWN_MOTOR);
            break;
        case 0x11:
            motor_ch[DOWN_MOTOR].ms = PARA_readParameter(addr + 0x5000);
            MC_refreshMicroStep(DOWN_MOTOR);
            break;
    }
}

/**
 @brief О©╫О©╫О©╫б╣О©╫О©╫О©╫О©╫О©╫п╡О©╫О©╫О©?
*/
static void MC_refreshParameter(void)
{
    // О©╫О©╫О©╫О©╫о╦О©╫О©╫
    motor_ch[GIVEN_MOTOR].ms = PARA_readParameter(0x500b);
    motor_ch[LET_MOTOR].ms = PARA_readParameter(0x500d);
    motor_ch[UP_MOTOR].ms = PARA_readParameter(0x500F);
    motor_ch[DOWN_MOTOR].ms = PARA_readParameter(0x5011);


    // О©╫О©╫О©╫б╣О©╫О©╫О©╫
    motor_ch[GIVEN_MOTOR].amp = PARA_readParameter(0x500a);
    motor_ch[LET_MOTOR].amp = PARA_readParameter(0x500C);
    motor_ch[UP_MOTOR].amp = PARA_readParameter(0x500E);
    motor_ch[DOWN_MOTOR].amp = PARA_readParameter(0x5010);

    motor_ch[GIVEN_MOTOR].mm2pulse = motor_ch[GIVEN_MOTOR].amp;
    // О©╫О©╫О©╫О©╫д©О©╫О©╫О©╫ы╤О©╫
    // motor_ch[motorNo].mm2pulse * PARA_readParameter(0x5000) * 0.01;
    if(systemPara.doGohome == 1)//ЁУй╪╩╞╩ьт╜╣Цкы╤х
    	motor_ch[GIVEN_MOTOR].aimSpeed = 1.2f * PARA_readParameter(0x5008) * motor_ch[GIVEN_MOTOR].mm2pulse * 0.01f * calculatePulseRatio(GIVEN_MOTOR);
    else
    	motor_ch[GIVEN_MOTOR].aimSpeed = 1.2f * PARA_readParameter(0x5000) * motor_ch[GIVEN_MOTOR].mm2pulse * 0.01f * calculatePulseRatio(GIVEN_MOTOR);

    motor_ch[LET_MOTOR].aimSpeed = PARA_readParameter(0x5002) * motor_ch[LET_MOTOR].mm2pulse * 0.01f * calculatePulseRatio(LET_MOTOR);
    motor_ch[UP_MOTOR].aimSpeed =  PARA_readParameter(0x5006) * motor_ch[BO_MOTOR].mm2pulse * 0.01f * calculatePulseRatio(BO_MOTOR);
    motor_ch[DOWN_MOTOR].aimSpeed = PARA_readParameter(0x5004) * motor_ch[SHOU_MOTOR].mm2pulse * 0.01f * calculatePulseRatio(SHOU_MOTOR);
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
 О©╫О©╫О©╫О©╫О©╫О©╫О©╫о╣О©╫О©╫ф╚О©╫О©╫setО©╫О©╫О©╫О©╫
*/
void MC_GMOffsetRun(void)
{
    uint8_t speed_h,speed_l;
    
    speed_h = ADDR_5000_H[0];
    speed_l = ADDR_5000_L[0];
    
    ADDR_5000_H[0] = CONF_GM_OFFSET_SETSPEED >> 8;
    ADDR_5000_L[0] = CONF_GM_OFFSET_SETSPEED & 0xff;
    
    MC_motorMoveForever(0,systemPara.MotorGivenDir);
    
    ADDR_5000_H[0] = speed_h;
    ADDR_5000_L[0] = speed_l;
}

/**
 О©╫О©╫О©╫О©╫в╙О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫
 @param -motorNo- ж╦О©╫О©╫О©╫О©╫О©?
    @arg -0- CH1
    @arg -1- CH2
    @arg -2- CH3
    @arg -3- CH4
 @param -mm- О©╫О©╫О©╫О©╫. О©╫О©╫н╩mm. 2н╩О©╫О©╫О©╫О©╫п║О©╫О©╫
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
 О©╫О©╫MC_motorMoveForeverж╝О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫pulseCountО©╫О©╫О©╫О©╫О©╫О©╫.
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
 О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫к╤О©╫ж╦О©╫О©╫О©╫О©╫О©╫О©?
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
    {//О©╫О©╫О©╫О©╫к╤О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫з╪О©╫О©╫О©╫О©╫О©╫О©╫ъ╣О©╫О©╫О©?, О©╫О©╫ж╢О©╫п╪О©╫О©╫О©╫О©╫О©╫О©╫О©╫
        
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
 О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫ж╦О©╫О©╫О©╫О©╫О©╫О©╫О©╫ч╪О©╫О©╫ы╧О©╫О©╫О©╫
 @note м╗О©╫О©╫О©╫О©╫О©╫О©╫MC_motorRunForeverж╝О©╫О©╫
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
 ж╠О©╫О©╫О©╫О©╫О©╫О©╫ж╦О©╫О©╫О©╫О©╫О©╫О©╫, О©╫ч╪с╪О©╫О©╫ы╧О©╫О©╫О©╫
 @param -motorNo- О©╫О©╫О©╫О©╫О©╫О©╫
 @param -cw- О©╫О©╫О©╫п╥О©╫О©╫О©╫
 @param -speed- О©╫ы╤О©╫
 @param -mm- О©╫О©╫О©╫п╬О©╫О©╫О©╫
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
 * Г■╣Ф°╨Г╖╩Е┼╗Ф▄┤Е╝ Х└┴Е├╡Ф∙╟Д╦■Д╦█Х╝║Г╝≈Е┼═Е┤▐И?÷Ф⌡╡Г╨?
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
 * Г■╣Ф°╨Х©░Е┼╗Ф▄┤Е╝ Х╥²Г╕╩Д╦■Д╦█Х╝║Г╝≈Е┼═Е┤▐И─÷Ф⌡╡Г╨?
 */
void MC_motorMoveDistanceWithoutCalculateARR(uint8_t motorNo,uint8_t cw, uint32_t mm)
{
    uint32_t pulseCount;
    uint8_t pulseRatio;     ///< О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫о╦О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫д╡О©╫О©╫О©?.

    if(mm == 0)return;
    MC_refreshParameter();

    pulseRatio = calculatePulseRatio(motorNo);
    pulseCount = (mm / 100.0f) * motor_ch[motorNo].mm2pulse * pulseRatio;
    MC_motorMovePulseWithoutCalculateARR(motorNo,cw,pulseCount);
}


#ifdef CONF_S_TYPE_ACC

/**
 О©╫О©╫О©╫и╤О©╫й╠О©╫О©╫ARRО©╫О©╫
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
#if CONF_S_100
            motor_ch[motorNo].arrTable[i] = CONF_S_TIMER_FREQ / (CONF_S_SPEED_MIN + factorMaxMin * expFactor[CONF_S_FACTOR_NO][i]);
#else
        	motor_ch[motorNo].arrTable[i] = CONF_S_TIMER_FREQ / (CONF_S_SPEED_MIN + factorMaxMin * expFactor_1000[i]);
#endif
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
 й╧О©╫О©╫SО©╫О©╫О©╫ы╣О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫
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
 О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫О©╫ф╣дЁО©╫й╠О©╫О©╫О©╫О©╫О©╫О©╫.
 @note д©г╟О©╫О©╫О©╫О©╫О©╫О©╫О©╫о╣О©╫О©╫О©╫О©╫п?.
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
 О©╫О©╫О©╫О©╫к╤О©?, О©╫О©╫О©╫О©╫О©╫О©╫с╕О©╫Б╡©О©╫п╤О©╫й╠, О©╫О©╫О©╫о©О©╫й╪О©╫О©╫ф╚О©╫О©╫, ж╠О©╫О©╫ф╚О©╫О©╫О©╫О©╫О©╫О©╫
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


        if(systemPara.RunStatus == 1 || systemPara.RunStatus == 11)
        {
        	if(systemPara.RSTAlarm == 0 && systemPara.SLmode == 1 && IBIO_getInput(3) == 0)
			{
				if(++SsorLowCnt11 >= 5)
				{
					SsorLowCnt11 = 0;
					systemPara.SLmode = 0;
					systemPara.RSTAlarm = 1;
					motor_ch[0].status = Motor_decSpeed;
				}
			}
        	if((systemPara.RSTAlarm == 1 || systemPara.SLmode == 2 )&& IBIO_getInput(2) == 0)
			{
				if(++SsorLowCnt11 >= 5)
				{
					SsorLowCnt11 = 0;
					motor_ch[0].status = Motor_Stop;
					GUI_mainMessageDisp("╠╗╬╞пео╒ё╨лЫ╠Й╣Г╩З╢╔╥╒срочн╩ё║", 30);
					systemPara.RSTAlarm  = 1;
					Alarm(0);
					return;
				}
			}
        }
        else
        {
			if((systemPara.SLmode == 0  || systemPara.SLmode == 4 ) && IBIO_getInput(3) == 0)
			{
				if(++SsorLowCnt11 >= 5)
				{
					SsorLowCnt11 = 0;
					motor_ch[0].status = Motor_Stop;
					GUI_mainMessageDisp("╠╗╬╞пео╒ё╨лЫ╠Й╣Г╩З╢╔╥╒вСочн╩ё║", 30);
					systemPara.RSTAlarm  = 1;
					Alarm(0);
					return;
				}
			}
			if((systemPara.SLmode == 1 || systemPara.SLmode == 3) && IBIO_getInput(2) == 0)
			{
				if(++SsorLowCnt11 >= 5)
				{
					SsorLowCnt11 = 0;
					motor_ch[0].status = Motor_Stop;
					GUI_mainMessageDisp("╠╗╬╞пео╒ё╨лЫ╠Й╣Г╩З╢╔╥╒срочн╩ё║", 30);
					systemPara.RSTAlarm  = 1;
					Alarm(0);
					return;
				}
			}
        }
        if(systemPara.SLmode == 1)
        {
        	if(IBIO_getInput(1) == 0)
			{
        		if(++SsorLowCnt1 >= 5)
				{
        			SsorLowCnt1 = 0;
					systemPara.SLmode = 0;
					if(systemPara.RSTAlarm == 0 || systemPara.Motor_orientation == 0)
					{
						motor_ch[0].beginRunOffset = 1;
						motor_ch[0].offsetPulseCount += PWM1_pulseCount;
						return;
					}
					else
					{
						motor_ch[0].status = Motor_decSpeed;
					}
				}
			}
        }
        else if(systemPara.SLmode == 2)
		{
			if(IBIO_getInput(1) == 1)
			{
				if(++SsorLowCnt1 >= 5)
				{
					SsorLowCnt1 = 0;
					systemPara.SLmode = 0;
					motor_ch[0].status = Motor_decSpeed;
				}
			}
		}

        switch(motor_ch[0].status)
        {
            case Motor_Stop:
                break;
            case Motor_incSpeed:
            	if(PWM1_pulseCount >= motor_ch[0].pulseCount.switchRun)
            	{
            		 motor_ch[0].status = Motor_Run;
            		 return;
            	}
                if(motor_ch[0].index < CONF_S_LEN)
                {
                	TIM1->ARR = motor_ch[0].arrTable[motor_ch[0].index];
					TIM1->CCR1 = TIM1->ARR >> 1;
					motor_ch[0].index += CONF_S_OFFSET_GIVEN;
                }
                break;
            case Motor_decSpeed:
                if(motor_ch[0].index < CONF_S_OFFSET_GIVEN)
                {
                    motor_ch[0].index = 0;
                    MC_motorStop(0);
                    return;
                }
            	motor_ch[0].index -= CONF_S_OFFSET_GIVEN;
                TIM1->ARR = motor_ch[0].arrTable[motor_ch[0].index];
                TIM1->CCR1 = TIM1->ARR >> 1;
                break;
            case Motor_Run:
                if(PWM1_pulseCount >= motor_ch[0].pulseCount.switchDec)
                {
                    motor_ch[0].status = Motor_decSpeed;
                }
                break;
            case Motor_runForever:
                if(motor_ch[0].index < CONF_S_LEN)
                {
                    TIM1->ARR = motor_ch[0].arrTable[motor_ch[0].index];
                    TIM1->CCR1 = TIM1->ARR >> 1;
                    motor_ch[0].index += CONF_S_OFFSET_GIVEN;
                }

                if((motor_ch[0].pulseCountLimit > 0) && (PWM1_pulseCount > motor_ch[0].pulseCountLimit))
                {
                	 motor_ch[0].status = Motor_decSpeed;
                }
                break;
            case Motor_run2Stop:
                if(PWM1_pulseCount >= motor_ch[0].pulseCount.switchStop)
                {
                	MC_motorStop(0);
                }
                break;
                
            case Motor_runWithOffset:
                if(motor_ch[0].index < CONF_S_LEN)
                {
                    TIM1->ARR = motor_ch[0].arrTable[motor_ch[0].index];
                    TIM1->CCR1 = TIM1->ARR >> 1;
                    motor_ch[0].index += CONF_S_OFFSET_GIVEN;
                }

                if(motor_ch[0].beginRunOffset)
                {  
                    if(PWM1_pulseCount >= motor_ch[0].offsetPulseCount)
                    {
//                    	MC_motorStopDec(0);
                        motor_ch[0].status = Motor_decSpeed;//Е┤▐И?÷Е│°Ф╜?
                        motor_ch[0].beginRunOffset = 0;
                    }
                }
                else
                {
					if((motor_ch[0].pulseCountLimit > 0) && (PWM1_pulseCount > motor_ch[0].pulseCountLimit))
					{
						motor_ch[0].stopEvent = EVENT_exceedPulseCountLimit;
	//                    MC_motorStop(0);
						motor_ch[0].status = Motor_decSpeed;//Е┤▐И?÷Е│°Ф╜?

					}
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

        switch(motor_ch[1].status)
        {
            case Motor_Stop:
                break;
            case Motor_incSpeed:
            	if(PWM2_pulseCount >= motor_ch[1].pulseCount.switchRun)
				{
					 motor_ch[1].status = Motor_Run;
					 return;
				}
                if(motor_ch[1].index < CONF_S_LEN)
                {
                	TIM2->ARR = motor_ch[1].arrTable[motor_ch[1].index];
					TIM2->CCR1 = TIM2->ARR >> 1;
					motor_ch[1].index += CONF_S_OFFSET_LET;
                }
                break;
            case Motor_decSpeed:
                if(motor_ch[1].index < CONF_S_OFFSET_LET)
                {
                    motor_ch[1].index = 0;
                    MC_motorStop(1);
                    return;
                }
            	motor_ch[1].index -= CONF_S_OFFSET_LET;
            	TIM2->ARR = motor_ch[1].arrTable[motor_ch[1].index];
                TIM2->CCR1 = TIM2->ARR >> 1;
                break;
            case Motor_Run:
                if(PWM2_pulseCount >= motor_ch[1].pulseCount.switchDec)
                {
                    motor_ch[1].status = Motor_decSpeed;
                }
                break;
            case Motor_runForever:
                if(motor_ch[1].index < CONF_S_LEN)
                {
                    TIM2->ARR = motor_ch[1].arrTable[motor_ch[1].index];
                    TIM2->CCR1 = TIM2->ARR >> 1;
                    motor_ch[1].index += CONF_S_OFFSET_LET;
                }
                if((motor_ch[1].pulseCountLimit > 0) && (PWM2_pulseCount > motor_ch[1].pulseCountLimit))
                {
                	motor_ch[1].status = Motor_decSpeed;
                }
                break;
            case Motor_run2Stop:
                if(PWM2_pulseCount >= motor_ch[1].pulseCount.switchStop)
                {
                	MC_motorStop(1);
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

        switch(motor_ch[2].status)
        {
            case Motor_Stop:
                break;
            case Motor_incSpeed:
            	if(PWM3_pulseCount >= motor_ch[2].pulseCount.switchRun)
            	{
            		motor_ch[2].status = Motor_Run;
            		return;
            	}
				if(motor_ch[2].index < CONF_S_LEN)
                {
					TIM3->ARR = motor_ch[2].arrTable[motor_ch[2].index];
					TIM3->CCR3 = TIM3->ARR >> 1;
					motor_ch[2].index += CONF_S_OFFSET_BO;
                }
                break;
            case Motor_decSpeed:
                if(motor_ch[2].index < CONF_S_OFFSET_BO)
                {
                    motor_ch[2].index = 0;
                    MC_motorStop(2);
                    return;
                }
				motor_ch[2].index -= CONF_S_OFFSET_BO;
                TIM3->ARR = motor_ch[2].arrTable[motor_ch[2].index];
                TIM3->CCR3 = TIM3->ARR >> 1;
                break;
            case Motor_Run:
                if(PWM3_pulseCount >= motor_ch[2].pulseCount.switchDec)
                {
                    motor_ch[2].status = Motor_decSpeed;
                }
                break;
            case Motor_runForever:
                if(motor_ch[2].index < CONF_S_LEN)
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

				if (motor_ch[2].index < CONF_S_LEN)
				{
					TIM3->ARR = motor_ch[2].arrTable[motor_ch[2].index];
					TIM3->CCR3 = TIM3->ARR >> 1;
					motor_ch[2].index+= CONF_S_OFFSET_BO;
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

                if((motor_ch[2].pulseCountLimit > 0) && (PWM3_pulseCount > motor_ch[2].pulseCountLimit))
                {
                    motor_ch[2].stopEvent = EVENT_exceedPulseCountLimit;
                    //MC_motorStop(2);
                    motor_ch[2].status = Motor_decSpeed;

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
			if(IBIO_getInput(6) == 0)
			{
				if(++SsorLowCnt4 >= 5)
				{
					SsorLowCnt4 = 0;
					systemPara.UPSLmode = 0;
//					motor_ch[3].status = Motor_decSpeed;
					MC_motorStop(3);
				}
			}
        }
        switch(motor_ch[3].status)
        {
            case Motor_Stop:
                break;
            case Motor_incSpeed:
            	if(PWM4_pulseCount >= motor_ch[3].pulseCount.switchRun)
				{
					motor_ch[3].status = Motor_Run;
					return;
				}
            	if(motor_ch[3].index < CONF_S_LEN)
				{
            		TIM4->ARR = motor_ch[3].arrTable[motor_ch[3].index];
					TIM4->CCR1 = TIM4->ARR >> 1;
					motor_ch[3].index += CONF_S_OFFSET_SHOU;
				}
                break;
            case Motor_decSpeed:
                if(motor_ch[3].index < CONF_S_OFFSET_SHOU)
                {
                    motor_ch[3].index = 0;
                    MC_motorStop(3);
                    return;
                }
            	motor_ch[3].index -= CONF_S_OFFSET_SHOU;
            	TIM4->ARR = motor_ch[3].arrTable[motor_ch[3].index];
                TIM4->CCR1 = TIM4->ARR >> 1;
                break;
            case Motor_Run:
                if(PWM4_pulseCount >= motor_ch[3].pulseCount.switchDec)
                {
                    motor_ch[3].status = Motor_decSpeed;
                }
                break;
            case Motor_runForever:
                if(motor_ch[3].index <= CONF_S_LEN)
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
                break;
        }
        
        TIM4->SR &= ~TIM_SR_UIF;
    }
}

