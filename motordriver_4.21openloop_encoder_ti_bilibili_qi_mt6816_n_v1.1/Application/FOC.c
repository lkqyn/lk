#include "FOC.h"
#include "tim.h"
#include "math.h"
#include "adc.h"
#include "adc_task.h"
#include <stdlib.h>
#include "myflash.h"
#include "mt6816.h"
#include "pid.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

foc_TypeDef m1_foc;

extern uint16_t mt6816_count;
uint16_t bias = 0;
int16_t Scope[200];
uint16_t angledata[200] = {0};

extern float current[2];
extern float vol;
extern uint8_t limit_vol;
extern uint32_t add_angle;
extern uint16_t init_angle;
extern uint16_t encoder_last_count;

float PID_factor = 1.0f;

static inline uint16_t tim_period(TIM_HandleTypeDef *htim)
{
    return (uint16_t)(__HAL_TIM_GET_AUTORELOAD(htim) + 1U);
}

static inline uint16_t clamp_u16(uint32_t v, uint16_t hi)
{
    return (v > hi) ? hi : (uint16_t)v;
}

static inline float wrap_sin_1024(uint16_t angle)
{
    float rad = (2.0f * (float)M_PI * (float)(angle & 0x03FF)) / 1024.0f;
    return sinf(rad);
}

static inline float wrap_cos_1024(uint16_t angle)
{
    float rad = (2.0f * (float)M_PI * (float)(angle & 0x03FF)) / 1024.0f;
    return cosf(rad);
}

/* ====================== 你的底层驱动：不改 ====================== */
static inline void hb_off(TIM_HandleTypeDef *htim, uint32_t ch)
{
    HAL_TIM_PWM_Stop(htim, ch);
    HAL_TIMEx_PWMN_Stop(htim, ch);
}

static inline void hb_low(TIM_HandleTypeDef *htim, uint32_t ch)
{
    HAL_TIM_PWM_Start(htim, ch);
    HAL_TIMEx_PWMN_Start(htim, ch);
    __HAL_TIM_SET_COMPARE(htim, ch, 0);
}

static inline void hb_pwm(TIM_HandleTypeDef *htim, uint32_t ch, uint16_t duty)
{
    HAL_TIM_PWM_Start(htim, ch);
    HAL_TIMEx_PWMN_Start(htim, ch);
    __HAL_TIM_SET_COMPARE(htim, ch, duty);
}

static inline void HN1_off(void) { hb_off(&htim1, TIM_CHANNEL_1); }
static inline void HN2_off(void) { hb_off(&htim1, TIM_CHANNEL_2); }
static inline void HN3_off(void) { hb_off(&htim1, TIM_CHANNEL_3); }
static inline void HN4_off(void) { hb_off(&htim8, TIM_CHANNEL_1); }

static inline void HN1_low(void) { hb_low(&htim1, TIM_CHANNEL_1); }
static inline void HN2_low(void) { hb_low(&htim1, TIM_CHANNEL_2); }
static inline void HN3_low(void) { hb_low(&htim1, TIM_CHANNEL_3); }
static inline void HN4_low(void) { hb_low(&htim8, TIM_CHANNEL_1); }

static inline void HN1_set(uint16_t d) { hb_pwm(&htim1, TIM_CHANNEL_1, d); }
static inline void HN2_set(uint16_t d) { hb_pwm(&htim1, TIM_CHANNEL_2, d); }
static inline void HN3_set(uint16_t d) { hb_pwm(&htim1, TIM_CHANNEL_3, d); }
static inline void HN4_set(uint16_t d) { hb_pwm(&htim8, TIM_CHANNEL_1, d); }

void FOC_AllOff(void)
{
    HN1_off();
    HN2_off();
    HN3_off();
    HN4_off();
}

/* ====================== 参考代码算法层 ====================== */
void get_scope(void)
{
    Flash_Read_Angle(angledata);
    Flash_Read_Angle(angledata);

    for(uint8_t i = 0; i < 200; i++)
    {
        if(i != 199)
            Scope[i] = DIR * (int16_t)(angledata[i + 1] - angledata[i]);
        else
            Scope[i] = DIR * (int16_t)(angledata[0] - angledata[i]);

        if(abs(Scope[i]) > 2000)
        {
            Scope[i] = 16384 + Scope[i];
            bias = i;
        }
    }

    Scope[0] = (int16_t)((Scope[0] + Scope[199] + Scope[198] + Scope[197]) * 0.25f);
    Scope[199] = Scope[0];
}

void FOC_Init(void)
{
    if(flash_read_dir(DIR_AD) == 11)
        DIR = 1;
    else
        DIR = -1;

    set_uduq(&m1_foc, 0.0f, 0.5f);
    foc_open(&m1_foc, 0);
    HAL_Delay(100);

    m1_foc.lead_angle = 0;
    get_scope();
}

void Sector_tracker(void)
{
    uint8_t prev_sector = m1_foc.angle_sector;
    int32_t angle_calc = 0;
    int temp = (DIR * (mt6816_count / 82) + bias + 200) % 200;

    for(int i = temp - 1; i <= temp + 1; i++)
    {
        int idx = (i + 200) % 200;
        int idxn = (idx + 1) % 200;

        if(DIR == 1)
        {
            if(angledata[idxn] > angledata[idx])
            {
                if(mt6816_count >= angledata[idx] && mt6816_count <= angledata[idxn])
                {
                    m1_foc.angle_sector = (uint8_t)idx;
                    break;
                }
            }
            else if(angledata[idxn] < angledata[idx])
            {
                if(mt6816_count >= angledata[idx] && mt6816_count <= (angledata[idxn] + 16384))
                {
                    m1_foc.angle_sector = (uint8_t)idx;
                    break;
                }
            }
        }
        else
        {
            if(angledata[idxn] < angledata[idx])
            {
                if(mt6816_count <= angledata[idx] && mt6816_count >= angledata[idxn])
                {
                    m1_foc.angle_sector = (uint8_t)idx;
                    break;
                }
            }
            else if(angledata[idxn] > angledata[idx])
            {
                if(mt6816_count <= angledata[idx] && (mt6816_count + 16384) >= angledata[idxn])
                {
                    m1_foc.angle_sector = (uint8_t)idx;
                    break;
                }
            }
        }
    }

    m1_foc.scope = Scope[m1_foc.angle_sector];
    if(m1_foc.scope == 0)
    {
        m1_foc.angle_sector = prev_sector;
        m1_foc.scope = Scope[m1_foc.angle_sector];
        if(m1_foc.scope == 0)
        {
            m1_foc.angle = 0;
            return;
        }
    }

    if(DIR * angledata[(m1_foc.angle_sector + 1) % 200] > DIR * angledata[m1_foc.angle_sector])
    {
        angle_calc = (int32_t)(DIR * (int32_t)(mt6816_count - angledata[m1_foc.angle_sector]) * 256 / m1_foc.scope);
    }
    else
    {
        if(DIR == 1)
        {
            if(mt6816_count > 16200)
                angle_calc = (int32_t)((mt6816_count - angledata[m1_foc.angle_sector]) * 256 / m1_foc.scope);
            else
                angle_calc = (int32_t)((16384 - angledata[m1_foc.angle_sector] + mt6816_count) * 256 / m1_foc.scope);
        }
        else
        {
            if(mt6816_count > 16200)
                angle_calc = (int32_t)((16384 + angledata[m1_foc.angle_sector] - mt6816_count) * 256 / m1_foc.scope);
            else
                angle_calc = (int32_t)((-mt6816_count + angledata[m1_foc.angle_sector]) * 256 / m1_foc.scope);
        }
    }

    if(angle_calc < 0)
    {
        if(m1_foc.angle_sector == 0)
            m1_foc.angle_sector = 199;
        else
            m1_foc.angle_sector--;
        angle_calc += 256;
    }
    else if(angle_calc > 255)
    {
        m1_foc.angle_sector = (uint8_t)((m1_foc.angle_sector + 1) % 200);
        angle_calc -= 256;
    }

    if(angle_calc < 0) angle_calc = 0;
    if(angle_calc > 255) angle_calc = 255;
    m1_foc.angle = (int16_t)angle_calc;
}

void setfoc_angle(foc_TypeDef *mfoc, uint16_t angle)
{
    mfoc->sintheta = wrap_sin_1024(angle);
    mfoc->costheta = wrap_cos_1024(angle);
}

void repark_transfer(foc_TypeDef *mfoc)
{
    mfoc->Ualpha = mfoc->costheta * mfoc->Ud - mfoc->sintheta * mfoc->Uq;
    mfoc->Ubeta  = mfoc->sintheta * mfoc->Ud + mfoc->costheta * mfoc->Uq;
}

void svpwmctr(foc_TypeDef *mfoc)
{
    float ualpha = fabsf(mfoc->Ualpha);
    float ubeta = fabsf(mfoc->Ubeta);

    if(mfoc->Ualpha >= 0.0f && mfoc->Ubeta >= 0.0f)
        mfoc->sector = 1;
    else if(mfoc->Ualpha < 0.0f && mfoc->Ubeta >= 0.0f)
        mfoc->sector = 2;
    else if(mfoc->Ualpha < 0.0f && mfoc->Ubeta < 0.0f)
        mfoc->sector = 3;
    else
        mfoc->sector = 4;

    mfoc->Ta = (uint16_t)(ualpha / mfoc->Udc * mfoc->Ts);
    mfoc->Tb = (uint16_t)(ubeta / mfoc->Udc * mfoc->Ts);
}

void set_duty_by_sector(foc_TypeDef *mfoc)
{
    uint16_t per1 = tim_period(&htim1);
    uint16_t per8 = tim_period(&htim8);
    uint16_t dutyA = clamp_u16(mfoc->Ta, per1);
    uint16_t dutyB = clamp_u16(mfoc->Tb, per8);

    if ((mfoc->Ta <= (uint16_t)(mfoc->deadband * mfoc->Ts)) &&
        (mfoc->Tb <= (uint16_t)(mfoc->deadband * mfoc->Ts)))
    {
        FOC_AllOff();
        return;
    }

    switch (mfoc->sector)
    {
        case 1:
            if (mfoc->Ta <= (uint16_t)(mfoc->deadband * mfoc->Ts)) {
                HN1_off(); HN2_off();
            } else {
                HN1_set(dutyA);
                HN2_low();
            }

            if (mfoc->Tb <= (uint16_t)(mfoc->deadband * mfoc->Ts)) {
                HN3_off(); HN4_off();
            } else {
                HN3_set(dutyB);
                HN4_low();
            }
            break;

        case 2:
            if (mfoc->Ta <= (uint16_t)(mfoc->deadband * mfoc->Ts)) {
                HN1_off(); HN2_off();
            } else {
                HN1_low();
                HN2_set(dutyA);
            }

            if (mfoc->Tb <= (uint16_t)(mfoc->deadband * mfoc->Ts)) {
                HN3_off(); HN4_off();
            } else {
                HN3_set(dutyB);
                HN4_low();
            }
            break;

        case 3:
            if (mfoc->Ta <= (uint16_t)(mfoc->deadband * mfoc->Ts)) {
                HN1_off(); HN2_off();
            } else {
                HN1_low();
                HN2_set(dutyA);
            }

            if (mfoc->Tb <= (uint16_t)(mfoc->deadband * mfoc->Ts)) {
                HN3_off(); HN4_off();
            } else {
                HN3_low();
                HN4_set(dutyB);
            }
            break;

        case 4:
            if (mfoc->Ta <= (uint16_t)(mfoc->deadband * mfoc->Ts)) {
                HN1_off(); HN2_off();
            } else {
                HN1_set(dutyA);
                HN2_low();
            }

            if (mfoc->Tb <= (uint16_t)(mfoc->deadband * mfoc->Ts)) {
                HN3_off(); HN4_off();
            } else {
                HN3_low();
                HN4_set(dutyB);
            }
            break;

        default:
            FOC_AllOff();
            break;
    }
}

void set_uduq(foc_TypeDef *mfoc, float ud, float uq)
{
    if(ud > mfoc->outmax || ud < -mfoc->outmax) return;
    if(uq > mfoc->outmax || uq < -mfoc->outmax) return;

    mfoc->Ud = ud;
    mfoc->Uq = uq;
}

void foc_open(foc_TypeDef *mfoc, uint16_t angle)
{
    setfoc_angle(mfoc, angle);
    repark_transfer(mfoc);
    svpwmctr(mfoc);
    set_duty_by_sector(mfoc);
}

void park_transform(foc_TypeDef *mfoc)
{
    mfoc->Iq = -mfoc->Ialpha * mfoc->sintheta + mfoc->Ibeta * mfoc->costheta;
    mfoc->Id =  mfoc->Ialpha * mfoc->costheta + mfoc->Ibeta * mfoc->sintheta;
}

void get_AB_current(foc_TypeDef *mfoc)
{
    float raw_a;
    float raw_b;
    float phase_a;
    float phase_b;

    (void)mfoc;

    raw_a = get_phase_current(ADC_INJECTED_RANK_2, 0);
    raw_b = -get_phase_current(ADC_INJECTED_RANK_1, 1);

    current[0] = raw_a;
    current[1] = raw_b;

    switch(m1_foc.sector)
    {
        case 1:
            phase_a = raw_a;
            phase_b = raw_b;
            break;

        case 2:
            phase_a = -raw_a;
            phase_b = raw_b;
            break;

        case 3:
            phase_a = -raw_a;
            phase_b = -raw_b;
            break;

        case 4:
            phase_a = raw_a;
            phase_b = -raw_b;
            break;

        default:
            phase_a = 0.0f;
            phase_b = 0.0f;
            break;
    }

    m1_foc.Ialpha = phase_b;
    m1_foc.Ibeta  = phase_a;
}

void adjust_I(void)
{
    current[0] = get_phase_current(ADC_INJECTED_RANK_2, 0);
    current[1] = get_phase_current(ADC_INJECTED_RANK_1, 1);
}

void can_foc_init(void)
{
    m1_foc.Ts = tim_period(&htim1);
    m1_foc.Udc = 24.0f;
    m1_foc.outmax = 20.0f;
    m1_foc.deadband = 0.0f;

    if(PID_factor > 2.0f || PID_factor < 0.5f)
        PID_factor = 1.0f;

    m1_foc.kp = 1.8f * PID_factor;
    m1_foc.ki = 0.020f * PID_factor;
    m1_foc.kd = 0.0f;

    m1_foc.tar_Id = 0.0f;
    m1_foc.tar_Iq = 0.0f;

    m1_foc.id_err = 0.0f;
    m1_foc.iq_err = 0.0f;
    m1_foc.id_inter = 0.0f;
    m1_foc.iq_inter = 0.0f;
    m1_foc.last_Id_err = 0.0f;
    m1_foc.last_Iq_err = 0.0f;

    m1_foc.Ls = 0.0041f;
    m1_foc.FLux = 0.002f;
}

void b_foc_init(void)
{
    PID_factor = (*(__IO uint32_t *)(FLASH_USER_START_ADDR + 0x800)) * 0.01f;
    if(PID_factor > 2.0f || PID_factor < 0.5f)
        PID_factor = 1.0f;

    can_foc_init();
    FOC_Init();

    set_uduq(&m1_foc, 0.0f, 0.0f);
    foc_open(&m1_foc, 0);

    add_angle = 0;
    for(uint8_t jj = 0; jj < 250; jj++)
    {
        add_angle += (uint32_t)REIN_MT6816_GetAngleData();
        HAL_Delay(1);
    }

    encoder_last_count = mt6816_count = init_angle = (uint16_t)(add_angle / 250U);
}

void set_foc_Iqcurrent(foc_TypeDef *mfoc, float current_set)
{
    if(current_set > System_MAX_I) current_set = System_MAX_I;
    else if(current_set < -System_MAX_I) current_set = -System_MAX_I;

    mfoc->tar_Iq = current_set;
}

#define OMAX_UQ 10.0f
#define OMAX_UD 6.0f

void foc_current_pid(foc_TypeDef *mfoc)
{
    mfoc->id_err = mfoc->tar_Id - mfoc->Id;
    mfoc->iq_err = mfoc->tar_Iq - mfoc->Iq;

    mfoc->id_inter += mfoc->ki * mfoc->id_err;
    mfoc->iq_inter += mfoc->ki * mfoc->iq_err;

    if(mfoc->id_inter > limit_vol)  mfoc->id_inter = limit_vol;
    if(mfoc->id_inter < -limit_vol) mfoc->id_inter = -limit_vol;

    if(mfoc->iq_inter > limit_vol)  mfoc->iq_inter = limit_vol;
    if(mfoc->iq_inter < -limit_vol) mfoc->iq_inter = -limit_vol;

    mfoc->Ud = mfoc->kp * mfoc->id_err + mfoc->id_inter + mfoc->kd * (mfoc->id_err - mfoc->last_Id_err);
    mfoc->Uq = mfoc->kp * mfoc->iq_err + mfoc->iq_inter + mfoc->kd * (mfoc->iq_err - mfoc->last_Iq_err);

    if(mfoc->Ud > OMAX_UD) mfoc->Ud = OMAX_UD;
    if(mfoc->Ud < -OMAX_UD) mfoc->Ud = -OMAX_UD;

    if(mfoc->Uq > OMAX_UQ) mfoc->Uq = OMAX_UQ;
    if(mfoc->Uq < -OMAX_UQ) mfoc->Uq = -OMAX_UQ;

    mfoc->last_Id_err = mfoc->id_err;
    mfoc->last_Iq_err = mfoc->iq_err;
}

void current_ctr(foc_TypeDef *mfoc, uint16_t angle)
{
    setfoc_angle(mfoc, angle);
    park_transform(mfoc);
    foc_current_pid(mfoc);
    repark_transfer(mfoc);
    svpwmctr(mfoc);
    set_duty_by_sector(mfoc);
}
