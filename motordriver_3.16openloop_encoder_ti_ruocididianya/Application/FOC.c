#include "FOC.h"
#include "tim.h"
#include "math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ====================== 四路半桥通道映射（按你当前工程） ======================
// A+ : TIM1_CH1 (PA8)  / CH1N (PB13)
// A- : TIM1_CH2 (PA9)  / CH2N (PB14)
// B+ : TIM1_CH3 (PA10) / CH3N (PB15)
// B- : TIM8_CH1 (PC6)  / CH1N (PA7)

// ====================== 基础工具 ======================
static inline uint16_t tim_period(TIM_HandleTypeDef *htim)
{
    return (uint16_t)(__HAL_TIM_GET_AUTORELOAD(htim) + 1U);
}

static inline uint16_t clamp_u16(uint32_t v, uint16_t hi)
{
    return (v > hi) ? hi : (uint16_t)v;
}

static inline float f_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static inline float f_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float wrap_pm_2pi(float x)
{
    const float two_pi = 2.0f * (float)M_PI;

    while (x >= two_pi) x -= two_pi;
    while (x < 0.0f)    x += two_pi;

    return x;
}

// ====================== 半桥底层 ======================
// --- 真OFF：Stop CH + Stop CHN，输出进入 IdleState ---
static inline void hb_off(TIM_HandleTypeDef *htim, uint32_t ch)
{
    HAL_TIM_PWM_Stop(htim, ch);
    HAL_TIMEx_PWMN_Stop(htim, ch);
}

// --- LOW（刹车）：Start CH+CHN 并置 CCR=0 -> 恒 00（下管导通）---
static inline void hb_low(TIM_HandleTypeDef *htim, uint32_t ch)
{
    HAL_TIM_PWM_Start(htim, ch);
    HAL_TIMEx_PWMN_Start(htim, ch);
    __HAL_TIM_SET_COMPARE(htim, ch, 0);
}

// --- PWM：Start CH+CHN 并写 CCR=duty -> 同相PWM ---
static inline void hb_pwm(TIM_HandleTypeDef *htim, uint32_t ch, uint16_t duty)
{
    HAL_TIM_PWM_Start(htim, ch);
    HAL_TIMEx_PWMN_Start(htim, ch);
    __HAL_TIM_SET_COMPARE(htim, ch, duty);
}

// ---------------------- 四路半桥封装 ----------------------
static inline void AP_OFF(void) { hb_off(&htim1, TIM_CHANNEL_1); }
static inline void AN_OFF(void) { hb_off(&htim1, TIM_CHANNEL_2); }
static inline void BP_OFF(void) { hb_off(&htim1, TIM_CHANNEL_3); }
static inline void BN_OFF(void) { hb_off(&htim8, TIM_CHANNEL_1); }

static inline void AP_LOW(void) { hb_low(&htim1, TIM_CHANNEL_1); }
static inline void AN_LOW(void) { hb_low(&htim1, TIM_CHANNEL_2); }
static inline void BP_LOW(void) { hb_low(&htim1, TIM_CHANNEL_3); }
static inline void BN_LOW(void) { hb_low(&htim8, TIM_CHANNEL_1); }

static inline void AP_PWM(uint16_t d) { hb_pwm(&htim1, TIM_CHANNEL_1, d); }
static inline void AN_PWM(uint16_t d) { hb_pwm(&htim1, TIM_CHANNEL_2, d); }
static inline void BP_PWM(uint16_t d) { hb_pwm(&htim1, TIM_CHANNEL_3, d); }
static inline void BN_PWM(uint16_t d) { hb_pwm(&htim8, TIM_CHANNEL_1, d); }

// ====================== 输出层：Va/Vb -> 四路半桥 ======================
/**
 * @brief 对A相、B相施加电压（Va=Ualpha, Vb=Ubeta）
 *        方向靠符号决定；幅值靠占空比决定
 *
 *        这里仍然沿用你现有工程的“单桥臂PWM + 另一桥臂LOW”的方式，
 *        不强行改成别人的 sector/A4950 专用写法。
 */
void foc_apply_Vab(const foc_ol_t *foc, float Va, float Vb)
{
    const uint16_t per1 = tim_period(&htim1);
    const uint16_t per8 = tim_period(&htim8);

    float da = 0.0f;
    float db = 0.0f;

    if (foc->Vdc > 0.1f)
    {
        da = f_absf(Va) / foc->Vdc;
        db = f_absf(Vb) / foc->Vdc;
    }

    da = f_clampf(da, 0.0f, foc->duty_max);
    db = f_clampf(db, 0.0f, foc->duty_max);

    // A相
    if (da < foc->deadband)
    {
        AP_OFF();
        AN_OFF();
    }
    else
    {
        uint16_t dutyA = clamp_u16((uint32_t)(da * (float)per1), per1);

        if (Va >= 0.0f)
        {
            // A正向：A+ PWM, A- LOW
            AP_PWM(dutyA);
            AN_LOW();
        }
        else
        {
            // A反向：A- PWM, A+ LOW
            AP_LOW();
            AN_PWM(dutyA);
        }
    }

    // B相
    if (db < foc->deadband)
    {
        BP_OFF();
        BN_OFF();
    }
    else
    {
        uint16_t dutyB = clamp_u16((uint32_t)(db * (float)per8), per8);

        if (Vb >= 0.0f)
        {
            // B正向：B+ PWM, B- LOW
            BP_PWM(dutyB);
            BN_LOW();
        }
        else
        {
            // B反向：B- PWM, B+ LOW
            BP_LOW();
            BN_PWM(dutyB);
        }
    }
}

// ====================== FOC 开环主体 ======================
void FOC_OpenLoopInit(foc_ol_t *foc, float Vdc, float Uq, float fe_hz)
{
    foc->Vdc   = Vdc;
    foc->Uq    = Uq;
    foc->Ud    = 0.0f;
    foc->fe    = fe_hz;
    foc->theta = 0.0f;

    FOC_AllOff();
}

void FOC_SetDefaultParams(foc_ol_t *foc)
{
    // 你原来是 0.5f，会限制最高可用电压。
    // 这里我提高到 0.90f，更适合你现在做高速试验。
    // 如果后面觉得太激进，可退回 0.80f。
    foc->duty_max = 0.90f;

    // 小于 deadband 则真OFF，避免极小电压时一直刹车发热
    foc->deadband = 0.0f;
}

void FOC_OpenLoopUpdate(foc_ol_t *foc, float dt)
{
    const float w = 2.0f * (float)M_PI * foc->fe;

    foc->theta += w * dt;
    foc->theta = wrap_pm_2pi(foc->theta);

    {
        const float s = sinf(foc->theta);
        const float c = cosf(foc->theta);

        // 反Park
        // Ualpha = c*Ud - s*Uq
        // Ubeta  = s*Ud + c*Uq
        const float Ualpha = c * foc->Ud - s * foc->Uq;
        const float Ubeta  = s * foc->Ud + c * foc->Uq;

        foc_apply_Vab(foc, Ualpha, Ubeta);
    }
}

void FOC_OutputByTheta(foc_ol_t *foc, float theta, float Uq)
{
    const float th = wrap_pm_2pi(theta);
    const float s  = sinf(th);
    const float c  = cosf(th);

    foc->theta = th;
    foc->Uq    = Uq;

    {
        const float Ualpha = c * foc->Ud - s * foc->Uq;
        const float Ubeta  = s * foc->Ud + c * foc->Uq;

        foc_apply_Vab(foc, Ualpha, Ubeta);
    }
}

void FOC_AllOff(void)
{
    AP_OFF();
    AN_OFF();
    BP_OFF();
    BN_OFF();
}

void FOC_OpenLoopSetFe(foc_ol_t *foc, float fe_hz)
{
    foc->fe = fe_hz;
}

void FOC_OpenLoopSetUqRatio(foc_ol_t *foc, float uq_ratio)
{
    uq_ratio = f_clampf(uq_ratio, 0.0f, foc->duty_max);
    foc->Uq  = uq_ratio * foc->Vdc;
}

void FOC_OpenLoopRampUqRatio(foc_ol_t *foc, float target_ratio, float step_per_call)
{
    float cur = 0.0f;

    if (foc->Vdc > 0.1f)
        cur = foc->Uq / foc->Vdc;

    target_ratio = f_clampf(target_ratio, 0.0f, foc->duty_max);

    if (step_per_call < 0.0f)
        step_per_call = -step_per_call;

    if (cur < target_ratio)
    {
        cur += step_per_call;
        if (cur > target_ratio) cur = target_ratio;
    }
    else if (cur > target_ratio)
    {
        cur -= step_per_call;
        if (cur < target_ratio) cur = target_ratio;
    }

    foc->Uq = cur * foc->Vdc;
}