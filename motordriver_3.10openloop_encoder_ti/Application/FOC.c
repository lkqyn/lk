#include "FOC.h"
#include "tim.h"
#include "math.h"

// ====================== 四路半桥通道映射（按IO表） ======================
// A+ : TIM1_CH1 (PA8)  / CH1N (PB13)
// A- : TIM1_CH2 (PA9)  / CH2N (PB14)
// B+ : TIM1_CH3 (PA10) / CH3N (PB15)
// B- : TIM8_CH1 (PC6)  / CH1N (PA7)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline uint16_t tim_period(TIM_HandleTypeDef *htim)
{
    return (uint16_t)(__HAL_TIM_GET_AUTORELOAD(htim) + 1);
}

static inline uint16_t clamp_u16(uint32_t v, uint16_t hi)
{
    return (v > hi) ? hi : (uint16_t)v;
}

// --- 真OFF：Stop CH + Stop CHN，输出进入 IdleState（你已配 CH=1 / CHN=0）---
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

// --- PWM：Start CH+CHN 并写 CCR = duty -> 00/11 同相PWM ---
static inline void hb_pwm(TIM_HandleTypeDef *htim, uint32_t ch, uint16_t duty)
{
    HAL_TIM_PWM_Start(htim, ch);
    HAL_TIMEx_PWMN_Start(htim, ch);
    __HAL_TIM_SET_COMPARE(htim, ch, duty);
}

// ---------------------- 对四路半桥的封装 ----------------------
static inline void AP_OFF(void){ hb_off(&htim1, TIM_CHANNEL_1); }
static inline void AN_OFF(void){ hb_off(&htim1, TIM_CHANNEL_2); }
static inline void BP_OFF(void){ hb_off(&htim1, TIM_CHANNEL_3); }
static inline void BN_OFF(void){ hb_off(&htim8, TIM_CHANNEL_1); }

static inline void AP_LOW(void){ hb_low(&htim1, TIM_CHANNEL_1); }
static inline void AN_LOW(void){ hb_low(&htim1, TIM_CHANNEL_2); }
static inline void BP_LOW(void){ hb_low(&htim1, TIM_CHANNEL_3); }
static inline void BN_LOW(void){ hb_low(&htim8, TIM_CHANNEL_1); }

static inline void AP_PWM(uint16_t d){ hb_pwm(&htim1, TIM_CHANNEL_1, d); }
static inline void AN_PWM(uint16_t d){ hb_pwm(&htim1, TIM_CHANNEL_2, d); }
static inline void BP_PWM(uint16_t d){ hb_pwm(&htim1, TIM_CHANNEL_3, d); }
static inline void BN_PWM(uint16_t d){ hb_pwm(&htim8, TIM_CHANNEL_1, d); }

// ====================== 输出层：把 Va/Vb（±电压）映射到四路半桥 ======================
static inline float f_abs(float x){ return (x >= 0.0f) ? x : -x; }
static inline float f_clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/**
 * @brief  对A相、B相施加电压（Va=Ualpha, Vb=Ubeta）
 *         方向靠符号决定；幅值靠占空比决定
 */
void foc_apply_Vab(const foc_ol_t *foc, float Va, float Vb)
{
    // 取周期（TIM1 与 TIM8 建议你保持相同ARR，若不同也没关系，各自算）
    const uint16_t per1 = tim_period(&htim1);
    const uint16_t per8 = tim_period(&htim8);

    // 归一化占空比
    float da = (foc->Vdc > 0.1f) ? (f_abs(Va) / foc->Vdc) : 0.0f;
    float db = (foc->Vdc > 0.1f) ? (f_abs(Vb) / foc->Vdc) : 0.0f;
    da = f_clamp(da, 0.0f, foc->duty_max);
    db = f_clamp(db, 0.0f, foc->duty_max);

    // 小于 deadband 直接真 OFF（避免一直刹车发热/噪声）
    if (da < foc->deadband) {
        AP_OFF(); AN_OFF();
    } else {
        uint16_t dutyA = clamp_u16((uint32_t)(da * (float)per1), per1);
        if (Va >= 0.0f) {       // A正向：AP PWM, AN LOW
            AP_PWM(dutyA);
            AN_LOW();
        } else {                // A反向：AN PWM, AP LOW
            AP_LOW();
            AN_PWM(dutyA);
        }
    }

    if (db < foc->deadband) {
        BP_OFF(); BN_OFF();
    } else {
        uint16_t dutyB = clamp_u16((uint32_t)(db * (float)per8), per8);
        if (Vb >= 0.0f) {       // B正向：BP PWM, BN LOW
            BP_PWM(dutyB);
            BN_LOW();
        } else {                // B反向：BN PWM, BP LOW
            BP_LOW();
            BN_PWM(dutyB);
        }
    }
}

// ====================== 开环FOC主体（借鉴你贴的 repark 思路） ======================
void FOC_OpenLoopInit(foc_ol_t *foc, float Vdc, float Uq, float fe_hz)
{
    foc->Vdc      = Vdc;
    foc->Uq       = Uq;
    foc->Ud       = 0.0f;
    foc->fe       = fe_hz;

    foc->theta    = 0.0f;
    FOC_AllOff();
}

void FOC_SetDefaultParams(foc_ol_t *foc)
{
    foc->duty_max = 0.5f;   // 安全值
    foc->deadband = 0.0f;    
}

void FOC_OpenLoopUpdate(foc_ol_t *foc, float dt)
{
    // 1) 更新电角度
    float w = 2.0f * (float)M_PI * foc->fe;
    foc->theta += w * dt;
    // wrap到 [0, 2π)
    while (foc->theta >= 2.0f * (float)M_PI) foc->theta -= 2.0f * (float)M_PI;
    while (foc->theta < 0.0f)               foc->theta += 2.0f * (float)M_PI;

    // 2) 计算 sin/cos（贴的代码用查表，先用 sinf/cosf，跑通后再换查表优化）
    float s = sinf(foc->theta);
    float c = cosf(foc->theta);

    // 3) 反Park
    // Ualpha = c*Ud - s*Uq
    // Ubeta  = s*Ud + c*Uq
    float Ualpha = c * foc->Ud - s * foc->Uq;
    float Ubeta  = s * foc->Ud + c * foc->Uq;

    // 4) 输出到两相半桥
    foc_apply_Vab(foc, Ualpha, Ubeta);
}


void FOC_OutputByTheta(foc_ol_t *foc, float theta, float Uq)
{
    foc->theta = theta;
    foc->Uq = Uq;

    float s = sinf(theta);
    float c = cosf(theta);

    float Ualpha = c * foc->Ud - s * foc->Uq;
    float Ubeta  = s * foc->Ud + c * foc->Uq;

    foc_apply_Vab(foc, Ualpha, Ubeta);
}


void FOC_AllOff(void)
{
    AP_OFF(); AN_OFF();
    BP_OFF(); BN_OFF();
}

void FOC_OpenLoopSetFe(foc_ol_t *foc, float fe_hz)
{
    foc->fe = fe_hz;
}

void FOC_OpenLoopSetUqRatio(foc_ol_t *foc, float uq_ratio)
{
    if (uq_ratio < 0.0f) uq_ratio = 0.0f;
    if (uq_ratio > foc->duty_max) uq_ratio = foc->duty_max;
    foc->Uq = uq_ratio * foc->Vdc;
}

// 每次调用让Uq比例向目标靠近一点
void FOC_OpenLoopRampUqRatio(foc_ol_t *foc, float target_ratio, float step_per_call)
{
    float cur = (foc->Vdc > 0.1f) ? (foc->Uq / foc->Vdc) : 0.0f;

    if (target_ratio < 0.0f) target_ratio = 0.0f;
    if (target_ratio > foc->duty_max) target_ratio = foc->duty_max;
    if (step_per_call < 0.0f) step_per_call = -step_per_call;

    if (cur < target_ratio) {
        cur += step_per_call;
        if (cur > target_ratio) cur = target_ratio;
    } else if (cur > target_ratio) {
        cur -= step_per_call;
        if (cur < target_ratio) cur = target_ratio;
    }

    foc->Uq = cur * foc->Vdc;
}
