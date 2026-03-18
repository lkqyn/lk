#include "FOC.h"
#include "tim.h"
#include "math.h"
#include  "encoder.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ====================== 四路半桥通道映射 ======================
// A+ : TIM1_CH1 (PA8)  / CH1N (PB13)
// A- : TIM1_CH2 (PA9)  / CH2N (PB14)
// B+ : TIM1_CH3 (PA10) / CH3N (PB15)
// B- : TIM8_CH1 (PC6)  / CH1N (PA7)

foc_TypeDef m1_foc;

// ====================== 基础工具 ======================
static inline uint16_t tim_period(TIM_HandleTypeDef *htim)
{
    return (uint16_t)(__HAL_TIM_GET_AUTORELOAD(htim) + 1);
}

static inline uint16_t clamp_u16(uint32_t v, uint16_t hi)
{
    return (v > hi) ? hi : (uint16_t)v;
}

static inline float f_abs(float x)
{
    return (x >= 0.0f) ? x : -x;
}

// ====================== 半桥底层 ======================
// 真OFF：Stop CH + Stop CHN，输出进入 IdleState
static inline void hb_off(TIM_HandleTypeDef *htim, uint32_t ch)
{
    HAL_TIM_PWM_Stop(htim, ch);
    HAL_TIMEx_PWMN_Stop(htim, ch);
}

// LOW：下桥导通（刹车态）
static inline void hb_low(TIM_HandleTypeDef *htim, uint32_t ch)
{
    HAL_TIM_PWM_Start(htim, ch);
    HAL_TIMEx_PWMN_Start(htim, ch);
    __HAL_TIM_SET_COMPARE(htim, ch, 0);
}

// PWM：同相PWM
static inline void hb_pwm(TIM_HandleTypeDef *htim, uint32_t ch, uint16_t duty)
{
    HAL_TIM_PWM_Start(htim, ch);
    HAL_TIMEx_PWMN_Start(htim, ch);
    __HAL_TIM_SET_COMPARE(htim, ch, duty);
}

// ====================== 四个半桥封装 ======================
static inline void HN1_off(void) { hb_off(&htim1, TIM_CHANNEL_1); } // A+
static inline void HN2_off(void) { hb_off(&htim1, TIM_CHANNEL_2); } // A-
static inline void HN3_off(void) { hb_off(&htim1, TIM_CHANNEL_3); } // B+
static inline void HN4_off(void) { hb_off(&htim8, TIM_CHANNEL_1); } // B-

static inline void HN1_low(void) { hb_low(&htim1, TIM_CHANNEL_1); }
static inline void HN2_low(void) { hb_low(&htim1, TIM_CHANNEL_2); }
static inline void HN3_low(void) { hb_low(&htim1, TIM_CHANNEL_3); }
static inline void HN4_low(void) { hb_low(&htim8, TIM_CHANNEL_1); }

static inline void HN1_set(uint16_t d) { hb_pwm(&htim1, TIM_CHANNEL_1, d); }
static inline void HN2_set(uint16_t d) { hb_pwm(&htim1, TIM_CHANNEL_2, d); }
static inline void HN3_set(uint16_t d) { hb_pwm(&htim1, TIM_CHANNEL_3, d); }
static inline void HN4_set(uint16_t d) { hb_pwm(&htim8, TIM_CHANNEL_1, d); }

// ====================== 总关闭 ======================
void FOC_AllOff(void)
{
    HN1_off();
    HN2_off();
    HN3_off();
    HN4_off();
}

// ====================== 初始化 ======================
void FOC_Init(void)
{
    m1_foc.Ts       = (float)tim_period(&htim1);   // 建议TIM1/TIM8 ARR一致
    m1_foc.Udc      = 12.0f;
    m1_foc.outmax   = 12.0f;

    m1_foc.Ud       = 0.0f;
    m1_foc.Uq       = 0.0f;
    m1_foc.Ualpha   = 0.0f;
    m1_foc.Ubeta    = 0.0f;

    m1_foc.angle    = 0.0f;
    m1_foc.sintheta = 0.0f;
    m1_foc.costheta = 1.0f;

    m1_foc.fe       = 0.0f;

    m1_foc.duty_max = 0.9f;   // 安全默认值
    m1_foc.deadband = 0.0f;

    m1_foc.sector   = 1;
    m1_foc.Ta       = 0;
    m1_foc.Tb       = 0;

    FOC_AllOff();
}
// 适配增量式AB编码器，不再依赖 angledata[] / Scope[]
void Sector_tracker_inc_encoder(foc_TypeDef *mfoc, encoder_t *enc)
{
    // 当前单圈计数，归一到 0~3999
    int32_t cnt = Encoder_GetCountInOneTurn(enc);
    if (cnt < 0) cnt += 4000;

    // 每机械转 50 个电角周期
    // 每个电角周期 = 4000 / 50 = 80 count
    const int32_t counts_per_elec_cycle = 80;

    // 1个电角周期再分4个扇区，每扇区 20 count
    const int32_t counts_per_sector = 20;

    // 当前位于第几个电角周期 0~49
    int32_t elec_cycle_idx = cnt / counts_per_elec_cycle;

    // 周期内位置 0~79
    int32_t cnt_in_cycle = cnt % counts_per_elec_cycle;

    // 当前在该电角周期内的第几个90°扇区 0~3
    int32_t sub_sector = cnt_in_cycle / counts_per_sector;

    // 扇区内位置 0~19
    int32_t cnt_in_sector = cnt_in_cycle % counts_per_sector;

    // 对齐第一份代码的“总 sector 编号”
    // 一圈 50 个电角周期 * 4 = 200 个扇区
    mfoc->angle_sector = (uint16_t)(elec_cycle_idx * 4 + sub_sector);

    // 当前扇区宽度固定 20 count
    mfoc->scope = (int16_t)counts_per_sector;

    // 映射为扇区内 0~255 细分角度
    mfoc->angle = (uint16_t)((cnt_in_sector * 256) / counts_per_sector);
}
// ====================== 设置Ud/Uq ======================
void set_uduq(foc_TypeDef *mfoc, float ud, float uq)
{
    if (ud > mfoc->outmax)  ud = mfoc->outmax;
    if (ud < -mfoc->outmax) ud = -mfoc->outmax;
    if (uq > mfoc->outmax)  uq = mfoc->outmax;
    if (uq < -mfoc->outmax) uq = -mfoc->outmax;

    mfoc->Ud = ud;
    mfoc->Uq = uq;
}

// ====================== 设置电角度 ======================
void setfoc_angle(foc_TypeDef *mfoc, float angle)
{
    mfoc->angle = angle;

    while (mfoc->angle >= 2.0f * (float)M_PI) mfoc->angle -= 2.0f * (float)M_PI;
    while (mfoc->angle <  0.0f)               mfoc->angle += 2.0f * (float)M_PI;

    mfoc->sintheta = sinf(mfoc->angle);
    mfoc->costheta = cosf(mfoc->angle);
}

// ====================== 反Park变换 ======================
void repark_transfer(foc_TypeDef *mfoc)
{
    mfoc->Ualpha = mfoc->costheta * mfoc->Ud - mfoc->sintheta * mfoc->Uq;
    mfoc->Ubeta  = mfoc->sintheta * mfoc->Ud + mfoc->costheta * mfoc->Uq;
}

// ====================== 4扇区简化调制 ======================
// 与你参考代码保持风格：
// 1) 先按Ualpha/Ubeta符号判sector
// 2) Ta/Tb分别对应A/B两相电压幅值
void svpwmctr(foc_TypeDef *mfoc)
{
    float ualpha = f_abs(mfoc->Ualpha);
    float ubeta  = f_abs(mfoc->Ubeta);

    if (mfoc->Udc <= 0.1f) {
        mfoc->sector = 1;
        mfoc->Ta = 0;
        mfoc->Tb = 0;
        return;
    }

    if      (mfoc->Ualpha >= 0.0f && mfoc->Ubeta >= 0.0f) mfoc->sector = 1;
    else if (mfoc->Ualpha <  0.0f && mfoc->Ubeta >= 0.0f) mfoc->sector = 2;
    else if (mfoc->Ualpha <  0.0f && mfoc->Ubeta <  0.0f) mfoc->sector = 3;
    else                                                  mfoc->sector = 4;

    // 转为占空
    if (ualpha > mfoc->Udc * mfoc->duty_max) ualpha = mfoc->Udc * mfoc->duty_max;
    if (ubeta  > mfoc->Udc * mfoc->duty_max) ubeta  = mfoc->Udc * mfoc->duty_max;

    mfoc->Ta = (uint16_t)(ualpha / mfoc->Udc * mfoc->Ts);
    mfoc->Tb = (uint16_t)(ubeta  / mfoc->Udc * mfoc->Ts);

    if (mfoc->Ta > (uint16_t)mfoc->Ts) mfoc->Ta = (uint16_t)mfoc->Ts;
    if (mfoc->Tb > (uint16_t)mfoc->Ts) mfoc->Tb = (uint16_t)mfoc->Ts;
}

// ====================== 按扇区下发到你的四路半桥 ======================
void set_duty_by_sector(foc_TypeDef *mfoc)
{
    uint16_t per1 = tim_period(&htim1);
    uint16_t per8 = tim_period(&htim8);

    // A相占空基于TIM1
    uint16_t dutyA = clamp_u16(mfoc->Ta, per1);

    // B相占空最好要求TIM1/TIM8 ARR一致
    // 若不一致，这里按TIM8限制
    uint16_t dutyB = clamp_u16(mfoc->Tb, per8);

    // 小占空直接关断
    if ((mfoc->Ta <= (uint16_t)(mfoc->deadband * mfoc->Ts)) &&
        (mfoc->Tb <= (uint16_t)(mfoc->deadband * mfoc->Ts)))
    {
        FOC_AllOff();
        return;
    }

    switch (mfoc->sector)
    {
        case 1: // A+, B+
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

        case 2: // A-, B+
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

        case 3: // A-, B-
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

        case 4: // A+, B-
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

// ====================== 开环输出 ======================
void foc_open(foc_TypeDef *mfoc, float angle)
{
    setfoc_angle(mfoc, angle);
    repark_transfer(mfoc);
    svpwmctr(mfoc);
    set_duty_by_sector(mfoc);
}

// ====================== 兼容你当前项目的开环接口 ======================
void FOC_OpenLoopInit(foc_TypeDef *mfoc, float vbus, float uq, float fe_hz)
{
    mfoc->Udc   = vbus;
    mfoc->Ud    = 0.0f;
    mfoc->Uq    = uq;
    mfoc->fe    = fe_hz;
    mfoc->angle = 0.0f;

    FOC_AllOff();
}

void FOC_SetDefaultParams(foc_TypeDef *mfoc)
{
    mfoc->duty_max = 0.5f;
    mfoc->deadband = 0.0f;
}

void FOC_OpenLoopUpdate(foc_TypeDef *mfoc, float dt)
{
    float w = 2.0f * (float)M_PI * mfoc->fe;
    mfoc->angle += w * dt;

    while (mfoc->angle >= 2.0f * (float)M_PI) mfoc->angle -= 2.0f * (float)M_PI;
    while (mfoc->angle <  0.0f)               mfoc->angle += 2.0f * (float)M_PI;

    foc_open(mfoc, mfoc->angle);
}

void FOC_OutputByTheta(foc_TypeDef *mfoc, float angle, float uq)
{
    mfoc->Uq = uq;
    foc_open(mfoc, angle);
}

void FOC_OpenLoopSetFe(foc_TypeDef *mfoc, float fe_hz)
{
    mfoc->fe = fe_hz;
}

void FOC_OpenLoopSetUqRatio(foc_TypeDef *mfoc, float uq_ratio)
{
    if (uq_ratio < 0.0f) uq_ratio = 0.0f;
    if (uq_ratio > mfoc->duty_max) uq_ratio = mfoc->duty_max;

    mfoc->Uq = uq_ratio * mfoc->Udc;
}

void FOC_OpenLoopRampUqRatio(foc_TypeDef *mfoc, float target_ratio, float step_per_call)
{
    float cur = (mfoc->Udc > 0.1f) ? (mfoc->Uq / mfoc->Udc) : 0.0f;

    if (target_ratio < 0.0f) target_ratio = 0.0f;
    if (target_ratio > mfoc->duty_max) target_ratio = mfoc->duty_max;
    if (step_per_call < 0.0f) step_per_call = -step_per_call;

    if (cur < target_ratio) {
        cur += step_per_call;
        if (cur > target_ratio) cur = target_ratio;
    } else if (cur > target_ratio) {
        cur -= step_per_call;
        if (cur < target_ratio) cur = target_ratio;
    }

    mfoc->Uq = cur * mfoc->Udc;
}