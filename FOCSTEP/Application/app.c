#include "app.h"

#include "current.h"
#include "debug.h"
#include "encoder.h"
#include "foc.h"
#include "foc_v.h"
#include "main.h"
#include "motion_ctrl.h"
#include "motor_pwm.h"
#include <math.h>

#define APP_BUS_VOLTAGE_V              (24.0f)
#define APP_ALIGN_VOLTAGE_V            (3.0f)
#define APP_ALIGN_UD                   (APP_ALIGN_VOLTAGE_V / APP_BUS_VOLTAGE_V)
#define APP_ALIGN_ANGLE_RAD            (0.0f)
#define APP_ALIGN_TIME_MS              (1000U)
#define APP_FAST_LOOP_HZ               (20000U)
#define APP_FAST_LOOP_DT_S             (1.0f / (float)APP_FAST_LOOP_HZ)
#define APP_SPEED_LOOP_DIV             (20U)
#define APP_ALIGN_TICKS                ((APP_ALIGN_TIME_MS * APP_FAST_LOOP_HZ) / 1000U)
#define APP_OVERSPEED_PROTECT_RPM      (4000.0f)
#define APP_POLE_PAIRS                 (50.0f)
#define APP_MODEL_PSI_WB               (0.0120f)
#define APP_MODEL_LD_H                 (0.00185f)
#define APP_MODEL_LQ_H                 (0.00185f)
#define APP_WEAKENING_MAX              (0.80f)
#define APP_VOLTAGE_MARGIN             (0.90f)
#define APP_LOW_SPEED_ID_A             (0.0f)
#define APP_DEFAULT_ANGLE_OFFSET_DEG   (0.0f)
#define APP_DEG_TO_RAD                 (3.14159265358979323846f / 180.0f)

typedef enum
{
    APP_STATE_ALIGN = 0,
    APP_STATE_RUN_OPEN_LOOP
} App_State_t;

static App_State_t s_app_state;
static float s_app_electrical_angle;
static uint8_t s_app_voltage_lock_enabled;
static uint8_t s_app_current_loop_enabled;
static uint8_t s_app_encoder_voltage_enabled;
static uint8_t s_app_current_feedforward_enabled;
static float s_app_voltage_lock_angle;
static float s_app_voltage_lock_ud;
static float s_app_voltage_lock_uq;
static float s_app_encoder_voltage_uq;
static float s_app_encoder_voltage_lead_rad;
static float s_app_angle_offset_rad;
static float s_app_voltage_lead_rad;
static float s_app_speed_id_ref;
static uint32_t s_app_align_tick;
static uint16_t s_app_speed_tick_div;
static uint8_t s_app_fault_overspeed;

static float App_AbsFloat(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

static float App_GetControlElectricalAngle(const Encoder_State_t *encoder)
{
    if (encoder == 0)
    {
        return s_app_angle_offset_rad;
    }

    return encoder->electrical_angle_rad + s_app_angle_offset_rad;
}

static float App_LimitFloat(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

static void App_AllocateSpeedCurrent(float iq_request,
                                     float speed_rpm,
                                     float voltage_limit,
                                     float psi_wb,
                                     float ld_h,
                                     float lq_h,
                                     float *id_target,
                                     float *iq_target)
{
    float omega_e;
    float omega_abs;
    float v_available;
    float flux_min;
    float id_min;
    float voltage_per_speed_sq;
    float lq_iq_sq;
    float ld_id_psi_sq;
    float id;
    float iq;

    if ((id_target == 0) || (iq_target == 0))
    {
        return;
    }

    omega_e = speed_rpm * APP_POLE_PAIRS * (2.0f * 3.14159265358979323846f / 60.0f);
    omega_abs = App_AbsFloat(omega_e);
    if ((psi_wb <= 0.0f) || (ld_h <= 0.0f) || (lq_h <= 0.0f))
    {
        *id_target = s_app_speed_id_ref;
        *iq_target = iq_request;
        return;
    }

    id_min = -(psi_wb * APP_WEAKENING_MAX) / ld_h;

    if (omega_abs <= 1.0e-6f)
    {
        *id_target = APP_LOW_SPEED_ID_A + s_app_speed_id_ref;
        *iq_target = iq_request;
        return;
    }

    v_available = APP_BUS_VOLTAGE_V * voltage_limit * APP_VOLTAGE_MARGIN;
    voltage_per_speed_sq = (v_available / omega_abs) * (v_available / omega_abs);
    lq_iq_sq = (lq_h * iq_request) * (lq_h * iq_request);
    flux_min = (1.0f - APP_WEAKENING_MAX) * psi_wb;

    if ((flux_min * flux_min) <= (voltage_per_speed_sq - lq_iq_sq))
    {
        iq = iq_request;
        ld_id_psi_sq = voltage_per_speed_sq - lq_iq_sq;
        if (ld_id_psi_sq < 0.0f)
        {
            ld_id_psi_sq = 0.0f;
        }

        id = (sqrtf(ld_id_psi_sq) - psi_wb) / ld_h;
        id = App_LimitFloat(id, id_min, APP_LOW_SPEED_ID_A);
    }
    else
    {
        float lq_iq_available_sq;

        id = id_min;
        lq_iq_available_sq =
            voltage_per_speed_sq -
            (((ld_h * id_min) + psi_wb) *
             ((ld_h * id_min) + psi_wb));

        if (lq_iq_available_sq > 0.0f)
        {
            iq = sqrtf(lq_iq_available_sq) / lq_h;
            if (iq_request < 0.0f)
            {
                iq = -iq;
            }
        }
        else
        {
            iq = 0.0f;
        }

        if (iq_request >= 0.0f)
        {
            iq = App_LimitFloat(iq_request, 0.0f, iq);
        }
        else
        {
            iq = App_LimitFloat(iq_request, iq, 0.0f);
        }
    }

    *id_target = id + s_app_speed_id_ref;
    *iq_target = iq;
}

static void App_EnterOverspeedFault(void)
{
    s_app_fault_overspeed = 1U;
    s_app_voltage_lock_enabled = 0U;
    s_app_current_loop_enabled = 0U;
    MotionCtrl_SetMode(APP_RUN_MODE_OPEN_LOOP);
    MotionCtrl_SetOpenLoop(0.0f, 0.0f);
    FOC_SetCurrentTarget(0.0f, 0.0f);
    FOC_ResetCurrentLoop();
    FocV_Stop();
}

void App_Init(void)
{
    s_app_electrical_angle = 0.0f;
    s_app_voltage_lock_enabled = 0U;
    s_app_current_loop_enabled = 0U;
    s_app_encoder_voltage_enabled = 0U;
    s_app_current_feedforward_enabled = 0U;
    s_app_voltage_lock_angle = 0.0f;
    s_app_voltage_lock_ud = 0.0f;
    s_app_voltage_lock_uq = 0.0f;
    s_app_encoder_voltage_uq = 0.0f;
    s_app_encoder_voltage_lead_rad = 0.0f;
    s_app_angle_offset_rad = APP_DEFAULT_ANGLE_OFFSET_DEG * APP_DEG_TO_RAD;
    s_app_voltage_lead_rad = 0.0f;
    s_app_speed_id_ref = 0.0f;
    s_app_align_tick = 0U;
    s_app_speed_tick_div = 0U;
    s_app_fault_overspeed = 0U;
    s_app_state = APP_STATE_ALIGN;

    FocV_Init();
    FOC_Init();
    FOC_SetMotorModel(APP_MODEL_PSI_WB, APP_MODEL_LD_H, APP_MODEL_LQ_H);
    Current_Init();
    Current_CalibrateOffset(256U);
    Current_StartSampling();
    Encoder_Init();
    MotionCtrl_Init();
    Debug_Init();
    MotorPWM_Init();
}

void App_Loop(void)
{
    Debug_Loop();
    Debug_TelemetryLoop();
}

void App_FastLoop(void)
{
    float motion_dt_s = APP_FAST_LOOP_DT_S;
    uint8_t speed_update_due = 0U;
    const MotionCtrl_Output_t *ctrl_output;
    const Encoder_State_t *encoder;
    const Current_State_t *current;
    float control_angle;

    Encoder_Update();
    Current_Update();

    if (s_app_fault_overspeed != 0U)
    {
        FocV_Stop();
        return;
    }

    if (s_app_state == APP_STATE_ALIGN)
    {
        s_app_electrical_angle = APP_ALIGN_ANGLE_RAD;
        FocV_SetVoltage(APP_ALIGN_UD, 0.0f);
        FocV_SetElectricalAngle(APP_ALIGN_ANGLE_RAD);
        FocV_Run();

        s_app_align_tick++;
        if (s_app_align_tick >= APP_ALIGN_TICKS)
        {
            Encoder_SetZeroCurrentPosition();
            s_app_electrical_angle = 0.0f;
            s_app_speed_tick_div = 0U;
            s_app_state = APP_STATE_RUN_OPEN_LOOP;
        }

        return;
    }

    s_app_speed_tick_div++;
    if (s_app_speed_tick_div >= APP_SPEED_LOOP_DIV)
    {
        Encoder_UpdateSpeed(APP_FAST_LOOP_DT_S * (float)APP_SPEED_LOOP_DIV);
        s_app_speed_tick_div = 0U;
        motion_dt_s = APP_FAST_LOOP_DT_S * (float)APP_SPEED_LOOP_DIV;
        speed_update_due = 1U;

        if (App_AbsFloat(Encoder_GetState()->speed_lpf_rpm) >= APP_OVERSPEED_PROTECT_RPM)
        {
            App_EnterOverspeedFault();
            return;
        }
    }

    encoder = Encoder_GetState();
    current = Current_GetState();
    control_angle = App_GetControlElectricalAngle(encoder);

    if (s_app_voltage_lock_enabled != 0U)
    {
        FOC_SetFeedforwardSpeed(0.0f);
        FOC_UpdateCurrent(current->ia, current->ib, s_app_voltage_lock_angle);
        FocV_SetVoltage(s_app_voltage_lock_ud, s_app_voltage_lock_uq);
        FocV_SetElectricalAngle(s_app_voltage_lock_angle);
        FocV_Run();
        return;
    }

    FOC_UpdateCurrent(current->ia, current->ib, control_angle);

    if (s_app_encoder_voltage_enabled != 0U)
    {
        FOC_SetFeedforwardSpeed(0.0f);
        FocV_SetVoltage(0.0f, s_app_encoder_voltage_uq);
        FocV_SetElectricalAngle(control_angle + s_app_encoder_voltage_lead_rad);
        FocV_Run();
        return;
    }

    if (s_app_current_loop_enabled != 0U)
    {
        const FOC_State_t *foc_current;

        if (s_app_current_feedforward_enabled != 0U)
        {
            FOC_SetFeedforwardSpeed(encoder->speed_lpf_rpm);
        }
        else
        {
            FOC_SetFeedforwardSpeed(0.0f);
        }
        FOC_RunCurrentLoop(APP_FAST_LOOP_DT_S);
        foc_current = FOC_GetState();
        FocV_SetVoltage(foc_current->ud, foc_current->uq);
        FocV_SetElectricalAngle(control_angle + s_app_voltage_lead_rad);
        FocV_Run();
        return;
    }

    if (MotionCtrl_GetMode() == APP_RUN_MODE_SPEED_PI)
    {
        const FOC_State_t *foc_current;
        float id_target;
        float iq_target;

        if (speed_update_due != 0U)
        {
            MotionCtrl_Update(motion_dt_s, &s_app_electrical_angle, encoder);
        }

        ctrl_output = MotionCtrl_GetOutput();
        foc_current = FOC_GetState();
        App_AllocateSpeedCurrent(ctrl_output->uq,
                                 encoder->speed_lpf_rpm,
                                 foc_current->voltage_limit,
                                 foc_current->psi_wb,
                                 foc_current->ld_h,
                                 foc_current->lq_h,
                                 &id_target,
                                 &iq_target);
        FOC_SetCurrentTarget(id_target, iq_target);
        if (s_app_current_feedforward_enabled != 0U)
        {
            FOC_SetFeedforwardSpeed(encoder->speed_lpf_rpm);
        }
        else
        {
            FOC_SetFeedforwardSpeed(0.0f);
        }
        FOC_RunCurrentLoop(APP_FAST_LOOP_DT_S);
        foc_current = FOC_GetState();
        FocV_SetVoltage(foc_current->ud, foc_current->uq);
        FocV_SetElectricalAngle(control_angle + s_app_voltage_lead_rad);
        FocV_Run();
        Debug_SpeedTraceSample();
        return;
    }

    MotionCtrl_Update(motion_dt_s, &s_app_electrical_angle, encoder);
    ctrl_output = MotionCtrl_GetOutput();
    FocV_RunUq(ctrl_output->uq, ctrl_output->angle_rad);
}

void App_ClearFault(void)
{
    s_app_fault_overspeed = 0U;
}

uint8_t App_IsFaulted(void)
{
    return s_app_fault_overspeed;
}

void App_SetOpenLoop(float uq, float elec_freq_hz)
{
    App_ClearVoltageLock();
    App_ClearCurrentLoop();
    App_ClearEncoderVoltage();
    MotionCtrl_SetMode(APP_RUN_MODE_OPEN_LOOP);
    MotionCtrl_SetOpenLoop(uq, elec_freq_hz);
}

void App_SetEncoderVoltage(float uq_volt, float lead_deg)
{
    App_ClearVoltageLock();
    App_ClearCurrentLoop();
    MotionCtrl_SetMode(APP_RUN_MODE_OPEN_LOOP);
    MotionCtrl_SetOpenLoop(0.0f, 0.0f);
    s_app_encoder_voltage_uq = uq_volt / APP_BUS_VOLTAGE_V;
    s_app_encoder_voltage_lead_rad = lead_deg * (3.14159265358979323846f / 180.0f);
    s_app_encoder_voltage_enabled = 1U;
}

void App_ClearEncoderVoltage(void)
{
    s_app_encoder_voltage_enabled = 0U;
    s_app_encoder_voltage_uq = 0.0f;
    s_app_encoder_voltage_lead_rad = 0.0f;
}

void App_SetPositionP(float speed_cps, float kp, float max_uq)
{
    App_ClearVoltageLock();
    App_ClearCurrentLoop();
    App_ClearEncoderVoltage();
    MotionCtrl_SetPositionP(speed_cps, kp, max_uq);
}

void App_SetSpeedPI(float target_rpm, float kp, float ki, float max_iq, float max_integral_iq)
{
    App_ClearVoltageLock();
    App_ClearCurrentLoop();
    App_ClearEncoderVoltage();
    s_app_speed_id_ref = 0.0f;
    MotionCtrl_SetSpeedPI(target_rpm, kp, ki, max_iq, max_integral_iq);
}

void App_SetSpeedIdRef(float id_ref)
{
    s_app_speed_id_ref = id_ref;
}

void App_SetAngleOffsetDeg(float offset_deg)
{
    s_app_angle_offset_rad = offset_deg * APP_DEG_TO_RAD;
    FOC_ResetCurrentLoop();
}

float App_GetAngleOffsetDeg(void)
{
    return s_app_angle_offset_rad / APP_DEG_TO_RAD;
}

void App_SetVoltageLeadDeg(float lead_deg)
{
    s_app_voltage_lead_rad = lead_deg * APP_DEG_TO_RAD;
    FOC_ResetCurrentLoop();
}

float App_GetVoltageLeadDeg(void)
{
    return s_app_voltage_lead_rad / APP_DEG_TO_RAD;
}

void App_SetCurrentFeedforward(uint8_t enabled)
{
    s_app_current_feedforward_enabled = (enabled != 0U) ? 1U : 0U;
    FOC_SetFeedforwardSpeed(0.0f);
    FOC_ResetCurrentLoop();
}

uint8_t App_GetCurrentFeedforward(void)
{
    return s_app_current_feedforward_enabled;
}

void App_SetRunMode(App_RunMode_t mode)
{
    App_ClearVoltageLock();
    App_ClearCurrentLoop();
    App_ClearEncoderVoltage();
    MotionCtrl_SetMode(mode);
}

void App_SetVoltageLock(float angle_rad, float ud, float uq)
{
    MotionCtrl_SetMode(APP_RUN_MODE_OPEN_LOOP);
    MotionCtrl_SetOpenLoop(0.0f, 0.0f);
    App_ClearEncoderVoltage();
    s_app_voltage_lock_angle = angle_rad;
    s_app_voltage_lock_ud = ud;
    s_app_voltage_lock_uq = uq;
    s_app_voltage_lock_enabled = 1U;
    s_app_current_loop_enabled = 0U;
}

void App_ClearVoltageLock(void)
{
    s_app_voltage_lock_enabled = 0U;
    s_app_voltage_lock_angle = 0.0f;
    s_app_voltage_lock_ud = 0.0f;
    s_app_voltage_lock_uq = 0.0f;
}

void App_SetCurrentLoop(float id_ref, float iq_ref)
{
    App_ClearVoltageLock();
    App_ClearEncoderVoltage();
    MotionCtrl_SetMode(APP_RUN_MODE_OPEN_LOOP);
    MotionCtrl_SetOpenLoop(0.0f, 0.0f);
    FOC_SetCurrentTarget(id_ref, iq_ref);
    FOC_ResetCurrentLoop();
    s_app_current_loop_enabled = 1U;
}

void App_SetCurrentLoopGains(float kp, float ki, float voltage_limit)
{
    FOC_SetCurrentLoopGains(kp, ki);
    FOC_SetCurrentLoopLimit(voltage_limit);
}

void App_SetCurrentLoopBandwidth(float bandwidth_hz, float voltage_limit)
{
    FOC_SetCurrentLoopBandwidth(bandwidth_hz);
    FOC_SetCurrentLoopLimit(voltage_limit);
}

void App_ClearCurrentLoop(void)
{
    s_app_current_loop_enabled = 0U;
    App_ClearEncoderVoltage();
    FOC_SetCurrentTarget(0.0f, 0.0f);
    FOC_ResetCurrentLoop();
}

const App_OpenLoopConfig_t *App_GetOpenLoopConfig(void)
{
    return MotionCtrl_GetOpenLoopConfig();
}

const App_PositionPState_t *App_GetPositionPState(void)
{
    return MotionCtrl_GetPositionPState();
}

const App_SpeedPIState_t *App_GetSpeedPIState(void)
{
    return MotionCtrl_GetSpeedPIState();
}

App_RunMode_t App_GetRunMode(void)
{
    return MotionCtrl_GetMode();
}
