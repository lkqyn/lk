#include "foc.h"

#include <math.h>

#define FOC_PI                  (3.14159265358979323846f)
#define FOC_MOTOR_R_OHM         (0.45f)
#define FOC_MOTOR_L_H           (0.00185f)
#define FOC_MOTOR_PSI_WB        (0.0120f)
#define FOC_MOTOR_LD_H          (0.00185f)
#define FOC_MOTOR_LQ_H          (0.00185f)
#define FOC_POLE_PAIRS          (50.0f)
#define FOC_BUS_VOLTAGE_V       (24.0f)
#define FOC_DEFAULT_BANDWIDTH_HZ (1200.0f)
#define FOC_DEFAULT_KP          ((2.0f * FOC_PI * FOC_MOTOR_L_H * FOC_DEFAULT_BANDWIDTH_HZ) / FOC_BUS_VOLTAGE_V)
#define FOC_DEFAULT_KI          ((2.0f * FOC_PI * FOC_MOTOR_R_OHM * FOC_DEFAULT_BANDWIDTH_HZ) / FOC_BUS_VOLTAGE_V)
#define FOC_DEFAULT_VOLT_LIMIT  (0.9500f)
#define FOC_DEFAULT_FF_SCALE    (0.25f)

static FOC_State_t s_foc;

static float FOC_Limit(float value, float min_value, float max_value)
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

static float FOC_LimitVoltageVector(float *ud, float *uq, float limit)
{
    float mag_sq = ((*ud) * (*ud)) + ((*uq) * (*uq));
    float limit_sq = limit * limit;
    float scale;

    if ((limit <= 0.0f) || (mag_sq <= limit_sq))
    {
        return 1.0f;
    }

    scale = limit / sqrtf(mag_sq);
    *ud *= scale;
    *uq *= scale;
    return scale;
}

void FOC_Init(void)
{
    s_foc.ia = 0.0f;
    s_foc.ib = 0.0f;
    s_foc.electrical_angle = 0.0f;
    s_foc.id = 0.0f;
    s_foc.iq = 0.0f;
    s_foc.id_ref = 0.0f;
    s_foc.iq_ref = 0.0f;
    s_foc.id_error = 0.0f;
    s_foc.iq_error = 0.0f;
    s_foc.id_integral = 0.0f;
    s_foc.iq_integral = 0.0f;
    s_foc.kp = FOC_DEFAULT_KP;
    s_foc.ki = FOC_DEFAULT_KI;
    s_foc.voltage_limit = FOC_DEFAULT_VOLT_LIMIT;
    s_foc.feedforward_speed_rpm = 0.0f;
    s_foc.feedforward_scale = FOC_DEFAULT_FF_SCALE;
    s_foc.psi_wb = FOC_MOTOR_PSI_WB;
    s_foc.ld_h = FOC_MOTOR_LD_H;
    s_foc.lq_h = FOC_MOTOR_LQ_H;
    s_foc.ud_ff = 0.0f;
    s_foc.uq_ff = 0.0f;
    s_foc.ud = 0.0f;
    s_foc.uq = 0.0f;
}

void FOC_UpdateCurrent(float ia, float ib, float electrical_angle)
{
    float sin_angle = sinf(electrical_angle);
    float cos_angle = cosf(electrical_angle);

    s_foc.ia = ia;
    s_foc.ib = ib;
    s_foc.electrical_angle = electrical_angle;

    s_foc.id = (ia * cos_angle) + (ib * sin_angle);
    s_foc.iq = (-ia * sin_angle) + (ib * cos_angle);
}

void FOC_SetCurrentLoopGains(float kp, float ki)
{
    if (kp < 0.0f)
    {
        kp = 0.0f;
    }

    if (ki < 0.0f)
    {
        ki = 0.0f;
    }

    s_foc.kp = kp;
    s_foc.ki = ki;
}

void FOC_SetCurrentLoopBandwidth(float bandwidth_hz)
{
    if (bandwidth_hz < 0.0f)
    {
        bandwidth_hz = 0.0f;
    }

    s_foc.kp = (2.0f * FOC_PI * FOC_MOTOR_L_H * bandwidth_hz) / FOC_BUS_VOLTAGE_V;
    s_foc.ki = (2.0f * FOC_PI * FOC_MOTOR_R_OHM * bandwidth_hz) / FOC_BUS_VOLTAGE_V;
}

void FOC_SetCurrentLoopLimit(float voltage_limit)
{
    if (voltage_limit < 0.0f)
    {
        voltage_limit = -voltage_limit;
    }

    if (voltage_limit > 1.0f)
    {
        voltage_limit = 1.0f;
    }

    s_foc.voltage_limit = voltage_limit;
    s_foc.id_integral = FOC_Limit(s_foc.id_integral, -voltage_limit, voltage_limit);
    s_foc.iq_integral = FOC_Limit(s_foc.iq_integral, -voltage_limit, voltage_limit);
}

void FOC_SetMotorModel(float psi_wb, float ld_h, float lq_h)
{
    if (psi_wb < 0.0f)
    {
        psi_wb = 0.0f;
    }

    if (ld_h < 0.0f)
    {
        ld_h = 0.0f;
    }

    if (lq_h < 0.0f)
    {
        lq_h = 0.0f;
    }

    s_foc.psi_wb = psi_wb;
    s_foc.ld_h = ld_h;
    s_foc.lq_h = lq_h;
}

void FOC_SetFeedforwardSpeed(float speed_rpm)
{
    s_foc.feedforward_speed_rpm = speed_rpm;
}

void FOC_SetFeedforwardScale(float scale)
{
    if (scale < 0.0f)
    {
        scale = 0.0f;
    }

    if (scale > 1.0f)
    {
        scale = 1.0f;
    }

    s_foc.feedforward_scale = scale;
}

void FOC_SetCurrentTarget(float id_ref, float iq_ref)
{
    s_foc.id_ref = id_ref;
    s_foc.iq_ref = iq_ref;
}

void FOC_ResetCurrentLoop(void)
{
    s_foc.id_error = 0.0f;
    s_foc.iq_error = 0.0f;
    s_foc.id_integral = 0.0f;
    s_foc.iq_integral = 0.0f;
    s_foc.ud_ff = 0.0f;
    s_foc.uq_ff = 0.0f;
    s_foc.ud = 0.0f;
    s_foc.uq = 0.0f;
}

void FOC_RunCurrentLoop(float dt_s)
{
    float ud_unclamped;
    float uq_unclamped;
    float ud_out;
    float uq_out;
    float proportional_d;
    float proportional_q;
    float scale;

    if (dt_s < 0.0f)
    {
        dt_s = 0.0f;
    }

    s_foc.id_error = s_foc.id_ref - s_foc.id;
    s_foc.iq_error = s_foc.iq_ref - s_foc.iq;

    proportional_d = s_foc.kp * s_foc.id_error;
    proportional_q = s_foc.kp * s_foc.iq_error;

    s_foc.id_integral += s_foc.ki * s_foc.id_error * dt_s;
    s_foc.iq_integral += s_foc.ki * s_foc.iq_error * dt_s;
    s_foc.id_integral = FOC_Limit(s_foc.id_integral, -s_foc.voltage_limit, s_foc.voltage_limit);
    s_foc.iq_integral = FOC_Limit(s_foc.iq_integral, -s_foc.voltage_limit, s_foc.voltage_limit);

    {
        float omega_e = s_foc.feedforward_speed_rpm * FOC_POLE_PAIRS * (2.0f * FOC_PI / 60.0f);

        s_foc.ud_ff = s_foc.feedforward_scale *
                      (-(omega_e * s_foc.lq_h * s_foc.iq_ref) / FOC_BUS_VOLTAGE_V);
        s_foc.uq_ff = s_foc.feedforward_scale *
                      ((omega_e * ((s_foc.ld_h * s_foc.id_ref) + s_foc.psi_wb)) / FOC_BUS_VOLTAGE_V);
    }

    ud_unclamped = proportional_d + s_foc.id_integral + s_foc.ud_ff;
    uq_unclamped = proportional_q + s_foc.iq_integral + s_foc.uq_ff;
    ud_out = ud_unclamped;
    uq_out = uq_unclamped;

    scale = FOC_LimitVoltageVector(&ud_out, &uq_out, s_foc.voltage_limit);
    if ((scale < 1.0f) && (s_foc.ki > 0.0f))
    {
        s_foc.id_integral = ud_out - proportional_d - s_foc.ud_ff;
        s_foc.iq_integral = uq_out - proportional_q - s_foc.uq_ff;
        s_foc.id_integral = FOC_Limit(s_foc.id_integral, -s_foc.voltage_limit, s_foc.voltage_limit);
        s_foc.iq_integral = FOC_Limit(s_foc.iq_integral, -s_foc.voltage_limit, s_foc.voltage_limit);
    }

    s_foc.ud = ud_out;
    s_foc.uq = uq_out;
}

const FOC_State_t *FOC_GetState(void)
{
    return &s_foc;
}
