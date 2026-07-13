#include "foc_v.h"

#include "motor_pwm.h"
#include <math.h>

#define FOC_V_PI         (3.14159265358979323846f)
#define FOC_V_TWO_PI     (2.0f * FOC_V_PI)

static FocV_State_t s_foc_v;

static float FocV_LimitVoltage(float voltage)
{
    if (voltage > 1.0f)
    {
        return 1.0f;
    }

    if (voltage < -1.0f)
    {
        return -1.0f;
    }

    return voltage;
}

static float FocV_NormalizeAngle(float angle_rad)
{
    while (angle_rad >= FOC_V_TWO_PI)
    {
        angle_rad -= FOC_V_TWO_PI;
    }

    while (angle_rad < 0.0f)
    {
        angle_rad += FOC_V_TWO_PI;
    }

    return angle_rad;
}

void FocV_Init(void)
{
    s_foc_v.ud = 0.0f;
    s_foc_v.uq = 0.0f;
    s_foc_v.electrical_angle = 0.0f;
    s_foc_v.ua = 0.0f;
    s_foc_v.ub = 0.0f;

    MotorPWM_SetPhaseVoltage(0.0f, 0.0f);
}

void FocV_Stop(void)
{
    FocV_SetVoltage(0.0f, 0.0f);
    FocV_Run();
}

void FocV_SetVoltage(float ud, float uq)
{
    s_foc_v.ud = FocV_LimitVoltage(ud);
    s_foc_v.uq = FocV_LimitVoltage(uq);
}

void FocV_SetElectricalAngle(float angle_rad)
{
    s_foc_v.electrical_angle = FocV_NormalizeAngle(angle_rad);
}

void FocV_Run(void)
{
    float sin_angle = sinf(s_foc_v.electrical_angle);
    float cos_angle = cosf(s_foc_v.electrical_angle);

    s_foc_v.ua = (s_foc_v.ud * cos_angle) - (s_foc_v.uq * sin_angle);
    s_foc_v.ub = (s_foc_v.ud * sin_angle) + (s_foc_v.uq * cos_angle);

    MotorPWM_SetPhaseVoltage(s_foc_v.ua, s_foc_v.ub);
}

void FocV_RunUq(float uq, float angle_rad)
{
    FocV_SetVoltage(0.0f, uq);
    FocV_SetElectricalAngle(angle_rad);
    FocV_Run();
}

const FocV_State_t *FocV_GetState(void)
{
    return &s_foc_v;
}
