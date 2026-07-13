#include "motor_pwm.h"

#include "tim.h"

static MotorPWM_Duty_t s_motor_pwm_duty;
static uint8_t s_motor_pwm_started;

static uint16_t MotorPWM_LimitDuty(uint16_t duty)
{
    if (duty > MOTOR_PWM_MAX_DUTY)
    {
        return MOTOR_PWM_MAX_DUTY;
    }

    return duty;
}

static float MotorPWM_LimitPhaseVoltage(float voltage)
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

static uint16_t MotorPWM_PhaseVoltageToDuty(float voltage)
{
    float abs_voltage = voltage;

    if (abs_voltage < 0.0f)
    {
        abs_voltage = -abs_voltage;
    }

    return (uint16_t)((abs_voltage * (float)MOTOR_PWM_MAX_DUTY) + 0.5f);
}

static void MotorPWM_WriteCompare(const MotorPWM_Duty_t *duty)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty->ap);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty->an);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty->bp);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty->bn);
}

void MotorPWM_Init(void)
{
    MotorPWM_SetRaw(0U, 0U, 0U, 0U);
    MotorPWM_Enable();
}

void MotorPWM_Enable(void)
{
    if (s_motor_pwm_started != 0U)
    {
        return;
    }

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Base_Start_IT(&htim1);

    s_motor_pwm_started = 1U;
}

void MotorPWM_Disable(void)
{
    MotorPWM_SetRaw(0U, 0U, 0U, 0U);

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Base_Stop_IT(&htim1);

    s_motor_pwm_started = 0U;
}

void MotorPWM_SetRaw(uint16_t ap, uint16_t an, uint16_t bp, uint16_t bn)
{
    s_motor_pwm_duty.ap = MotorPWM_LimitDuty(ap);
    s_motor_pwm_duty.an = MotorPWM_LimitDuty(an);
    s_motor_pwm_duty.bp = MotorPWM_LimitDuty(bp);
    s_motor_pwm_duty.bn = MotorPWM_LimitDuty(bn);

    MotorPWM_WriteCompare(&s_motor_pwm_duty);
}

void MotorPWM_SetPhaseVoltage(float ua, float ub)
{
    float ua_limited = MotorPWM_LimitPhaseVoltage(ua);
    float ub_limited = MotorPWM_LimitPhaseVoltage(ub);
    uint16_t ua_duty = MotorPWM_PhaseVoltageToDuty(ua_limited);
    uint16_t ub_duty = MotorPWM_PhaseVoltageToDuty(ub_limited);

    if (ua_limited >= 0.0f)
    {
        s_motor_pwm_duty.ap = ua_duty;
        s_motor_pwm_duty.an = 0U;
    }
    else
    {
        s_motor_pwm_duty.ap = 0U;
        s_motor_pwm_duty.an = ua_duty;
    }

    if (ub_limited >= 0.0f)
    {
        s_motor_pwm_duty.bp = ub_duty;
        s_motor_pwm_duty.bn = 0U;
    }
    else
    {
        s_motor_pwm_duty.bp = 0U;
        s_motor_pwm_duty.bn = ub_duty;
    }

    MotorPWM_WriteCompare(&s_motor_pwm_duty);
}

MotorPWM_Duty_t MotorPWM_GetDuty(void)
{
    return s_motor_pwm_duty;
}
