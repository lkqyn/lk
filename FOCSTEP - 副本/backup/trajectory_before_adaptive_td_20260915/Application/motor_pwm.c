#include "motor_pwm.h"

#include "tim.h"

static uint8_t s_pwm_enabled;

static uint16_t MotorPWM_GetNeutralCompare(void)
{
    return (uint16_t)((__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U) / 2U);
}

static void MotorPWM_ClearCompareRegisters(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
}

static int32_t MotorPWM_ClampCompare(int32_t compare)
{
    int32_t maximum_compare = (int32_t)__HAL_TIM_GET_AUTORELOAD(&htim1);

    if (compare < 0L)
    {
        return 0L;
    }
    if (compare > maximum_compare)
    {
        return maximum_compare;
    }
    return compare;
}

static int16_t MotorPWM_VoltageToDifferentialCounts(int32_t voltage_mv,
                                                     uint32_t bus_voltage_mv)
{
    int64_t numerator;
    int64_t period_counts;
    int64_t differential_counts;

    if (bus_voltage_mv == 0U)
    {
        return 0;
    }

    period_counts = (int64_t)__HAL_TIM_GET_AUTORELOAD(&htim1) + 1LL;
    numerator = (int64_t)voltage_mv * period_counts;
    numerator += (numerator >= 0LL) ? ((int64_t)bus_voltage_mv / 2LL) :
                                      -((int64_t)bus_voltage_mv / 2LL);
    differential_counts = numerator / (int64_t)bus_voltage_mv;

    if (differential_counts > INT16_MAX)
    {
        differential_counts = INT16_MAX;
    }
    else if (differential_counts < INT16_MIN)
    {
        differential_counts = INT16_MIN;
    }
    return (int16_t)differential_counts;
}

void MotorPWM_Init(void)
{
    /* 上电默认进入制动状态，避免参数未确认时电机意外动作。 */
    MotorPWM_EnterBrakeState();
}

void MotorPWM_EnterBrakeState(void)
{
    /*
     * EG2131的LIN为低电平有效，而本板HIN与LIN共用一路PWM。
     * 因此比较值清零后四个低端MOS管导通，电机进入低端短路制动，
     * 并非功率管高阻关断。这是当前硬件的既定停机方式。
     */
    /* 先关闭高级定时器总输出，避免逐通道停止时产生差分电压。 */
    __HAL_TIM_MOE_DISABLE(&htim1);

    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    (void)HAL_TIM_Base_Stop_IT(&htim1);
    MotorPWM_ClearCompareRegisters();

    s_pwm_enabled = 0U;
}

void MotorPWM_StartNeutral(void)
{
    uint16_t neutral_compare = MotorPWM_GetNeutralCompare();

    /* 先屏蔽总输出，装载四路中性占空比后再同时开启。 */
    __HAL_TIM_MOE_DISABLE(&htim1);
    __HAL_TIM_DISABLE(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, neutral_compare);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, neutral_compare);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, neutral_compare);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, neutral_compare);

    htim1.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    htim1.Instance->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E |
                            TIM_CCER_CC3E | TIM_CCER_CC4E;

    __HAL_TIM_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim1);
    s_pwm_enabled = 1U;
}

void MotorPWM_SetPhaseDifferentialCounts(MotorPWM_Phase_t phase,
                                         int16_t differential_counts)
{
    int32_t neutral = (int32_t)MotorPWM_GetNeutralCompare();
    int32_t positive_compare = neutral + ((int32_t)differential_counts / 2L);
    int32_t negative_compare = positive_compare - (int32_t)differential_counts;
    int32_t maximum_compare = (int32_t)__HAL_TIM_GET_AUTORELOAD(&htim1);

    if (positive_compare < 0L)
    {
        positive_compare = 0L;
    }
    else if (positive_compare > maximum_compare)
    {
        positive_compare = maximum_compare;
    }

    if (negative_compare < 0L)
    {
        negative_compare = 0L;
    }
    else if (negative_compare > maximum_compare)
    {
        negative_compare = maximum_compare;
    }

    /* 非目标相恢复为50%，避免上一次调试值残留。 */
    if (phase == MOTOR_PWM_PHASE_A)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)positive_compare);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)negative_compare);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)neutral);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)neutral);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)neutral);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)neutral);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)positive_compare);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)negative_compare);
    }
}

void MotorPWM_SetPhaseVoltagesMv(int32_t voltage_a_mv,
                                 int32_t voltage_b_mv,
                                 uint32_t bus_voltage_mv)
{
    int32_t neutral = (int32_t)MotorPWM_GetNeutralCompare();
    int32_t differential_a = (int32_t)MotorPWM_VoltageToDifferentialCounts(
        voltage_a_mv, bus_voltage_mv);
    int32_t differential_b = (int32_t)MotorPWM_VoltageToDifferentialCounts(
        voltage_b_mv, bus_voltage_mv);
    int32_t positive_a = neutral + (differential_a / 2L);
    int32_t negative_a = positive_a - differential_a;
    int32_t positive_b = neutral + (differential_b / 2L);
    int32_t negative_b = positive_b - differential_b;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          (uint32_t)MotorPWM_ClampCompare(positive_a));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                          (uint32_t)MotorPWM_ClampCompare(negative_a));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                          (uint32_t)MotorPWM_ClampCompare(positive_b));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
                          (uint32_t)MotorPWM_ClampCompare(negative_b));
}

uint8_t MotorPWM_IsEnabled(void)
{
    return s_pwm_enabled;
}
