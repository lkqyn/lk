#include "phase_current_loop.h"

#include "current_controller.h"
#include "current_sense.h"
#include "motor_pwm.h"
#include "stm32f4xx_hal.h"

typedef struct
{
    CurrentController_t controller;
    volatile PhaseCurrentLoop_State_t state;
    int32_t maximum_target_current_ma;
    int32_t overcurrent_limit_ma;
    uint32_t bus_voltage_mv;
} PhaseCurrentLoop_Context_t;

static PhaseCurrentLoop_Context_t s_phase_current_loop;

static int32_t PhaseCurrentLoop_Absolute(int32_t value)
{
    return (value < 0L) ? -value : value;
}

static void PhaseCurrentLoop_SampleCallback(void *context)
{
    PhaseCurrentLoop_Context_t *loop = (PhaseCurrentLoop_Context_t *)context;
    const CurrentSense_State_t *current = CurrentSense_GetState();
    int32_t abs_current_a = PhaseCurrentLoop_Absolute(current->current_a_ma);
    int32_t abs_current_b = PhaseCurrentLoop_Absolute(current->current_b_ma);
    int32_t abs_output_a;
    int32_t abs_output_b;

    if (abs_current_a > loop->state.peak_abs_current_a_ma)
    {
        loop->state.peak_abs_current_a_ma = abs_current_a;
    }
    if (abs_current_b > loop->state.peak_abs_current_b_ma)
    {
        loop->state.peak_abs_current_b_ma = abs_current_b;
    }

    if ((abs_current_a >= loop->overcurrent_limit_ma) ||
        (abs_current_b >= loop->overcurrent_limit_ma))
    {
        loop->state.fault = PHASE_CURRENT_LOOP_FAULT_OVERCURRENT;
        MotorPWM_EnterBrakeState();
        return;
    }

    CurrentController_Step(&loop->controller,
                           loop->state.target_a_ma,
                           loop->state.target_b_ma,
                           current->current_a_ma,
                           current->current_b_ma);
    MotorPWM_SetPhaseVoltagesMv(loop->controller.output_axis_1_mv,
                                loop->controller.output_axis_2_mv,
                                loop->bus_voltage_mv);

    loop->state.output_a_mv = loop->controller.output_axis_1_mv;
    loop->state.output_b_mv = loop->controller.output_axis_2_mv;
    loop->state.sample_count++;
    abs_output_a = PhaseCurrentLoop_Absolute(loop->state.output_a_mv);
    abs_output_b = PhaseCurrentLoop_Absolute(loop->state.output_b_mv);
    if (abs_output_a > loop->state.peak_abs_output_a_mv)
    {
        loop->state.peak_abs_output_a_mv = abs_output_a;
    }
    if (abs_output_b > loop->state.peak_abs_output_b_mv)
    {
        loop->state.peak_abs_output_b_mv = abs_output_b;
    }
    if (loop->controller.saturated != 0U)
    {
        loop->state.saturated_sample_count++;
    }
}

uint8_t PhaseCurrentLoop_Start(const PhaseCurrentLoop_Config_t *config)
{
    const CurrentSense_State_t *current = CurrentSense_GetState();

    if ((config == 0) || (s_phase_current_loop.state.running != 0U) ||
        (current->offset_valid == 0U) ||
        (config->maximum_target_current_ma <= 0L) ||
        (config->overcurrent_limit_ma <= config->maximum_target_current_ma) ||
        (config->bus_voltage_mv == 0U))
    {
        return 0U;
    }

    if (CurrentController_Init(&s_phase_current_loop.controller,
                               config->bandwidth_hz,
                               config->sample_frequency_hz,
                               config->maximum_voltage_mv) == 0U)
    {
        return 0U;
    }

    s_phase_current_loop.maximum_target_current_ma =
        config->maximum_target_current_ma;
    s_phase_current_loop.overcurrent_limit_ma = config->overcurrent_limit_ma;
    s_phase_current_loop.bus_voltage_mv = config->bus_voltage_mv;
    s_phase_current_loop.state.target_a_ma = 0L;
    s_phase_current_loop.state.target_b_ma = 0L;
    s_phase_current_loop.state.output_a_mv = 0L;
    s_phase_current_loop.state.output_b_mv = 0L;
    s_phase_current_loop.state.peak_abs_current_a_ma = 0L;
    s_phase_current_loop.state.peak_abs_current_b_ma = 0L;
    s_phase_current_loop.state.peak_abs_output_a_mv = 0L;
    s_phase_current_loop.state.peak_abs_output_b_mv = 0L;
    s_phase_current_loop.state.sample_count = 0U;
    s_phase_current_loop.state.saturated_sample_count = 0U;
    s_phase_current_loop.state.fault = PHASE_CURRENT_LOOP_FAULT_NONE;

    CurrentSense_SetSampleCallback(PhaseCurrentLoop_SampleCallback,
                                   &s_phase_current_loop);
    if (CurrentSense_StartSynchronizedSampling() == 0U)
    {
        CurrentSense_SetSampleCallback(0, 0);
        return 0U;
    }

    s_phase_current_loop.state.running = 1U;
    MotorPWM_StartNeutral();
    return 1U;
}

uint8_t PhaseCurrentLoop_SetTargets(int32_t target_a_ma,
                                    int32_t target_b_ma)
{
    uint32_t interrupt_mask;

    if ((s_phase_current_loop.state.running == 0U) ||
        (PhaseCurrentLoop_Absolute(target_a_ma) >
         s_phase_current_loop.maximum_target_current_ma) ||
        (PhaseCurrentLoop_Absolute(target_b_ma) >
         s_phase_current_loop.maximum_target_current_ma))
    {
        return 0U;
    }

    /* 保证ISR看到的是同一时刻的A/B电流矢量，避免只更新一相的中间状态。 */
    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_phase_current_loop.state.target_a_ma = target_a_ma;
    s_phase_current_loop.state.target_b_ma = target_b_ma;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

void PhaseCurrentLoop_Stop(void)
{
    if (s_phase_current_loop.state.running == 0U)
    {
        return;
    }

    MotorPWM_EnterBrakeState();
    CurrentSense_StopSynchronizedSampling();
    CurrentSense_SetSampleCallback(0, 0);
    s_phase_current_loop.state.target_a_ma = 0L;
    s_phase_current_loop.state.target_b_ma = 0L;
    s_phase_current_loop.state.output_a_mv = 0L;
    s_phase_current_loop.state.output_b_mv = 0L;
    s_phase_current_loop.state.running = 0U;
}

const volatile PhaseCurrentLoop_State_t *PhaseCurrentLoop_GetState(void)
{
    return &s_phase_current_loop.state;
}
