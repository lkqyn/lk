#include "dq_current_loop.h"

#include "current_controller.h"
#include "current_sense.h"
#include "dq_transform.h"
#include "electrical_angle.h"
#include "encoder.h"
#include "motor_pwm.h"
#include "motor_parameters.h"
#include "stm32f4xx_hal.h"

#define DQ_CURRENT_LOOP_MAX_ANGLE_ADVANCE_DELAY_US (100UL)
/* 24V、20kHz PWM实测最优的反Park电压相位超前，用于补偿采样与更新延迟。 */
#define DQ_CURRENT_LOOP_DEFAULT_ANGLE_ADVANCE_DELAY_US (40UL)
#define DQ_CURRENT_LOOP_US_PER_MINUTE               (60000000LL)
#define DQ_CURRENT_LOOP_PHASE_INDEXES                (1024L)
#define DQ_CURRENT_LOOP_D_PRIORITY_ERROR_THRESHOLD_MA (50L)
/* 超过32个相位索引即超过11.25°电角度，用于高速整数角度离散诊断。 */
#define DQ_CURRENT_LOOP_OBSERVER_ERROR_THRESHOLD_INDEX (32L)

typedef struct
{
    CurrentController_t controller;
    volatile DqCurrentLoop_State_t state;
    int32_t maximum_target_current_ma;
    int32_t overcurrent_limit_ma;
    int32_t maximum_position_deviation_count;
    int32_t start_position_count;
    uint32_t bus_voltage_mv;
    uint32_t angle_advance_delay_us;
    int32_t previous_raw_phase_index;
    uint8_t previous_raw_phase_valid;
    DqCurrentLoop_SampleObserver_t sample_observer;
    void *sample_observer_context;
} DqCurrentLoop_Context_t;

static DqCurrentLoop_Context_t s_dq_current_loop =
{
    .angle_advance_delay_us = DQ_CURRENT_LOOP_DEFAULT_ANGLE_ADVANCE_DELAY_US
};

static int32_t DqCurrentLoop_Absolute(int32_t value)
{
    return (value < 0L) ? -value : value;
}

static uint8_t DqCurrentLoop_TargetIsValid(int32_t target_d_ma,
                                           int32_t target_q_ma)
{
    int64_t magnitude_squared =
        ((int64_t)target_d_ma * target_d_ma) +
        ((int64_t)target_q_ma * target_q_ma);
    int64_t maximum_squared =
        (int64_t)s_dq_current_loop.maximum_target_current_ma *
        s_dq_current_loop.maximum_target_current_ma;

    return (magnitude_squared <= maximum_squared) ? 1U : 0U;
}

static int32_t DqCurrentLoop_CalculateAngleAdvanceCount(
    int32_t mechanical_speed_mrpm)
{
    int64_t numerator =
        (int64_t)mechanical_speed_mrpm *
        (int64_t)MOTOR_ELECTRICAL_CYCLES_PER_REVOLUTION *
        (int64_t)MOTOR_ENCODER_COUNTS_PER_REVOLUTION *
        (int64_t)s_dq_current_loop.angle_advance_delay_us;
    int64_t denominator =
        DQ_CURRENT_LOOP_US_PER_MINUTE * 1000LL;

    /* mrpm先除以1000得到rpm，随后按电周期数换算到电角度计数。 */
    if (numerator >= 0LL)
    {
        numerator += denominator / 2LL;
    }
    else
    {
        numerator -= denominator / 2LL;
    }
    return (int32_t)(numerator / denominator);
}

static int32_t DqCurrentLoop_WrapElectricalCount(int32_t count)
{
    int32_t wrapped = count % MOTOR_ENCODER_COUNTS_PER_REVOLUTION;

    if (wrapped < 0L)
    {
        wrapped += MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
    }
    return wrapped;
}

static int32_t DqCurrentLoop_ElectricalCountToPhaseIndex(int32_t count)
{
    int64_t numerator =
        (int64_t)count * DQ_CURRENT_LOOP_PHASE_INDEXES;

    numerator += (numerator >= 0LL) ?
        (MOTOR_ENCODER_COUNTS_PER_REVOLUTION / 2L) :
        -(MOTOR_ENCODER_COUNTS_PER_REVOLUTION / 2L);
    return (int32_t)(numerator /
                     MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
}

static int32_t DqCurrentLoop_WrapPhaseIndex(int32_t phase_index)
{
    int32_t wrapped = phase_index % DQ_CURRENT_LOOP_PHASE_INDEXES;

    if (wrapped < 0L)
    {
        wrapped += DQ_CURRENT_LOOP_PHASE_INDEXES;
    }
    return wrapped;
}

static int32_t DqCurrentLoop_WrapPhaseError(int32_t phase_error_index)
{
    int32_t wrapped = DqCurrentLoop_WrapPhaseIndex(phase_error_index);

    if (wrapped >= (DQ_CURRENT_LOOP_PHASE_INDEXES / 2L))
    {
        wrapped -= DQ_CURRENT_LOOP_PHASE_INDEXES;
    }
    return wrapped;
}

static void DqCurrentLoop_SampleCallback(void *context)
{
    DqCurrentLoop_Context_t *loop = (DqCurrentLoop_Context_t *)context;
    const CurrentSense_State_t *current = CurrentSense_GetState();
    const ElectricalAngle_State_t *angle = ElectricalAngle_GetState();
    int32_t abs_current_a = DqCurrentLoop_Absolute(current->current_a_ma);
    int32_t abs_current_b = DqCurrentLoop_Absolute(current->current_b_ma);
    int32_t position_count;
    int32_t electrical_count;
    int32_t voltage_electrical_count;
    int32_t raw_phase_index;
    int32_t calibrated_phase_index;
    int32_t observer_phase_index;
    int32_t calibrated_predictive_phase_index;
    int32_t control_phase_index;
    int32_t voltage_phase_index;
    int32_t angle_advance_phase_index;
    int32_t angle_advance_count;
    int32_t raw_observer_phase_error_index;
    int32_t abs_raw_observer_phase_error_index;
    int32_t raw_phase_step_index;
    int32_t abs_raw_phase_step_index;
    int32_t current_alpha_ma;
    int32_t current_beta_ma;
    int32_t measured_d_ma;
    int32_t measured_q_ma;
    int32_t voltage_alpha_mv;
    int32_t voltage_beta_mv;
    int32_t abs_output_d;
    int32_t abs_output_q;

    if (abs_current_a > loop->state.peak_abs_phase_current_a_ma)
    {
        loop->state.peak_abs_phase_current_a_ma = abs_current_a;
    }
    if (abs_current_b > loop->state.peak_abs_phase_current_b_ma)
    {
        loop->state.peak_abs_phase_current_b_ma = abs_current_b;
    }

    if ((abs_current_a >= loop->overcurrent_limit_ma) ||
        (abs_current_b >= loop->overcurrent_limit_ma))
    {
        loop->state.fault = DQ_CURRENT_LOOP_FAULT_OVERCURRENT;
        MotorPWM_EnterBrakeState();
        return;
    }

    position_count = Encoder_GetPositionCountFast();
    if ((loop->maximum_position_deviation_count > 0L) &&
        (DqCurrentLoop_Absolute(position_count - loop->start_position_count) >
         loop->maximum_position_deviation_count))
    {
        loop->state.fault = DQ_CURRENT_LOOP_FAULT_POSITION_DEVIATION;
        MotorPWM_EnterBrakeState();
        return;
    }
    if (ElectricalAngle_CalculateCount(position_count,
                                       &electrical_count) == 0U)
    {
        loop->state.fault = DQ_CURRENT_LOOP_FAULT_ANGLE_INVALID;
        MotorPWM_EnterBrakeState();
        return;
    }
    raw_phase_index = DqCurrentLoop_WrapPhaseIndex(
        DqCurrentLoop_ElectricalCountToPhaseIndex(electrical_count));
    if (loop->previous_raw_phase_valid != 0U)
    {
        raw_phase_step_index = DqCurrentLoop_WrapPhaseError(
            raw_phase_index - loop->previous_raw_phase_index);
    }
    else
    {
        raw_phase_step_index = 0L;
        loop->previous_raw_phase_valid = 1U;
    }
    loop->previous_raw_phase_index = raw_phase_index;
    abs_raw_phase_step_index = DqCurrentLoop_Absolute(raw_phase_step_index);
    if (ElectricalAngle_CalculateControlPhaseIndex(
            position_count, &calibrated_phase_index) == 0U)
    {
        loop->state.fault = DQ_CURRENT_LOOP_FAULT_ANGLE_INVALID;
        MotorPWM_EnterBrakeState();
        return;
    }
    observer_phase_index = raw_phase_index;
    if (angle->observer_mode != ELECTRICAL_ANGLE_OBSERVER_OFF)
    {
        if (ElectricalAngle_UpdateObserver(position_count,
                                           &raw_phase_index,
                                           &observer_phase_index) == 0U)
        {
            loop->state.fault = DQ_CURRENT_LOOP_FAULT_ANGLE_INVALID;
            MotorPWM_EnterBrakeState();
            return;
        }
    }
    if (angle->observer_mode == ELECTRICAL_ANGLE_OBSERVER_ON)
    {
        control_phase_index = observer_phase_index;
    }
    else if (angle->observer_mode ==
             ELECTRICAL_ANGLE_OBSERVER_CALIBRATED_PREDICTIVE)
    {
        /* 将当前位置标定误差叠加于连续预测角，避免高速整数角度阶梯。 */
        calibrated_predictive_phase_index = DqCurrentLoop_WrapPhaseIndex(
            observer_phase_index + DqCurrentLoop_WrapPhaseError(
                calibrated_phase_index - raw_phase_index));
        control_phase_index = calibrated_predictive_phase_index;
    }
    else
    {
        control_phase_index = calibrated_phase_index;
    }
    raw_observer_phase_error_index = DqCurrentLoop_WrapPhaseError(
        raw_phase_index - observer_phase_index);
    abs_raw_observer_phase_error_index = DqCurrentLoop_Absolute(
        raw_observer_phase_error_index);
    current_alpha_ma = current->current_a_ma;
    current_beta_ma = (int32_t)angle->phase_b_axis_sign *
                      current->current_b_ma;
    DqTransform_ParkPhaseIndex(current_alpha_ma,
                               current_beta_ma,
                               control_phase_index,
                               &measured_d_ma,
                               &measured_q_ma);
    loop->state.measured_d_ma = measured_d_ma;
    loop->state.measured_q_ma = measured_q_ma;
    loop->state.measured_phase_a_ma = current->current_a_ma;
    loop->state.measured_phase_b_ma = current->current_b_ma;

    angle_advance_count = DqCurrentLoop_CalculateAngleAdvanceCount(
        Encoder_GetControlSpeedMrpmFast());
    voltage_electrical_count = DqCurrentLoop_WrapElectricalCount(
        electrical_count + angle_advance_count);
    angle_advance_phase_index =
        DqCurrentLoop_ElectricalCountToPhaseIndex(angle_advance_count);
    if ((angle->observer_mode == ELECTRICAL_ANGLE_OBSERVER_ON) ||
        (angle->observer_mode ==
         ELECTRICAL_ANGLE_OBSERVER_CALIBRATED_PREDICTIVE) ||
        (angle->calibration_enabled != 0U))
    {
        voltage_phase_index = DqCurrentLoop_WrapPhaseIndex(
            control_phase_index + angle_advance_phase_index);
    }
    else
    {
        /* off/shadow模式保持与旧整数电角度路径逐位一致。 */
        voltage_phase_index = DqCurrentLoop_WrapPhaseIndex(
            DqCurrentLoop_ElectricalCountToPhaseIndex(
                voltage_electrical_count));
    }

    CurrentController_StepWithFeedforward(
        &loop->controller,
        loop->state.target_d_ma,
        loop->state.target_q_ma,
        loop->state.measured_d_ma,
        loop->state.measured_q_ma,
        loop->state.feedforward_d_mv,
        loop->state.feedforward_q_mv,
        ((loop->state.target_d_ma < 0L) &&
         (loop->state.measured_d_ma >
          (loop->state.target_d_ma +
           DQ_CURRENT_LOOP_D_PRIORITY_ERROR_THRESHOLD_MA))) ? 1U : 0U);
    loop->state.output_d_mv = loop->controller.output_axis_1_mv;
    loop->state.output_q_mv = loop->controller.output_axis_2_mv;

    DqTransform_InverseParkPhaseIndex(loop->state.output_d_mv,
                                      loop->state.output_q_mv,
                                      voltage_phase_index,
                                      &voltage_alpha_mv,
                                      &voltage_beta_mv);
    loop->state.phase_voltage_a_mv = voltage_alpha_mv;
    loop->state.phase_voltage_b_mv =
        (int32_t)angle->phase_b_axis_sign * voltage_beta_mv;
    MotorPWM_SetPhaseVoltagesMv(loop->state.phase_voltage_a_mv,
                                loop->state.phase_voltage_b_mv,
                                loop->bus_voltage_mv);

    loop->state.encoder_position_count = position_count;
    loop->state.electrical_count = electrical_count;
    loop->state.voltage_electrical_count = voltage_electrical_count;
    loop->state.raw_phase_index = raw_phase_index;
    loop->state.observer_phase_index = observer_phase_index;
    loop->state.control_phase_index = control_phase_index;
    loop->state.voltage_phase_index = voltage_phase_index;
    loop->state.raw_observer_phase_error_index =
        raw_observer_phase_error_index;
    loop->state.raw_phase_step_index = raw_phase_step_index;
    loop->state.angle_advance_count = angle_advance_count;
    loop->state.angle_advance_delay_us = loop->angle_advance_delay_us;
    loop->state.sample_count++;
    abs_output_d = DqCurrentLoop_Absolute(loop->state.output_d_mv);
    abs_output_q = DqCurrentLoop_Absolute(loop->state.output_q_mv);
    if (abs_output_d > loop->state.peak_abs_output_d_mv)
    {
        loop->state.peak_abs_output_d_mv = abs_output_d;
    }
    if (abs_output_q > loop->state.peak_abs_output_q_mv)
    {
        loop->state.peak_abs_output_q_mv = abs_output_q;
    }
    if (loop->controller.saturated != 0U)
    {
        loop->state.saturated_sample_count++;
    }
    loop->state.voltage_saturated = loop->controller.saturated;
    if (loop->controller.axis_1_priority_active != 0U)
    {
        loop->state.d_axis_priority_sample_count++;
    }
    loop->state.d_axis_priority_active =
        loop->controller.axis_1_priority_active;
    if (abs_raw_observer_phase_error_index >
        loop->state.peak_abs_observer_phase_error_index)
    {
        loop->state.peak_abs_observer_phase_error_index =
            abs_raw_observer_phase_error_index;
    }
    if (abs_raw_observer_phase_error_index >
        DQ_CURRENT_LOOP_OBSERVER_ERROR_THRESHOLD_INDEX)
    {
        loop->state.observer_phase_error_exceed_sample_count++;
    }
    if (abs_raw_phase_step_index >
        loop->state.peak_abs_raw_phase_step_index)
    {
        loop->state.peak_abs_raw_phase_step_index = abs_raw_phase_step_index;
    }
    if (abs_raw_phase_step_index >
        DQ_CURRENT_LOOP_OBSERVER_ERROR_THRESHOLD_INDEX)
    {
        loop->state.raw_phase_step_exceed_sample_count++;
    }

    if (loop->sample_observer != 0)
    {
        loop->sample_observer(&loop->state,
                              loop->sample_observer_context);
    }
}

uint8_t DqCurrentLoop_Start(const DqCurrentLoop_Config_t *config)
{
    const CurrentSense_State_t *current = CurrentSense_GetState();
    const ElectricalAngle_State_t *angle = ElectricalAngle_GetState();

    if ((config == 0) || (s_dq_current_loop.state.running != 0U) ||
        (MotorPWM_IsEnabled() != 0U) || (current->synchronized != 0U) ||
        (current->offset_valid == 0U) || (angle->aligned == 0U) ||
        ((angle->phase_b_axis_sign != 1) &&
         (angle->phase_b_axis_sign != -1)) ||
        (config->maximum_target_current_ma <= 0L) ||
        (config->overcurrent_limit_ma <= config->maximum_target_current_ma) ||
        (config->maximum_position_deviation_count < 0L) ||
        (config->bus_voltage_mv == 0U))
    {
        return 0U;
    }

    if (CurrentController_Init(&s_dq_current_loop.controller,
                               config->bandwidth_hz,
                               config->sample_frequency_hz,
                               config->maximum_voltage_mv) == 0U)
    {
        return 0U;
    }

    s_dq_current_loop.maximum_target_current_ma =
        config->maximum_target_current_ma;
    s_dq_current_loop.overcurrent_limit_ma = config->overcurrent_limit_ma;
    s_dq_current_loop.maximum_position_deviation_count =
        config->maximum_position_deviation_count;
    s_dq_current_loop.start_position_count =
        Encoder_GetPositionCountFast();
    s_dq_current_loop.bus_voltage_mv = config->bus_voltage_mv;
    s_dq_current_loop.state.target_d_ma = 0L;
    s_dq_current_loop.state.target_q_ma = 0L;
    s_dq_current_loop.state.measured_d_ma = 0L;
    s_dq_current_loop.state.measured_q_ma = 0L;
    s_dq_current_loop.state.measured_phase_a_ma = 0L;
    s_dq_current_loop.state.measured_phase_b_ma = 0L;
    s_dq_current_loop.state.output_d_mv = 0L;
    s_dq_current_loop.state.output_q_mv = 0L;
    s_dq_current_loop.state.feedforward_d_mv = 0L;
    s_dq_current_loop.state.feedforward_q_mv = 0L;
    s_dq_current_loop.state.phase_voltage_a_mv = 0L;
    s_dq_current_loop.state.phase_voltage_b_mv = 0L;
    s_dq_current_loop.state.electrical_count = 0L;
    s_dq_current_loop.state.voltage_electrical_count = 0L;
    s_dq_current_loop.state.raw_phase_index = 0L;
    s_dq_current_loop.state.observer_phase_index = 0L;
    s_dq_current_loop.state.control_phase_index = 0L;
    s_dq_current_loop.state.voltage_phase_index = 0L;
    s_dq_current_loop.state.raw_observer_phase_error_index = 0L;
    s_dq_current_loop.state.raw_phase_step_index = 0L;
    s_dq_current_loop.state.angle_advance_count = 0L;
    s_dq_current_loop.state.angle_advance_delay_us =
        s_dq_current_loop.angle_advance_delay_us;
    s_dq_current_loop.state.encoder_position_count =
        Encoder_GetPositionCountFast();
    s_dq_current_loop.state.peak_abs_phase_current_a_ma = 0L;
    s_dq_current_loop.state.peak_abs_phase_current_b_ma = 0L;
    s_dq_current_loop.state.peak_abs_output_d_mv = 0L;
    s_dq_current_loop.state.peak_abs_output_q_mv = 0L;
    s_dq_current_loop.state.sample_count = 0U;
    s_dq_current_loop.state.saturated_sample_count = 0U;
    s_dq_current_loop.state.d_axis_priority_sample_count = 0U;
    s_dq_current_loop.state.observer_phase_error_exceed_sample_count = 0U;
    s_dq_current_loop.state.peak_abs_observer_phase_error_index = 0L;
    s_dq_current_loop.state.raw_phase_step_exceed_sample_count = 0U;
    s_dq_current_loop.state.peak_abs_raw_phase_step_index = 0L;
    s_dq_current_loop.state.fault = DQ_CURRENT_LOOP_FAULT_NONE;
    s_dq_current_loop.state.voltage_saturated = 0U;
    s_dq_current_loop.state.d_axis_priority_active = 0U;

    /* 每次闭环启动都从最新整数位置重置，禁止沿用上次停机速度。 */
    ElectricalAngle_ResetObserver(
        s_dq_current_loop.state.encoder_position_count);
    s_dq_current_loop.previous_raw_phase_index = 0L;
    s_dq_current_loop.previous_raw_phase_valid = 0U;

    CurrentSense_SetSampleCallback(DqCurrentLoop_SampleCallback,
                                   &s_dq_current_loop);
    if (CurrentSense_StartSynchronizedSampling() == 0U)
    {
        CurrentSense_SetSampleCallback(0, 0);
        return 0U;
    }

    s_dq_current_loop.state.running = 1U;
    MotorPWM_StartNeutral();
    return 1U;
}

uint8_t DqCurrentLoop_SetTargets(int32_t target_d_ma,
                                 int32_t target_q_ma)
{
    return DqCurrentLoop_SetOperatingPoint(target_d_ma,
                                           target_q_ma,
                                           0L,
                                           0L);
}

uint8_t DqCurrentLoop_SetOperatingPoint(int32_t target_d_ma,
                                        int32_t target_q_ma,
                                        int32_t feedforward_d_mv,
                                        int32_t feedforward_q_mv)
{
    uint32_t interrupt_mask;

    if ((s_dq_current_loop.state.running == 0U) ||
        (DqCurrentLoop_TargetIsValid(target_d_ma, target_q_ma) == 0U))
    {
        return 0U;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_dq_current_loop.state.target_d_ma = target_d_ma;
    s_dq_current_loop.state.target_q_ma = target_q_ma;
    s_dq_current_loop.state.feedforward_d_mv = feedforward_d_mv;
    s_dq_current_loop.state.feedforward_q_mv = feedforward_q_mv;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

uint8_t DqCurrentLoop_UpdateVoltageBudget(uint32_t bus_voltage_mv,
                                          int32_t maximum_voltage_mv)
{
    uint32_t interrupt_mask;
    uint8_t update_succeeded;

    if ((s_dq_current_loop.state.running == 0U) ||
        (bus_voltage_mv == 0U) ||
        (maximum_voltage_mv <= 0L) ||
        ((uint32_t)maximum_voltage_mv > bus_voltage_mv))
    {
        return 0U;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_dq_current_loop.bus_voltage_mv = bus_voltage_mv;
    update_succeeded = CurrentController_SetMaximumVoltage(
        &s_dq_current_loop.controller,
        maximum_voltage_mv);
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return update_succeeded;
}

uint8_t DqCurrentLoop_SetAngleAdvanceDelayUs(uint32_t delay_us)
{
    if ((s_dq_current_loop.state.running != 0U) ||
        (delay_us > DQ_CURRENT_LOOP_MAX_ANGLE_ADVANCE_DELAY_US))
    {
        return 0U;
    }

    s_dq_current_loop.angle_advance_delay_us = delay_us;
    s_dq_current_loop.state.angle_advance_delay_us = delay_us;
    s_dq_current_loop.state.angle_advance_count = 0L;
    return 1U;
}

void DqCurrentLoop_Stop(void)
{
    if (s_dq_current_loop.state.running == 0U)
    {
        return;
    }

    MotorPWM_EnterBrakeState();
    CurrentSense_StopSynchronizedSampling();
    CurrentSense_SetSampleCallback(0, 0);
    s_dq_current_loop.state.target_d_ma = 0L;
    s_dq_current_loop.state.target_q_ma = 0L;
    s_dq_current_loop.state.output_d_mv = 0L;
    s_dq_current_loop.state.output_q_mv = 0L;
    s_dq_current_loop.state.feedforward_d_mv = 0L;
    s_dq_current_loop.state.feedforward_q_mv = 0L;
    s_dq_current_loop.state.running = 0U;
}

const volatile DqCurrentLoop_State_t *DqCurrentLoop_GetState(void)
{
    return &s_dq_current_loop.state;
}

uint8_t DqCurrentLoop_SetSampleObserver(
    DqCurrentLoop_SampleObserver_t observer,
    void *context)
{
    if (s_dq_current_loop.state.running != 0U)
    {
        return 0U;
    }

    s_dq_current_loop.sample_observer = observer;
    s_dq_current_loop.sample_observer_context = context;
    return 1U;
}
