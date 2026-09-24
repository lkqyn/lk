#include "motor_test.h"

#include "current_sense.h"
#include "dq_current_loop.h"
#include "encoder.h"
#include "phase_current_loop.h"
#include "power_monitor.h"
#include "stm32f4xx_hal.h"

#define MOTOR_TEST_DIFFERENTIAL_COUNTS       (21)
#define MOTOR_TEST_DURATION_MS               (100U)
#define MOTOR_TEST_STATISTICS_START_MS       (50U)
#define MOTOR_TEST_PHASE_OVERCURRENT_LIMIT_MA (800L)
#define MOTOR_TEST_CURRENT_LOOP_FREQUENCY_HZ  (20000.0f)
#define MOTOR_TEST_PHASE_LOOP_BANDWIDTH_HZ    (800.0f)
#define MOTOR_TEST_PHASE_TARGET_LIMIT_MA      (300L)
#define MOTOR_TEST_PHASE_VOLTAGE_LIMIT_MV     (4000L)
#define MOTOR_TEST_DQ_LOOP_BANDWIDTH_HZ       (1200.0f)
#define MOTOR_TEST_DQ_TARGET_LIMIT_MA          (1000L)
#define MOTOR_TEST_DQ_OVERCURRENT_LIMIT_MA     (1500L)
#define MOTOR_TEST_DQ_VOLTAGE_LIMIT_MV         (18000L)
#define MOTOR_TEST_BUS_VOLTAGE_MIN_MV        (18000UL)
/* Keep diagnostic modes consistent with the normal 35 V bus hard limit. */
#define MOTOR_TEST_BUS_VOLTAGE_MAX_MV        (35000UL)
#define MOTOR_TEST_DQ_PRESTEP_SAMPLES         (50U)
#define MOTOR_TEST_DQ_FALL_STEP_SAMPLE        (250U)
#define MOTOR_TEST_IQ_FALL_STEP_SAMPLE        (1050U)
#define MOTOR_TEST_ID_TRACE_SAMPLES           (400U)
#define MOTOR_TEST_IQ_TRACE_SAMPLES           (1250U)
#define MOTOR_TEST_IQ_TARGET_MA                (1000L)
#define MOTOR_TEST_IQ_OVERCURRENT_LIMIT_MA     (1500L)
/*
 * Iq阶跃模式仅在转子已由夹具固定时使用；夹具和轴系弹性扭转会使位置保护错将正常阶跃中断。
 * 此处关闭位置偏差判定，但仍保留1.5A软件过流保护和62.5ms最大阶跃时长。
 */
#define MOTOR_TEST_IQ_POSITION_LIMIT_COUNT     (0L)
#define MOTOR_TEST_DQ_SAMPLE_PERIOD_US         (50L)
#define MOTOR_TEST_DQ_LIVE_DECIMATION          (20U)
#define MOTOR_TEST_DQ_LIVE_BUFFER_CAPACITY     (256U)
#define MOTOR_TEST_DQ_LIVE_BUFFER_MASK         (MOTOR_TEST_DQ_LIVE_BUFFER_CAPACITY - 1U)
#if ((MOTOR_TEST_DQ_LIVE_BUFFER_CAPACITY & MOTOR_TEST_DQ_LIVE_BUFFER_MASK) != 0U)
#error "MOTOR_TEST_DQ_LIVE_BUFFER_CAPACITY must be a power of two"
#endif
#define MOTOR_TEST_DQ_STABLE_WINDOW_SAMPLES   (20U)
#define MOTOR_TEST_DQ_STEADY_SAMPLES          (100U)
#define MOTOR_TEST_DQ_TRANSIENT_SAMPLES       (40U)
#define MOTOR_TEST_TIME_NOT_REACHED_US         (-1L)

static int32_t MotorTest_Absolute(int32_t value);
static uint32_t MotorTest_IntegerSquareRoot(uint64_t value);

typedef struct
{
    int64_t sum_a;
    int64_t sum_b;
    uint64_t square_sum_a;
    uint64_t square_sum_b;
    int32_t minimum_a;
    int32_t maximum_a;
    int32_t minimum_b;
    int32_t maximum_b;
    uint32_t count;
} MotorTest_Accumulator_t;

typedef struct
{
    MotorTest_DqTraceSample_t samples[MOTOR_TEST_DQ_TRACE_CAPACITY];
    volatile uint16_t count;
} MotorTest_DqTraceContext_t;

static MotorTest_DqTraceContext_t s_dq_trace;

typedef struct
{
    MotorTest_DqTraceSample_t samples[MOTOR_TEST_DQ_LIVE_BUFFER_CAPACITY];
    volatile uint16_t write_index;
    volatile uint16_t read_index;
    volatile uint16_t decimation_count;
    volatile uint32_t dropped_samples;
    uint32_t start_tick_ms;
    uint32_t next_step_tick_ms;
    uint32_t stop_tick_ms;
    uint32_t half_period_ms;
    int32_t target_d_ma;
    int32_t target_q_ma;
    volatile uint8_t fault;
    uint8_t target_is_high;
    volatile uint8_t running;
} MotorTest_DqCycleContext_t;

static MotorTest_DqCycleContext_t s_dq_cycle;

static int16_t MotorTest_ClampToInt16(int32_t value)
{
    if (value > INT16_MAX)
    {
        return INT16_MAX;
    }
    if (value < INT16_MIN)
    {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static void MotorTest_RecordDqCycleSample(
    const volatile DqCurrentLoop_State_t *state,
    void *context)
{
    MotorTest_DqCycleContext_t *cycle =
        (MotorTest_DqCycleContext_t *)context;
    uint16_t write_index;
    uint16_t next_index;
    MotorTest_DqTraceSample_t *sample;

    cycle->decimation_count++;
    if (cycle->decimation_count < MOTOR_TEST_DQ_LIVE_DECIMATION)
    {
        return;
    }
    cycle->decimation_count = 0U;

    write_index = cycle->write_index;
    next_index = (uint16_t)((write_index + 1U) &
                            MOTOR_TEST_DQ_LIVE_BUFFER_MASK);
    if (next_index == cycle->read_index)
    {
        cycle->dropped_samples++;
        return;
    }

    sample = &cycle->samples[write_index];
    sample->target_d_ma = MotorTest_ClampToInt16(state->target_d_ma);
    sample->target_q_ma = MotorTest_ClampToInt16(state->target_q_ma);
    sample->measured_d_ma = MotorTest_ClampToInt16(state->measured_d_ma);
    sample->measured_q_ma = MotorTest_ClampToInt16(state->measured_q_ma);
    sample->measured_phase_a_ma =
        MotorTest_ClampToInt16(state->measured_phase_a_ma);
    sample->measured_phase_b_ma =
        MotorTest_ClampToInt16(state->measured_phase_b_ma);
    sample->output_d_mv = MotorTest_ClampToInt16(state->output_d_mv);
    sample->output_q_mv = MotorTest_ClampToInt16(state->output_q_mv);
    sample->electrical_count =
        MotorTest_ClampToInt16(state->electrical_count);
    sample->voltage_saturated = state->voltage_saturated;

    /* 发布索引前确保样本内容已经全部写入。 */
    __DMB();
    cycle->write_index = next_index;
}

static void MotorTest_RecordDqSample(
    const volatile DqCurrentLoop_State_t *state,
    void *context)
{
    MotorTest_DqTraceContext_t *trace =
        (MotorTest_DqTraceContext_t *)context;
    uint16_t index = trace->count;
    MotorTest_DqTraceSample_t *sample;

    if (index >= MOTOR_TEST_DQ_TRACE_CAPACITY)
    {
        return;
    }

    sample = &trace->samples[index];
    sample->target_d_ma = MotorTest_ClampToInt16(state->target_d_ma);
    sample->target_q_ma = MotorTest_ClampToInt16(state->target_q_ma);
    sample->measured_d_ma = MotorTest_ClampToInt16(state->measured_d_ma);
    sample->measured_q_ma = MotorTest_ClampToInt16(state->measured_q_ma);
    sample->measured_phase_a_ma =
        MotorTest_ClampToInt16(state->measured_phase_a_ma);
    sample->measured_phase_b_ma =
        MotorTest_ClampToInt16(state->measured_phase_b_ma);
    sample->output_d_mv = MotorTest_ClampToInt16(state->output_d_mv);
    sample->output_q_mv = MotorTest_ClampToInt16(state->output_q_mv);
    sample->electrical_count =
        MotorTest_ClampToInt16(state->electrical_count);
    sample->voltage_saturated = state->voltage_saturated;

    /* 最后更新计数，保证主循环不会读到未写完的样本。 */
    trace->count = (uint16_t)(index + 1U);
}

static int32_t MotorTest_GetMeasuredAxis(
    const MotorTest_DqTraceSample_t *sample,
    uint8_t axis_is_q)
{
    return (axis_is_q != 0U) ? sample->measured_q_ma :
                               sample->measured_d_ma;
}

static int32_t MotorTest_GetOrthogonalAxis(
    const MotorTest_DqTraceSample_t *sample,
    uint8_t axis_is_q)
{
    return (axis_is_q != 0U) ? sample->measured_d_ma :
                               sample->measured_q_ma;
}

static int32_t MotorTest_GetTargetAxis(
    const MotorTest_DqTraceSample_t *sample,
    uint8_t axis_is_q)
{
    return (axis_is_q != 0U) ? sample->target_q_ma :
                               sample->target_d_ma;
}

static int32_t MotorTest_InterpolateCrossingTime(
    int32_t previous_value,
    int32_t current_value,
    int32_t threshold,
    int32_t current_time_us)
{
    int32_t value_change = current_value - previous_value;

    if (value_change <= 0L)
    {
        return current_time_us;
    }

    return current_time_us - MOTOR_TEST_DQ_SAMPLE_PERIOD_US +
           (int32_t)(((int64_t)(threshold - previous_value) *
                      MOTOR_TEST_DQ_SAMPLE_PERIOD_US) / value_change);
}

static int32_t MotorTest_InterpolateFallingCrossingTime(
    int32_t previous_value,
    int32_t current_value,
    int32_t threshold,
    int32_t current_time_us)
{
    int32_t value_change = previous_value - current_value;

    if (value_change <= 0L)
    {
        return current_time_us;
    }

    return current_time_us - MOTOR_TEST_DQ_SAMPLE_PERIOD_US +
           (int32_t)(((int64_t)(previous_value - threshold) *
                      MOTOR_TEST_DQ_SAMPLE_PERIOD_US) / value_change);
}

static void MotorTest_AnalyzeDqTrace(MotorTest_DqResult_t *result)
{
    uint16_t count = s_dq_trace.count;
    uint16_t step_index = count;
    uint16_t fall_index = count;
    uint16_t index;
    uint16_t stable_count = 0U;
    uint8_t axis_is_q;
    int32_t direction;
    int32_t target_axis_ma;
    int32_t target_abs_ma;
    int32_t band_5_percent_ma;
    int32_t peak_normalized_ma = INT32_MIN;
    int32_t crossing_90_us = MOTOR_TEST_TIME_NOT_REACHED_US;
    int32_t previous_normalized_d_ma = 0L;
    int64_t steady_sum_d = 0LL;
    int64_t steady_sum_q = 0LL;
    uint64_t steady_square_error_d = 0ULL;
    uint64_t steady_square_q = 0ULL;
    int32_t steady_min_d = INT32_MAX;
    int32_t steady_max_d = INT32_MIN;
    int32_t steady_q_abs_peak = 0L;
    uint16_t steady_start;
    uint16_t steady_count;

    result->response_delay_10_us = MOTOR_TEST_TIME_NOT_REACHED_US;
    result->rise_time_10_to_90_us = MOTOR_TEST_TIME_NOT_REACHED_US;
    result->first_target_cross_us = MOTOR_TEST_TIME_NOT_REACHED_US;
    result->first_enter_5_percent_us = MOTOR_TEST_TIME_NOT_REACHED_US;
    result->first_stable_5_percent_us = MOTOR_TEST_TIME_NOT_REACHED_US;
    result->peak_axis_ma = 0L;
    result->peak_time_us = MOTOR_TEST_TIME_NOT_REACHED_US;
    result->overshoot_permille = 0U;
    result->steady_axis_ma = 0L;
    result->steady_orthogonal_ma = 0L;
    result->steady_axis_error_rms_ma = 0U;
    result->steady_orthogonal_rms_ma = 0U;
    result->steady_axis_ripple_pp_ma = 0L;
    result->steady_orthogonal_abs_peak_ma = 0L;
    result->fall_time_90_to_10_us = MOTOR_TEST_TIME_NOT_REACHED_US;
    result->fall_first_enter_5_percent_us = MOTOR_TEST_TIME_NOT_REACHED_US;
    result->fall_first_stable_5_percent_us = MOTOR_TEST_TIME_NOT_REACHED_US;
    result->fall_min_axis_ma = 0L;
    result->fall_axis_rms_ma = 0U;
    result->fall_orthogonal_rms_ma = 0U;

    if (((result->target_d_ma == 0L) && (result->target_q_ma == 0L)) ||
        ((result->target_d_ma != 0L) && (result->target_q_ma != 0L)))
    {
        return;
    }
    axis_is_q = (result->target_q_ma != 0L) ? 1U : 0U;
    target_axis_ma = (axis_is_q != 0U) ? result->target_q_ma :
                                         result->target_d_ma;

    for (index = 0U; index < count; index++)
    {
        if (MotorTest_GetTargetAxis(&s_dq_trace.samples[index],
                                    axis_is_q) != 0L)
        {
            step_index = index;
            break;
        }
    }
    if (step_index >= count)
    {
        return;
    }

    for (index = (uint16_t)(step_index + 1U); index < count; index++)
    {
        if ((MotorTest_GetTargetAxis(&s_dq_trace.samples[index - 1U],
                                     axis_is_q) != 0L) &&
            (MotorTest_GetTargetAxis(&s_dq_trace.samples[index],
                                     axis_is_q) == 0L))
        {
            fall_index = index;
            break;
        }
    }

    direction = (target_axis_ma > 0L) ? 1L : -1L;
    target_abs_ma = MotorTest_Absolute(target_axis_ma);
    band_5_percent_ma = target_abs_ma / 20L;
    if (band_5_percent_ma < 1L)
    {
        band_5_percent_ma = 1L;
    }

    for (index = step_index; index < count; index++)
    {
        int32_t elapsed_us = (int32_t)(index - step_index) *
                             MOTOR_TEST_DQ_SAMPLE_PERIOD_US;
        int32_t normalized_d_ma = direction * MotorTest_GetMeasuredAxis(
            &s_dq_trace.samples[index], axis_is_q);
        int32_t abs_error_ma = MotorTest_Absolute(
            target_abs_ma - normalized_d_ma);

        if (index == step_index)
        {
            previous_normalized_d_ma = normalized_d_ma;
        }

        if ((result->response_delay_10_us < 0L) &&
            (normalized_d_ma >= (target_abs_ma / 10L)))
        {
            result->response_delay_10_us = (index == step_index) ?
                elapsed_us : MotorTest_InterpolateCrossingTime(
                    previous_normalized_d_ma,
                    normalized_d_ma,
                    target_abs_ma / 10L,
                    elapsed_us);
        }
        if ((crossing_90_us < 0L) &&
            (normalized_d_ma >= ((target_abs_ma * 9L) / 10L)) &&
            (result->response_delay_10_us >= 0L))
        {
            crossing_90_us = (index == step_index) ?
                elapsed_us : MotorTest_InterpolateCrossingTime(
                    previous_normalized_d_ma,
                    normalized_d_ma,
                    (target_abs_ma * 9L) / 10L,
                    elapsed_us);
            result->rise_time_10_to_90_us = crossing_90_us -
                result->response_delay_10_us;
        }
        if ((result->first_target_cross_us < 0L) &&
            (normalized_d_ma >= target_abs_ma))
        {
            result->first_target_cross_us = (index == step_index) ?
                elapsed_us : MotorTest_InterpolateCrossingTime(
                    previous_normalized_d_ma,
                    normalized_d_ma,
                    target_abs_ma,
                    elapsed_us);
        }
        if ((result->first_enter_5_percent_us < 0L) &&
            (abs_error_ma <= band_5_percent_ma))
        {
            result->first_enter_5_percent_us = elapsed_us;
        }

        if (abs_error_ma <= band_5_percent_ma)
        {
            stable_count++;
            if ((result->first_stable_5_percent_us < 0L) &&
                (stable_count >= MOTOR_TEST_DQ_STABLE_WINDOW_SAMPLES))
            {
                result->first_stable_5_percent_us =
                    elapsed_us -
                    ((int32_t)(MOTOR_TEST_DQ_STABLE_WINDOW_SAMPLES - 1U) *
                     MOTOR_TEST_DQ_SAMPLE_PERIOD_US);
            }
        }
        else
        {
            stable_count = 0U;
        }

        /* 动态过冲只在阶跃后前2ms内统计，避免把稳态噪声尖峰当成过冲。 */
        if (((index - step_index) < MOTOR_TEST_DQ_TRANSIENT_SAMPLES) &&
            (normalized_d_ma > peak_normalized_ma))
        {
            peak_normalized_ma = normalized_d_ma;
            result->peak_axis_ma = MotorTest_GetMeasuredAxis(
                &s_dq_trace.samples[index], axis_is_q);
            result->peak_time_us = elapsed_us;
        }
        previous_normalized_d_ma = normalized_d_ma;
    }

    if (peak_normalized_ma > target_abs_ma)
    {
        result->overshoot_permille = (uint32_t)(
            ((int64_t)(peak_normalized_ma - target_abs_ma) * 1000LL) /
            target_abs_ma);
    }

    steady_count = (fall_index < count) ? fall_index : count;
    steady_start = (steady_count >
                    (uint16_t)(step_index + MOTOR_TEST_DQ_STEADY_SAMPLES)) ?
                   (uint16_t)(step_index + MOTOR_TEST_DQ_STEADY_SAMPLES) :
                   step_index;
    steady_count = (uint16_t)(steady_count - steady_start);
    for (index = steady_start;
         index < (uint16_t)(steady_start + steady_count);
         index++)
    {
        int32_t measured_d_ma = MotorTest_GetMeasuredAxis(
            &s_dq_trace.samples[index], axis_is_q);
        int32_t measured_q_ma = MotorTest_GetOrthogonalAxis(
            &s_dq_trace.samples[index], axis_is_q);
        int32_t error_d_ma = target_axis_ma - measured_d_ma;
        int32_t abs_q_ma = MotorTest_Absolute(measured_q_ma);

        steady_sum_d += measured_d_ma;
        steady_sum_q += measured_q_ma;
        steady_square_error_d +=
            (uint64_t)((int64_t)error_d_ma * error_d_ma);
        steady_square_q +=
            (uint64_t)((int64_t)measured_q_ma * measured_q_ma);
        if (measured_d_ma < steady_min_d)
        {
            steady_min_d = measured_d_ma;
        }
        if (measured_d_ma > steady_max_d)
        {
            steady_max_d = measured_d_ma;
        }
        if (abs_q_ma > steady_q_abs_peak)
        {
            steady_q_abs_peak = abs_q_ma;
        }
    }
    if (steady_count > 0U)
    {
        result->steady_axis_ma = (int32_t)(steady_sum_d / steady_count);
        result->steady_orthogonal_ma =
            (int32_t)(steady_sum_q / steady_count);
        result->steady_axis_error_rms_ma = MotorTest_IntegerSquareRoot(
            steady_square_error_d / steady_count);
        result->steady_orthogonal_rms_ma = MotorTest_IntegerSquareRoot(
            steady_square_q / steady_count);
        result->steady_axis_ripple_pp_ma = steady_max_d - steady_min_d;
        result->steady_orthogonal_abs_peak_ma = steady_q_abs_peak;
    }


    if (fall_index < count)
    {
        int32_t crossing_90_fall_us = MOTOR_TEST_TIME_NOT_REACHED_US;
        int32_t crossing_10_fall_us = MOTOR_TEST_TIME_NOT_REACHED_US;
        int32_t previous_fall_d_ma = direction * MotorTest_GetMeasuredAxis(
            &s_dq_trace.samples[fall_index], axis_is_q);
        int32_t minimum_fall_d_ma = previous_fall_d_ma;
        uint16_t fall_stable_count = 0U;
        uint16_t fall_rms_start = (count > MOTOR_TEST_DQ_STEADY_SAMPLES) ?
            (uint16_t)(count - MOTOR_TEST_DQ_STEADY_SAMPLES) : fall_index;
        uint64_t fall_square_d = 0ULL;
        uint64_t fall_square_q = 0ULL;
        uint16_t fall_rms_count = (uint16_t)(count - fall_rms_start);

        for (index = fall_index; index < count; index++)
        {
            int32_t elapsed_us = (int32_t)(index - fall_index) *
                                 MOTOR_TEST_DQ_SAMPLE_PERIOD_US;
            int32_t normalized_d_ma = direction * MotorTest_GetMeasuredAxis(
                &s_dq_trace.samples[index], axis_is_q);
            int32_t abs_d_ma = MotorTest_Absolute(normalized_d_ma);

            if ((crossing_90_fall_us < 0L) &&
                (normalized_d_ma <= ((target_abs_ma * 9L) / 10L)))
            {
                crossing_90_fall_us = (index == fall_index) ? elapsed_us :
                    MotorTest_InterpolateFallingCrossingTime(
                        previous_fall_d_ma,
                        normalized_d_ma,
                        (target_abs_ma * 9L) / 10L,
                        elapsed_us);
            }
            if ((crossing_10_fall_us < 0L) &&
                (normalized_d_ma <= (target_abs_ma / 10L)))
            {
                crossing_10_fall_us = (index == fall_index) ? elapsed_us :
                    MotorTest_InterpolateFallingCrossingTime(
                        previous_fall_d_ma,
                        normalized_d_ma,
                        target_abs_ma / 10L,
                        elapsed_us);
            }
            if ((result->fall_first_enter_5_percent_us < 0L) &&
                (abs_d_ma <= band_5_percent_ma))
            {
                result->fall_first_enter_5_percent_us = elapsed_us;
            }

            if (abs_d_ma <= band_5_percent_ma)
            {
                fall_stable_count++;
                if ((result->fall_first_stable_5_percent_us < 0L) &&
                    (fall_stable_count >=
                     MOTOR_TEST_DQ_STABLE_WINDOW_SAMPLES))
                {
                    result->fall_first_stable_5_percent_us = elapsed_us -
                        ((int32_t)(MOTOR_TEST_DQ_STABLE_WINDOW_SAMPLES - 1U) *
                         MOTOR_TEST_DQ_SAMPLE_PERIOD_US);
                }
            }
            else
            {
                fall_stable_count = 0U;
            }

            if (normalized_d_ma < minimum_fall_d_ma)
            {
                minimum_fall_d_ma = normalized_d_ma;
            }
            previous_fall_d_ma = normalized_d_ma;
        }

        if ((crossing_90_fall_us >= 0L) && (crossing_10_fall_us >= 0L))
        {
            result->fall_time_90_to_10_us =
                crossing_10_fall_us - crossing_90_fall_us;
        }
        result->fall_min_axis_ma = direction * minimum_fall_d_ma;

        for (index = fall_rms_start; index < count; index++)
        {
            int32_t measured_d_ma = MotorTest_GetMeasuredAxis(
                &s_dq_trace.samples[index], axis_is_q);
            int32_t measured_q_ma = MotorTest_GetOrthogonalAxis(
                &s_dq_trace.samples[index], axis_is_q);

            fall_square_d +=
                (uint64_t)((int64_t)measured_d_ma * measured_d_ma);
            fall_square_q +=
                (uint64_t)((int64_t)measured_q_ma * measured_q_ma);
        }
        if (fall_rms_count > 0U)
        {
            result->fall_axis_rms_ma = MotorTest_IntegerSquareRoot(
                fall_square_d / fall_rms_count);
            result->fall_orthogonal_rms_ma = MotorTest_IntegerSquareRoot(
                fall_square_q / fall_rms_count);
        }
    }
}

static int32_t MotorTest_Absolute(int32_t value)
{
    return (value < 0L) ? -value : value;
}

static uint32_t MotorTest_IntegerSquareRoot(uint64_t value)
{
    uint64_t result = 0ULL;
    uint64_t bit = 1ULL << 62;

    while (bit > value)
    {
        bit >>= 2;
    }

    while (bit != 0ULL)
    {
        if (value >= (result + bit))
        {
            value -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }

    return (uint32_t)result;
}

static void MotorTest_AccumulatorInit(MotorTest_Accumulator_t *accumulator)
{
    accumulator->sum_a = 0LL;
    accumulator->sum_b = 0LL;
    accumulator->square_sum_a = 0ULL;
    accumulator->square_sum_b = 0ULL;
    accumulator->minimum_a = INT32_MAX;
    accumulator->maximum_a = INT32_MIN;
    accumulator->minimum_b = INT32_MAX;
    accumulator->maximum_b = INT32_MIN;
    accumulator->count = 0U;
}

static void MotorTest_ResultInit(MotorTest_Result_t *result)
{
    result->mean_current_a_ma = 0L;
    result->mean_current_b_ma = 0L;
    result->peak_abs_current_a_ma = 0L;
    result->peak_abs_current_b_ma = 0L;
    result->peak_to_peak_a_ma = 0L;
    result->peak_to_peak_b_ma = 0L;
    result->noise_rms_a_ma = 0U;
    result->noise_rms_b_ma = 0U;
    result->synchronized_sample_count = 0U;
    result->saturated_sample_count = 0U;
    result->peak_abs_voltage_a_mv = 0L;
    result->peak_abs_voltage_b_mv = 0L;
    result->elapsed_ms = 0U;
    result->overcurrent = 0U;
}

static void MotorTest_Accumulate(MotorTest_Accumulator_t *accumulator,
                                 int32_t current_a_ma,
                                 int32_t current_b_ma)
{
    accumulator->sum_a += current_a_ma;
    accumulator->sum_b += current_b_ma;
    accumulator->square_sum_a +=
        (uint64_t)((int64_t)current_a_ma * (int64_t)current_a_ma);
    accumulator->square_sum_b +=
        (uint64_t)((int64_t)current_b_ma * (int64_t)current_b_ma);

    if (current_a_ma < accumulator->minimum_a)
    {
        accumulator->minimum_a = current_a_ma;
    }
    if (current_a_ma > accumulator->maximum_a)
    {
        accumulator->maximum_a = current_a_ma;
    }
    if (current_b_ma < accumulator->minimum_b)
    {
        accumulator->minimum_b = current_b_ma;
    }
    if (current_b_ma > accumulator->maximum_b)
    {
        accumulator->maximum_b = current_b_ma;
    }

    accumulator->count++;
}

static void MotorTest_FinalizeStatistics(const MotorTest_Accumulator_t *accumulator,
                                         MotorTest_Result_t *result)
{
    int64_t mean_square_a;
    int64_t mean_square_b;
    int64_t variance_a;
    int64_t variance_b;

    if (accumulator->count == 0U)
    {
        return;
    }

    result->mean_current_a_ma =
        (int32_t)(accumulator->sum_a / (int64_t)accumulator->count);
    result->mean_current_b_ma =
        (int32_t)(accumulator->sum_b / (int64_t)accumulator->count);
    result->peak_to_peak_a_ma = accumulator->maximum_a - accumulator->minimum_a;
    result->peak_to_peak_b_ma = accumulator->maximum_b - accumulator->minimum_b;

    mean_square_a =
        (int64_t)(accumulator->square_sum_a / accumulator->count);
    mean_square_b =
        (int64_t)(accumulator->square_sum_b / accumulator->count);
    variance_a = mean_square_a -
                 ((int64_t)result->mean_current_a_ma *
                  (int64_t)result->mean_current_a_ma);
    variance_b = mean_square_b -
                 ((int64_t)result->mean_current_b_ma *
                  (int64_t)result->mean_current_b_ma);

    if (variance_a < 0LL)
    {
        variance_a = 0LL;
    }
    if (variance_b < 0LL)
    {
        variance_b = 0LL;
    }

    result->noise_rms_a_ma =
        MotorTest_IntegerSquareRoot((uint64_t)variance_a);
    result->noise_rms_b_ma =
        MotorTest_IntegerSquareRoot((uint64_t)variance_b);
}

static uint8_t MotorTest_Run(int16_t phase_a_differential_counts,
                             int16_t phase_b_differential_counts,
                             MotorTest_Result_t *result)
{
    const CurrentSense_State_t *current = CurrentSense_GetState();
    MotorTest_Accumulator_t accumulator;
    uint32_t start_tick_ms;
    uint32_t last_sample_count = 0U;

    if ((result == 0) || (current->offset_valid == 0U))
    {
        return 0U;
    }

    MotorTest_ResultInit(result);
    MotorTest_AccumulatorInit(&accumulator);

    if (CurrentSense_StartSynchronizedSampling() == 0U)
    {
        return 0U;
    }

    MotorPWM_StartNeutral();
    if (phase_a_differential_counts != 0)
    {
        MotorPWM_SetPhaseDifferentialCounts(MOTOR_PWM_PHASE_A,
                                             phase_a_differential_counts);
    }
    else if (phase_b_differential_counts != 0)
    {
        MotorPWM_SetPhaseDifferentialCounts(MOTOR_PWM_PHASE_B,
                                             phase_b_differential_counts);
    }

    start_tick_ms = HAL_GetTick();
    while ((HAL_GetTick() - start_tick_ms) < MOTOR_TEST_DURATION_MS)
    {
        int32_t current_a_ma;
        int32_t current_b_ma;
        int32_t abs_current_a;
        int32_t abs_current_b;
        uint32_t sample_count;
        uint32_t elapsed_ms;

        current = CurrentSense_GetState();
        sample_count = current->synchronized_sample_count;
        if (sample_count == last_sample_count)
        {
            continue;
        }
        last_sample_count = sample_count;
        current_a_ma = current->current_a_ma;
        current_b_ma = current->current_b_ma;
        abs_current_a = MotorTest_Absolute(current_a_ma);
        abs_current_b = MotorTest_Absolute(current_b_ma);

        if (abs_current_a > result->peak_abs_current_a_ma)
        {
            result->peak_abs_current_a_ma = abs_current_a;
        }
        if (abs_current_b > result->peak_abs_current_b_ma)
        {
            result->peak_abs_current_b_ma = abs_current_b;
        }

        if ((abs_current_a >= MOTOR_TEST_PHASE_OVERCURRENT_LIMIT_MA) ||
            (abs_current_b >= MOTOR_TEST_PHASE_OVERCURRENT_LIMIT_MA))
        {
            result->overcurrent = 1U;
            break;
        }

        elapsed_ms = HAL_GetTick() - start_tick_ms;
        if (elapsed_ms >= MOTOR_TEST_STATISTICS_START_MS)
        {
            MotorTest_Accumulate(&accumulator, current_a_ma, current_b_ma);
        }
    }

    MotorPWM_EnterBrakeState();
    result->synchronized_sample_count = current->synchronized_sample_count;
    result->elapsed_ms = HAL_GetTick() - start_tick_ms;
    CurrentSense_StopSynchronizedSampling();
    MotorTest_FinalizeStatistics(&accumulator, result);
    return 1U;
}

uint8_t MotorTest_RunPhaseCurrent(MotorPWM_Phase_t phase,
                                  int8_t polarity,
                                  MotorTest_Result_t *result)
{
    int16_t differential_counts;

    if (((phase != MOTOR_PWM_PHASE_A) && (phase != MOTOR_PWM_PHASE_B)) ||
        ((polarity != 1) && (polarity != -1)))
    {
        return 0U;
    }

    differential_counts =
        (int16_t)((int16_t)polarity * MOTOR_TEST_DIFFERENTIAL_COUNTS);
    return (phase == MOTOR_PWM_PHASE_A) ?
           MotorTest_Run(differential_counts, 0, result) :
           MotorTest_Run(0, differential_counts, result);
}

uint8_t MotorTest_RunCurrentNoise(MotorTest_Result_t *result)
{
    return MotorTest_Run(0, 0, result);
}

uint8_t MotorTest_RunClosedLoopCurrent(int32_t target_a_ma,
                                       int32_t target_b_ma,
                                       MotorTest_Result_t *result)
{
    const CurrentSense_State_t *current = CurrentSense_GetState();
    const PowerMonitor_State_t *power;
    const volatile PhaseCurrentLoop_State_t *loop;
    PhaseCurrentLoop_Config_t loop_config;
    MotorTest_Accumulator_t accumulator;
    uint32_t start_tick_ms;
    uint32_t last_sample_count = 0U;

    if ((result == 0) || (current->offset_valid == 0U) ||
        (MotorTest_Absolute(target_a_ma) > MOTOR_TEST_PHASE_TARGET_LIMIT_MA) ||
        (MotorTest_Absolute(target_b_ma) > MOTOR_TEST_PHASE_TARGET_LIMIT_MA))
    {
        return 0U;
    }

    PowerMonitor_Update();
    power = PowerMonitor_GetState();
    if ((power->bus_voltage_mv < MOTOR_TEST_BUS_VOLTAGE_MIN_MV) ||
        (power->bus_voltage_mv > MOTOR_TEST_BUS_VOLTAGE_MAX_MV))
    {
        return 0U;
    }

    MotorTest_ResultInit(result);
    MotorTest_AccumulatorInit(&accumulator);

    loop_config.bandwidth_hz = MOTOR_TEST_PHASE_LOOP_BANDWIDTH_HZ;
    loop_config.sample_frequency_hz = MOTOR_TEST_CURRENT_LOOP_FREQUENCY_HZ;
    loop_config.maximum_voltage_mv = MOTOR_TEST_PHASE_VOLTAGE_LIMIT_MV;
    loop_config.maximum_target_current_ma = MOTOR_TEST_PHASE_TARGET_LIMIT_MA;
    loop_config.overcurrent_limit_ma =
        MOTOR_TEST_PHASE_OVERCURRENT_LIMIT_MA;
    loop_config.bus_voltage_mv = power->bus_voltage_mv;
    if (PhaseCurrentLoop_Start(&loop_config) == 0U)
    {
        return 0U;
    }
    if (PhaseCurrentLoop_SetTargets(target_a_ma, target_b_ma) == 0U)
    {
        PhaseCurrentLoop_Stop();
        return 0U;
    }

    loop = PhaseCurrentLoop_GetState();
    start_tick_ms = HAL_GetTick();
    while (((HAL_GetTick() - start_tick_ms) < MOTOR_TEST_DURATION_MS) &&
           (loop->fault == PHASE_CURRENT_LOOP_FAULT_NONE))
    {
        int32_t current_a_ma;
        int32_t current_b_ma;
        int32_t abs_current_a;
        int32_t abs_current_b;
        uint32_t sample_count = current->synchronized_sample_count;

        if (sample_count == last_sample_count)
        {
            continue;
        }
        last_sample_count = sample_count;
        current_a_ma = current->current_a_ma;
        current_b_ma = current->current_b_ma;
        abs_current_a = MotorTest_Absolute(current_a_ma);
        abs_current_b = MotorTest_Absolute(current_b_ma);
        if (abs_current_a > result->peak_abs_current_a_ma)
        {
            result->peak_abs_current_a_ma = abs_current_a;
        }
        if (abs_current_b > result->peak_abs_current_b_ma)
        {
            result->peak_abs_current_b_ma = abs_current_b;
        }
        if ((HAL_GetTick() - start_tick_ms) >= MOTOR_TEST_STATISTICS_START_MS)
        {
            MotorTest_Accumulate(&accumulator, current_a_ma, current_b_ma);
        }
    }

    result->synchronized_sample_count = current->synchronized_sample_count;
    result->saturated_sample_count = loop->saturated_sample_count;
    result->peak_abs_voltage_a_mv = loop->peak_abs_output_a_mv;
    result->peak_abs_voltage_b_mv = loop->peak_abs_output_b_mv;
    result->elapsed_ms = HAL_GetTick() - start_tick_ms;
    result->overcurrent =
        (loop->fault == PHASE_CURRENT_LOOP_FAULT_OVERCURRENT) ? 1U : 0U;
    PhaseCurrentLoop_Stop();
    MotorTest_FinalizeStatistics(&accumulator, result);
    return 1U;
}

uint8_t MotorTest_RunDqCurrent(int32_t target_d_ma,
                              int32_t target_q_ma,
                              MotorTest_DqResult_t *result)
{
    const CurrentSense_State_t *current = CurrentSense_GetState();
    const PowerMonitor_State_t *power;
    const volatile DqCurrentLoop_State_t *loop;
    DqCurrentLoop_Config_t loop_config;
    int32_t start_position_count;
    uint32_t start_tick_ms;
    uint8_t fall_step_applied = 0U;
    uint8_t axis_is_q;
    uint16_t fall_step_sample;
    uint16_t trace_sample_count;
    int64_t target_magnitude_squared =
        ((int64_t)target_d_ma * target_d_ma) +
        ((int64_t)target_q_ma * target_q_ma);
    int64_t maximum_magnitude_squared =
        (int64_t)MOTOR_TEST_DQ_TARGET_LIMIT_MA *
        MOTOR_TEST_DQ_TARGET_LIMIT_MA;

    if ((result == 0) || (current->offset_valid == 0U) ||
        (target_magnitude_squared > maximum_magnitude_squared) ||
        ((target_d_ma != 0L) && (target_q_ma != 0L)) ||
        ((target_d_ma == 0L) && (target_q_ma == 0L)))
    {
        return 0U;
    }

    axis_is_q = (target_q_ma != 0L) ? 1U : 0U;
    if ((axis_is_q != 0U) &&
        (MotorTest_Absolute(target_q_ma) > MOTOR_TEST_IQ_TARGET_MA))
    {
        return 0U;
    }
    fall_step_sample = (axis_is_q != 0U) ?
        MOTOR_TEST_IQ_FALL_STEP_SAMPLE : MOTOR_TEST_DQ_FALL_STEP_SAMPLE;
    trace_sample_count = (axis_is_q != 0U) ?
        MOTOR_TEST_IQ_TRACE_SAMPLES : MOTOR_TEST_ID_TRACE_SAMPLES;

    PowerMonitor_Update();
    power = PowerMonitor_GetState();
    if ((power->bus_voltage_mv < MOTOR_TEST_BUS_VOLTAGE_MIN_MV) ||
        (power->bus_voltage_mv > MOTOR_TEST_BUS_VOLTAGE_MAX_MV))
    {
        return 0U;
    }

    result->target_d_ma = target_d_ma;
    result->target_q_ma = target_q_ma;
    result->bandwidth_hz =
        (uint32_t)(MOTOR_TEST_DQ_LOOP_BANDWIDTH_HZ + 0.5f);
    result->voltage_limit_mv = MOTOR_TEST_DQ_VOLTAGE_LIMIT_MV;
    result->movement_count = 0L;
    result->final_electrical_count = 0L;
    result->peak_abs_phase_current_a_ma = 0L;
    result->peak_abs_phase_current_b_ma = 0L;
    result->peak_abs_output_d_mv = 0L;
    result->peak_abs_output_q_mv = 0L;
    result->synchronized_sample_count = 0U;
    result->saturated_sample_count = 0U;
    result->elapsed_ms = 0U;
    result->fault = (uint8_t)DQ_CURRENT_LOOP_FAULT_NONE;

    loop_config.bandwidth_hz = MOTOR_TEST_DQ_LOOP_BANDWIDTH_HZ;
    loop_config.sample_frequency_hz = MOTOR_TEST_CURRENT_LOOP_FREQUENCY_HZ;
    loop_config.maximum_voltage_mv = MOTOR_TEST_DQ_VOLTAGE_LIMIT_MV;
    loop_config.maximum_target_current_ma = (axis_is_q != 0U) ?
        MOTOR_TEST_IQ_TARGET_MA : MOTOR_TEST_DQ_TARGET_LIMIT_MA;
    loop_config.overcurrent_limit_ma = (axis_is_q != 0U) ?
        MOTOR_TEST_IQ_OVERCURRENT_LIMIT_MA :
        MOTOR_TEST_DQ_OVERCURRENT_LIMIT_MA;
    loop_config.maximum_position_deviation_count = (axis_is_q != 0U) ?
        MOTOR_TEST_IQ_POSITION_LIMIT_COUNT : 0L;
    loop_config.bus_voltage_mv = power->bus_voltage_mv;
    s_dq_trace.count = 0U;
    if (DqCurrentLoop_SetSampleObserver(MotorTest_RecordDqSample,
                                       &s_dq_trace) == 0U)
    {
        return 0U;
    }
    if (DqCurrentLoop_Start(&loop_config) == 0U)
    {
        (void)DqCurrentLoop_SetSampleObserver(0, 0);
        return 0U;
    }
    loop = DqCurrentLoop_GetState();

    /* 先记录2.5ms零给定，再施加阶跃，便于判断基线和动态响应。 */
    start_tick_ms = HAL_GetTick();
    while ((s_dq_trace.count < MOTOR_TEST_DQ_PRESTEP_SAMPLES) &&
           (loop->fault == DQ_CURRENT_LOOP_FAULT_NONE) &&
           ((HAL_GetTick() - start_tick_ms) < MOTOR_TEST_DURATION_MS))
    {
        /* 数据由ADC同步中断写入。 */
    }
    if ((s_dq_trace.count < MOTOR_TEST_DQ_PRESTEP_SAMPLES) ||
        (DqCurrentLoop_SetTargets(target_d_ma, target_q_ma) == 0U))
    {
        DqCurrentLoop_Stop();
        (void)DqCurrentLoop_SetSampleObserver(0, 0);
        return 0U;
    }

    start_position_count = Encoder_GetPositionCountFast();
    start_tick_ms = HAL_GetTick();
    while ((s_dq_trace.count < trace_sample_count) &&
           ((HAL_GetTick() - start_tick_ms) < MOTOR_TEST_DURATION_MS) &&
           (loop->fault == DQ_CURRENT_LOOP_FAULT_NONE))
    {
        if ((fall_step_applied == 0U) &&
            (s_dq_trace.count >= fall_step_sample))
        {
            if (DqCurrentLoop_SetTargets(0L, 0L) == 0U)
            {
                break;
            }
            fall_step_applied = 1U;
        }
    }

    result->movement_count = Encoder_GetPositionCountFast() -
                             start_position_count;
    result->final_electrical_count = loop->electrical_count;
    result->peak_abs_phase_current_a_ma =
        loop->peak_abs_phase_current_a_ma;
    result->peak_abs_phase_current_b_ma =
        loop->peak_abs_phase_current_b_ma;
    result->peak_abs_output_d_mv = loop->peak_abs_output_d_mv;
    result->peak_abs_output_q_mv = loop->peak_abs_output_q_mv;
    result->synchronized_sample_count = loop->sample_count;
    result->saturated_sample_count = loop->saturated_sample_count;
    result->elapsed_ms = HAL_GetTick() - start_tick_ms;
    result->fault = (uint8_t)loop->fault;
    DqCurrentLoop_Stop();
    (void)DqCurrentLoop_SetSampleObserver(0, 0);
    MotorTest_AnalyzeDqTrace(result);
    return 1U;
}

const MotorTest_DqTraceSample_t *MotorTest_GetDqTrace(
    uint16_t *sample_count)
{
    if (sample_count != 0)
    {
        *sample_count = s_dq_trace.count;
    }
    return s_dq_trace.samples;
}

uint8_t MotorTest_StartDqCycle(int32_t target_d_ma,
                               int32_t target_q_ma,
                               uint32_t half_period_ms,
                               uint32_t duration_ms)
{
    const CurrentSense_State_t *current = CurrentSense_GetState();
    const PowerMonitor_State_t *power;
    DqCurrentLoop_Config_t loop_config;

    uint8_t axis_is_q = (target_q_ma != 0L) ? 1U : 0U;
    int64_t target_magnitude_squared =
        ((int64_t)target_d_ma * target_d_ma) +
        ((int64_t)target_q_ma * target_q_ma);

    if ((s_dq_cycle.running != 0U) ||
        ((target_d_ma != 0L) && (target_q_ma != 0L)) ||
        ((target_d_ma == 0L) && (target_q_ma == 0L)) ||
        (target_magnitude_squared >
         ((int64_t)MOTOR_TEST_DQ_TARGET_LIMIT_MA *
          MOTOR_TEST_DQ_TARGET_LIMIT_MA)) ||
        ((axis_is_q != 0U) &&
         (MotorTest_Absolute(target_q_ma) > MOTOR_TEST_IQ_TARGET_MA)) ||
        (half_period_ms == 0U) ||
        (duration_ms < (2U * half_period_ms)) ||
        (current->offset_valid == 0U))
    {
        return 0U;
    }

    PowerMonitor_Update();
    power = PowerMonitor_GetState();
    if ((power->bus_voltage_mv < MOTOR_TEST_BUS_VOLTAGE_MIN_MV) ||
        (power->bus_voltage_mv > MOTOR_TEST_BUS_VOLTAGE_MAX_MV))
    {
        return 0U;
    }

    loop_config.bandwidth_hz = MOTOR_TEST_DQ_LOOP_BANDWIDTH_HZ;
    loop_config.sample_frequency_hz = MOTOR_TEST_CURRENT_LOOP_FREQUENCY_HZ;
    loop_config.maximum_voltage_mv = MOTOR_TEST_DQ_VOLTAGE_LIMIT_MV;
    loop_config.maximum_target_current_ma = (axis_is_q != 0U) ?
        MOTOR_TEST_IQ_TARGET_MA : MOTOR_TEST_DQ_TARGET_LIMIT_MA;
    loop_config.overcurrent_limit_ma = (axis_is_q != 0U) ?
        MOTOR_TEST_IQ_OVERCURRENT_LIMIT_MA :
        MOTOR_TEST_DQ_OVERCURRENT_LIMIT_MA;
    /* Iq周期阶跃测试允许少量夹具弹性，明显松脱时仍快速停机。 */
    loop_config.maximum_position_deviation_count =
        (axis_is_q != 0U) ? 20L : 0L;
    loop_config.bus_voltage_mv = power->bus_voltage_mv;

    s_dq_cycle.write_index = 0U;
    s_dq_cycle.read_index = 0U;
    s_dq_cycle.decimation_count = 0U;
    s_dq_cycle.dropped_samples = 0U;
    s_dq_cycle.half_period_ms = half_period_ms;
    s_dq_cycle.target_d_ma = target_d_ma;
    s_dq_cycle.target_q_ma = target_q_ma;
    s_dq_cycle.target_is_high = 0U;
    s_dq_cycle.fault = (uint8_t)DQ_CURRENT_LOOP_FAULT_NONE;

    if (DqCurrentLoop_SetSampleObserver(
            MotorTest_RecordDqCycleSample,
            &s_dq_cycle) == 0U)
    {
        return 0U;
    }
    if (DqCurrentLoop_Start(&loop_config) == 0U)
    {
        (void)DqCurrentLoop_SetSampleObserver(0, 0);
        return 0U;
    }

    s_dq_cycle.start_tick_ms = HAL_GetTick();
    s_dq_cycle.next_step_tick_ms =
        s_dq_cycle.start_tick_ms + half_period_ms;
    s_dq_cycle.stop_tick_ms =
        s_dq_cycle.start_tick_ms + duration_ms;
    s_dq_cycle.running = 1U;
    return 1U;
}

void MotorTest_UpdateDqCycle(void)
{
    const volatile DqCurrentLoop_State_t *loop;
    uint32_t now_ms;

    if (s_dq_cycle.running == 0U)
    {
        return;
    }

    loop = DqCurrentLoop_GetState();
    if ((loop->running == 0U) ||
        (loop->fault != DQ_CURRENT_LOOP_FAULT_NONE))
    {
        s_dq_cycle.fault = (uint8_t)loop->fault;
        MotorTest_StopDqCycle();
        return;
    }

    now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - s_dq_cycle.stop_tick_ms) >= 0L)
    {
        MotorTest_StopDqCycle();
        return;
    }

    if ((int32_t)(now_ms - s_dq_cycle.next_step_tick_ms) >= 0L)
    {
        s_dq_cycle.target_is_high ^= 1U;
        if (DqCurrentLoop_SetTargets(
                (s_dq_cycle.target_is_high != 0U) ?
                    s_dq_cycle.target_d_ma : 0L,
                (s_dq_cycle.target_is_high != 0U) ?
                    s_dq_cycle.target_q_ma : 0L) == 0U)
        {
            MotorTest_StopDqCycle();
            return;
        }
        s_dq_cycle.next_step_tick_ms += s_dq_cycle.half_period_ms;
    }
}

void MotorTest_StopDqCycle(void)
{
    const volatile DqCurrentLoop_State_t *loop;

    if (s_dq_cycle.running == 0U)
    {
        return;
    }

    loop = DqCurrentLoop_GetState();

    if (loop->fault != DQ_CURRENT_LOOP_FAULT_NONE)
    {
        s_dq_cycle.fault = (uint8_t)loop->fault;
    }
    DqCurrentLoop_Stop();
    (void)DqCurrentLoop_SetSampleObserver(0, 0);
    s_dq_cycle.running = 0U;
    s_dq_cycle.target_is_high = 0U;
}

uint8_t MotorTest_PopDqCycleSample(MotorTest_DqTraceSample_t *sample)
{
    uint16_t read_index;

    if (sample == 0)
    {
        return 0U;
    }

    read_index = s_dq_cycle.read_index;
    if (read_index == s_dq_cycle.write_index)
    {
        return 0U;
    }

    __DMB();
    *sample = s_dq_cycle.samples[read_index];
    s_dq_cycle.read_index = (uint16_t)(
        (read_index + 1U) & MOTOR_TEST_DQ_LIVE_BUFFER_MASK);
    return 1U;
}

uint8_t MotorTest_DqCycleIsRunning(void)
{
    return s_dq_cycle.running;
}

uint8_t MotorTest_GetDqCycleFault(void)
{
    return s_dq_cycle.fault;
}

uint32_t MotorTest_GetDqCycleDroppedSamples(void)
{
    return s_dq_cycle.dropped_samples;
}
