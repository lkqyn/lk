/* 上电电角对齐及编码器整圈非线性边界标定的非阻塞状态机实现。 */
#include "rotor_alignment.h"

#include "electrical_angle.h"
#include "encoder.h"
#include "current_sense.h"
#include "motor_parameters.h"
#include "motor_pwm.h"
#include "phase_current_loop.h"
#include "power_monitor.h"
#include "stm32f4xx_hal.h"

#include <math.h>

/* 1.5 A实测移动17个计数，使用额定电流40%完成严格零点验证。 */
#define ROTOR_ALIGNMENT_CURRENT_MA              (2000L)
#define ROTOR_ALIGNMENT_CURRENT_RAMP_MS         (100U)
#define ROTOR_ALIGNMENT_HOLD_MS                 (300U)
#define ROTOR_ALIGNMENT_AVERAGE_WINDOW_MS       (100U)
#define ROTOR_ALIGNMENT_ROTATION_MS             (500U)
#define ROTOR_ALIGNMENT_CURRENT_LOOP_HZ         (20000.0f)
#define ROTOR_ALIGNMENT_CURRENT_BANDWIDTH_HZ    (800.0f)
#define ROTOR_ALIGNMENT_VOLTAGE_LIMIT_MV        (4000L)
#define ROTOR_ALIGNMENT_OVERCURRENT_MA          (2500L)
#define ROTOR_ALIGNMENT_BUS_MIN_MV              (18000UL)
/* Keep alignment consistent with the normal 35 V bus hard limit. */
#define ROTOR_ALIGNMENT_BUS_MAX_MV              (35000UL)
#define ROTOR_ALIGNMENT_AUTO_BUS_STABLE_MS       (500U)
#define ROTOR_ALIGNMENT_HALF_PI_RAD             (1.5707963267948966192f)
/* 4000线编码器下1计数对应4.5°电角度，零点验证误差限制为±9°。 */
#define ROTOR_ALIGNMENT_MOVEMENT_TOLERANCE      (2L)

/*
 * 编码器非线性标定仅采集数据，本阶段不将校正量接入FOC。
 * 20ms内平滑移动一个1.8°整步，再静置10ms取一个实时位置点。
 */
#define ENCODER_CALIBRATION_CURRENT_MA           (2000L)
#define ENCODER_CALIBRATION_CURRENT_RAMP_MS      (200U)
#define ENCODER_CALIBRATION_START_HOLD_MS        (300U)
#define ENCODER_CALIBRATION_MOVE_STEP_MS         (20U)
#define ENCODER_CALIBRATION_SETTLE_MS            (10U)
#define ENCODER_CALIBRATION_FULL_STEP_COUNT      (200U)
#define ENCODER_CALIBRATION_MINIMUM_INTERVAL     (12L)
#define ENCODER_CALIBRATION_MAXIMUM_INTERVAL     (28L)
#define ENCODER_CALIBRATION_TOTAL_TOLERANCE      (80L)
#define ENCODER_CALIBRATION_HALF_PI_RAD          (1.5707963267948966192f)

typedef struct
{
    RotorAlignment_State_t state;
    int64_t position_sum;
    int64_t current_a_sum;
    int64_t current_b_sum;
    uint32_t position_sample_count;
    uint32_t state_start_tick_ms;
    uint32_t last_average_tick_ms;
    uint32_t auto_ready_start_tick_ms;
    uint8_t auto_bus_ready;
} RotorAlignment_Context_t;

typedef struct
{
    EncoderCalibration_State_t state;
    int32_t forward_count[ENCODER_CALIBRATION_BOUNDARY_COUNT];
    int32_t reverse_count[ENCODER_CALIBRATION_BOUNDARY_COUNT];
    int32_t midpoint_q1[ENCODER_CALIBRATION_BOUNDARY_COUNT];
    uint32_t state_start_tick_ms;
    int32_t preload_position_count;
    int32_t overshoot_position_count;
    int16_t commanded_boundary;
} EncoderCalibration_Context_t;

static RotorAlignment_Context_t s_rotor_alignment;
static EncoderCalibration_Context_t s_encoder_calibration;

/* 返回有符号计数的绝对值，供对齐误差和容差判断使用。 */
static int32_t RotorAlignment_Absolute(int32_t value)
{
    return (value < 0L) ? -value : value;
}

/* 将浮点计算结果按远离零方向四舍五入为编码器/电流整数。 */
static int32_t RotorAlignment_RoundFloat(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                             (int32_t)(value - 0.5f);
}

/* 返回当前采样窗口内的平均机械位置，降低编码器瞬时抖动影响。 */
static int32_t RotorAlignment_AveragePosition(void)
{
    int64_t rounded_sum = s_rotor_alignment.position_sum;
    int64_t half_count;

    if (s_rotor_alignment.position_sample_count == 0U)
    {
        return Encoder_GetState()->position_count;
    }

    half_count = (int64_t)s_rotor_alignment.position_sample_count / 2LL;
    rounded_sum += (rounded_sum >= 0LL) ? half_count : -half_count;
    return (int32_t)(rounded_sum /
                     (int64_t)s_rotor_alignment.position_sample_count);
}

/* 将采样窗口的电流累加值转换为平均相电流。 */
static int32_t RotorAlignment_AverageCurrent(int64_t current_sum)
{
    int64_t half_count;

    if (s_rotor_alignment.position_sample_count == 0U)
    {
        return 0L;
    }

    half_count = (int64_t)s_rotor_alignment.position_sample_count / 2LL;
    current_sum += (current_sum >= 0LL) ? half_count : -half_count;
    return (int32_t)(current_sum /
                     (int64_t)s_rotor_alignment.position_sample_count);
}

/* 将跨一圈的机械位置差折返到半圈以内的最短差值。 */
static int32_t RotorAlignment_WrapMechanicalDelta(int32_t delta_count)
{
    int32_t wrapped = delta_count % MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
    int32_t half_revolution = MOTOR_ENCODER_COUNTS_PER_REVOLUTION / 2L;

    if (wrapped > half_revolution)
    {
        wrapped -= MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
    }
    else if (wrapped < -half_revolution)
    {
        wrapped += MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
    }
    return wrapped;
}

/* 切换主对齐状态并清空该状态的统计窗口。 */
static void RotorAlignment_EnterState(RotorAlignment_StateCode_t state,
                                      uint32_t now_ms)
{
    s_rotor_alignment.state.state = state;
    s_rotor_alignment.state_start_tick_ms = now_ms;
    s_rotor_alignment.position_sum = 0LL;
    s_rotor_alignment.current_a_sum = 0LL;
    s_rotor_alignment.current_b_sum = 0LL;
    s_rotor_alignment.position_sample_count = 0U;
    s_rotor_alignment.last_average_tick_ms = now_ms;
}

/* 向相电流环下发对齐矢量，同时记录当前命令值。 */
static uint8_t RotorAlignment_SetTargets(int32_t target_a_ma,
                                         int32_t target_b_ma)
{
    if (PhaseCurrentLoop_SetTargets(target_a_ma, target_b_ma) == 0U)
    {
        return 0U;
    }

    s_rotor_alignment.state.target_a_ma = target_a_ma;
    s_rotor_alignment.state.target_b_ma = target_b_ma;
    return 1U;
}

/* 每毫秒累加位置与两相电流，用于对齐结果的平均判定。 */
static void RotorAlignment_AccumulateMeasurements(uint32_t now_ms)
{
    const CurrentSense_State_t *current;

    if (now_ms == s_rotor_alignment.last_average_tick_ms)
    {
        return;
    }

    s_rotor_alignment.last_average_tick_ms = now_ms;
    current = CurrentSense_GetState();
    s_rotor_alignment.position_sum += Encoder_GetState()->position_count;
    s_rotor_alignment.current_a_sum += current->current_a_ma;
    s_rotor_alignment.current_b_sum += current->current_b_ma;
    s_rotor_alignment.position_sample_count++;
}

/* 将相电流环运行统计快照保存到对齐诊断状态。 */
static void RotorAlignment_CaptureLoopStatistics(void)
{
    const volatile PhaseCurrentLoop_State_t *loop =
        PhaseCurrentLoop_GetState();

    s_rotor_alignment.state.current_sample_count = loop->sample_count;
    s_rotor_alignment.state.saturated_sample_count =
        loop->saturated_sample_count;
    s_rotor_alignment.state.peak_abs_current_a_ma =
        loop->peak_abs_current_a_ma;
    s_rotor_alignment.state.peak_abs_current_b_ma =
        loop->peak_abs_current_b_ma;
    s_rotor_alignment.state.peak_abs_voltage_a_mv =
        loop->peak_abs_output_a_mv;
    s_rotor_alignment.state.peak_abs_voltage_b_mv =
        loop->peak_abs_output_b_mv;
}

/* 结束对齐失败路径：停止输出、使电角度失效并保留故障原因。 */
static void RotorAlignment_FinishFault(RotorAlignment_Fault_t fault)
{
    RotorAlignment_CaptureLoopStatistics();
    PhaseCurrentLoop_Stop();
    ElectricalAngle_Invalidate();
    s_rotor_alignment.state.target_a_ma = 0L;
    s_rotor_alignment.state.target_b_ma = 0L;
    s_rotor_alignment.state.valid = 0U;
    s_rotor_alignment.state.fault = fault;
    s_rotor_alignment.state.state = ROTOR_ALIGNMENT_STATE_FAULT;
}

/* 结束对齐成功路径：保存零点/相序符号并发布电角度映射。 */
static void RotorAlignment_FinishSuccess(void)
{
    RotorAlignment_CaptureLoopStatistics();
    PhaseCurrentLoop_Stop();
    s_rotor_alignment.state.target_a_ma = 0L;
    s_rotor_alignment.state.target_b_ma = 0L;
    s_rotor_alignment.state.alignment_offset_count =
        s_rotor_alignment.state.zero_position_count;
    s_rotor_alignment.state.phase_b_axis_sign =
        (s_rotor_alignment.state.movement_count < 0L) ? 1 : -1;
    s_rotor_alignment.state.valid = 1U;
    s_rotor_alignment.state.fault = ROTOR_ALIGNMENT_FAULT_NONE;
    s_rotor_alignment.state.state = ROTOR_ALIGNMENT_STATE_COMPLETE;
    ElectricalAngle_SetAlignment(
        s_rotor_alignment.state.alignment_offset_count,
        s_rotor_alignment.state.phase_b_axis_sign);
}

/* 切换编码器非线性标定子状态并重新开始计时。 */
static void EncoderCalibration_EnterState(
    EncoderCalibration_StateCode_t state,
    uint32_t now_ms)
{
    s_encoder_calibration.state.state = state;
    s_encoder_calibration.state_start_tick_ms = now_ms;
}

/* 按四分之一电周期生成标定电流矢量并下发到相电流环。 */
static uint8_t EncoderCalibration_SetVector(float quarter_step,
                                            int32_t amplitude_ma)
{
    float angle_rad = quarter_step * ENCODER_CALIBRATION_HALF_PI_RAD;
    int32_t target_a_ma = RotorAlignment_RoundFloat(
        (float)amplitude_ma * cosf(angle_rad));
    int32_t target_b_ma = RotorAlignment_RoundFloat(
        (float)amplitude_ma * sinf(angle_rad));

    target_b_ma *= (int32_t)s_rotor_alignment.state.phase_b_axis_sign;
    return PhaseCurrentLoop_SetTargets(target_a_ma, target_b_ma);
}

/* 将四分之一电周期索引规范到 0~3，供相位方向判断使用。 */
static int32_t EncoderCalibration_QuarterModulo(int32_t quarter_step)
{
    int32_t wrapped = quarter_step % 4L;

    if (wrapped < 0L)
    {
        wrapped += 4L;
    }
    return wrapped;
}

/* 结束编码器标定失败路径，并关闭用于转动转子的电流。 */
static void EncoderCalibration_FinishFault(
    EncoderCalibration_Fault_t fault)
{
    s_encoder_calibration.state.saturated_sample_count =
        PhaseCurrentLoop_GetState()->saturated_sample_count;
    PhaseCurrentLoop_Stop();
    s_encoder_calibration.state.data_valid = 0U;
    s_encoder_calibration.state.fault = fault;
    s_encoder_calibration.state.state = ENCODER_CALIBRATION_STATE_FAULT;
}

/* 判断相邻标定边界的编码器间隔是否落在可信范围内。 */
static uint8_t EncoderCalibration_IntervalIsValid(int32_t interval_count)
{
    return ((interval_count >= ENCODER_CALIBRATION_MINIMUM_INTERVAL) &&
            (interval_count <= ENCODER_CALIBRATION_MAXIMUM_INTERVAL)) ?
           1U : 0U;
}

/* 汇总正反向采样结果，计算边界误差/回差并发布有效标定数据。 */
static void EncoderCalibration_FinishSuccess(void)
{
    int32_t minimum_interval_q1 = INT32_MAX;
    int32_t maximum_interval_q1 = INT32_MIN;
    int32_t maximum_hysteresis_count = 0L;
    uint16_t index;

    s_encoder_calibration.state.forward_total_count =
        s_encoder_calibration.forward_count[
            ENCODER_CALIBRATION_FULL_STEP_COUNT] -
        s_encoder_calibration.forward_count[0U];
    s_encoder_calibration.state.reverse_total_count =
        s_encoder_calibration.reverse_count[
            ENCODER_CALIBRATION_FULL_STEP_COUNT] -
        s_encoder_calibration.reverse_count[0U];
    s_encoder_calibration.state.closure_error_q1 =
        s_encoder_calibration.state.forward_total_count +
        s_encoder_calibration.state.reverse_total_count -
        MOTOR_ENCODER_COUNTS_PER_REVOLUTION * 2L;

    if ((RotorAlignment_Absolute(
            s_encoder_calibration.state.forward_total_count -
            MOTOR_ENCODER_COUNTS_PER_REVOLUTION) >
         ENCODER_CALIBRATION_TOTAL_TOLERANCE) ||
        (RotorAlignment_Absolute(
            s_encoder_calibration.state.reverse_total_count -
            MOTOR_ENCODER_COUNTS_PER_REVOLUTION) >
         ENCODER_CALIBRATION_TOTAL_TOLERANCE))
    {
        EncoderCalibration_FinishFault(
            ENCODER_CALIBRATION_FAULT_TOTAL_MOVEMENT);
        return;
    }

    for (index = 0U;
         index < ENCODER_CALIBRATION_BOUNDARY_COUNT;
         index++)
    {
        int32_t hysteresis_count = RotorAlignment_Absolute(
            s_encoder_calibration.forward_count[index] -
            s_encoder_calibration.reverse_count[index]);

        s_encoder_calibration.midpoint_q1[index] =
            s_encoder_calibration.forward_count[index] +
            s_encoder_calibration.reverse_count[index];
        if (hysteresis_count > maximum_hysteresis_count)
        {
            maximum_hysteresis_count = hysteresis_count;
        }

        if (index > 0U)
        {
            int32_t interval_q1 =
                s_encoder_calibration.midpoint_q1[index] -
                s_encoder_calibration.midpoint_q1[index - 1U];

            if (interval_q1 < minimum_interval_q1)
            {
                minimum_interval_q1 = interval_q1;
            }
            if (interval_q1 > maximum_interval_q1)
            {
                maximum_interval_q1 = interval_q1;
            }
        }
    }

    s_encoder_calibration.state.minimum_interval_q1 =
        minimum_interval_q1;
    s_encoder_calibration.state.maximum_interval_q1 =
        maximum_interval_q1;
    s_encoder_calibration.state.maximum_hysteresis_count =
        maximum_hysteresis_count;
    if (ElectricalAngle_SetNonlinearityCalibration(
            s_encoder_calibration.midpoint_q1,
            ENCODER_CALIBRATION_BOUNDARY_COUNT) == 0U)
    {
        EncoderCalibration_FinishFault(
            ENCODER_CALIBRATION_FAULT_TABLE_INVALID);
        return;
    }
    s_encoder_calibration.state.saturated_sample_count =
        PhaseCurrentLoop_GetState()->saturated_sample_count;
    PhaseCurrentLoop_Stop();
    s_encoder_calibration.state.fault = ENCODER_CALIBRATION_FAULT_NONE;
    s_encoder_calibration.state.data_valid = 1U;
    s_encoder_calibration.state.state = ENCODER_CALIBRATION_STATE_COMPLETE;
}

/* 执行一次编码器正反向整圈扫描标定状态机更新。 */
static void EncoderCalibration_Update(void)
{
    const volatile PhaseCurrentLoop_State_t *loop;
    uint32_t now_ms;
    uint32_t elapsed_ms;

    if (RotorAlignment_IsEncoderCalibrationRunning() == 0U)
    {
        return;
    }

    loop = PhaseCurrentLoop_GetState();
    if (loop->fault == PHASE_CURRENT_LOOP_FAULT_OVERCURRENT)
    {
        EncoderCalibration_FinishFault(
            ENCODER_CALIBRATION_FAULT_OVERCURRENT);
        return;
    }

    now_ms = HAL_GetTick();
    elapsed_ms = now_ms - s_encoder_calibration.state_start_tick_ms;
    switch (s_encoder_calibration.state.state)
    {
        case ENCODER_CALIBRATION_STATE_RAMP_CURRENT:
        {
            int32_t amplitude_ma = (int32_t)(
                ((uint32_t)ENCODER_CALIBRATION_CURRENT_MA * elapsed_ms) /
                ENCODER_CALIBRATION_CURRENT_RAMP_MS);

            if (elapsed_ms >= ENCODER_CALIBRATION_CURRENT_RAMP_MS)
            {
                amplitude_ma = ENCODER_CALIBRATION_CURRENT_MA;
                EncoderCalibration_EnterState(
                    ENCODER_CALIBRATION_STATE_HOLD_START, now_ms);
            }
            if (EncoderCalibration_SetVector(0.0f, amplitude_ma) == 0U)
            {
                EncoderCalibration_FinishFault(
                    ENCODER_CALIBRATION_FAULT_TARGET_UPDATE);
            }
        }
        break;

        case ENCODER_CALIBRATION_STATE_HOLD_START:
            if (elapsed_ms >= ENCODER_CALIBRATION_START_HOLD_MS)
            {
                s_encoder_calibration.state.start_position_count =
                    Encoder_GetState()->position_count;
                s_encoder_calibration.commanded_boundary = -1;
                s_encoder_calibration.state.boundary_index = 0U;
                EncoderCalibration_EnterState(
                    ENCODER_CALIBRATION_STATE_MOVE_PRELOAD_REVERSE, now_ms);
            }
            break;

        case ENCODER_CALIBRATION_STATE_MOVE_PRELOAD_REVERSE:
        {
            float progress = (float)elapsed_ms /
                             (float)ENCODER_CALIBRATION_MOVE_STEP_MS;

            if (progress > 1.0f)
            {
                progress = 1.0f;
            }
            if (EncoderCalibration_SetVector(-progress,
                                             ENCODER_CALIBRATION_CURRENT_MA) == 0U)
            {
                EncoderCalibration_FinishFault(
                    ENCODER_CALIBRATION_FAULT_TARGET_UPDATE);
                break;
            }
            if (elapsed_ms >= ENCODER_CALIBRATION_MOVE_STEP_MS)
            {
                EncoderCalibration_EnterState(
                    ENCODER_CALIBRATION_STATE_SETTLE_PRELOAD_REVERSE,
                    now_ms);
            }
        }
        break;

        case ENCODER_CALIBRATION_STATE_SETTLE_PRELOAD_REVERSE:
            if (elapsed_ms >= ENCODER_CALIBRATION_SETTLE_MS)
            {
                s_encoder_calibration.preload_position_count =
                    Encoder_GetState()->position_count;
                if (EncoderCalibration_IntervalIsValid(
                        s_encoder_calibration.state.start_position_count -
                        s_encoder_calibration.preload_position_count) == 0U)
                {
                    EncoderCalibration_FinishFault(
                        ENCODER_CALIBRATION_FAULT_REVERSE_INTERVAL);
                    break;
                }
                s_encoder_calibration.commanded_boundary = 0;
                EncoderCalibration_EnterState(
                    ENCODER_CALIBRATION_STATE_RETURN_FORWARD_ZERO, now_ms);
            }
            break;

        case ENCODER_CALIBRATION_STATE_RETURN_FORWARD_ZERO:
        {
            float progress = (float)elapsed_ms /
                             (float)ENCODER_CALIBRATION_MOVE_STEP_MS;

            if (progress > 1.0f)
            {
                progress = 1.0f;
            }
            if (EncoderCalibration_SetVector(-1.0f + progress,
                                             ENCODER_CALIBRATION_CURRENT_MA) == 0U)
            {
                EncoderCalibration_FinishFault(
                    ENCODER_CALIBRATION_FAULT_TARGET_UPDATE);
                break;
            }
            if (elapsed_ms >= ENCODER_CALIBRATION_MOVE_STEP_MS)
            {
                EncoderCalibration_EnterState(
                    ENCODER_CALIBRATION_STATE_SETTLE_FORWARD_ZERO, now_ms);
            }
        }
        break;

        case ENCODER_CALIBRATION_STATE_SETTLE_FORWARD_ZERO:
            if (elapsed_ms >= ENCODER_CALIBRATION_SETTLE_MS)
            {
                s_encoder_calibration.forward_count[0U] =
                    Encoder_GetState()->position_count;
                if (EncoderCalibration_IntervalIsValid(
                        s_encoder_calibration.forward_count[0U] -
                        s_encoder_calibration.preload_position_count) == 0U)
                {
                    EncoderCalibration_FinishFault(
                        ENCODER_CALIBRATION_FAULT_FORWARD_INTERVAL);
                    break;
                }
                s_encoder_calibration.state.start_position_count =
                    s_encoder_calibration.forward_count[0U];
                s_encoder_calibration.commanded_boundary = 1;
                s_encoder_calibration.state.boundary_index = 1U;
                EncoderCalibration_EnterState(
                    ENCODER_CALIBRATION_STATE_MOVE_FORWARD, now_ms);
            }
            break;

        case ENCODER_CALIBRATION_STATE_MOVE_FORWARD:
        {
            float progress = (float)elapsed_ms /
                             (float)ENCODER_CALIBRATION_MOVE_STEP_MS;
            float start_quarter = (float)(
                EncoderCalibration_QuarterModulo(
                    s_encoder_calibration.commanded_boundary - 1));

            if (progress > 1.0f)
            {
                progress = 1.0f;
            }
            if (EncoderCalibration_SetVector(start_quarter + progress,
                                             ENCODER_CALIBRATION_CURRENT_MA) == 0U)
            {
                EncoderCalibration_FinishFault(
                    ENCODER_CALIBRATION_FAULT_TARGET_UPDATE);
                break;
            }
            if (elapsed_ms >= ENCODER_CALIBRATION_MOVE_STEP_MS)
            {
                EncoderCalibration_EnterState(
                    ENCODER_CALIBRATION_STATE_SETTLE_FORWARD, now_ms);
            }
        }
        break;

        case ENCODER_CALIBRATION_STATE_SETTLE_FORWARD:
            if (elapsed_ms >= ENCODER_CALIBRATION_SETTLE_MS)
            {
                uint16_t index =
                    (uint16_t)s_encoder_calibration.commanded_boundary;
                int32_t interval_count;

                s_encoder_calibration.forward_count[index] =
                    Encoder_GetState()->position_count;
                interval_count =
                    s_encoder_calibration.forward_count[index] -
                    s_encoder_calibration.forward_count[index - 1U];
                if (EncoderCalibration_IntervalIsValid(interval_count) == 0U)
                {
                    EncoderCalibration_FinishFault(
                        ENCODER_CALIBRATION_FAULT_FORWARD_INTERVAL);
                    break;
                }

                if (index >= ENCODER_CALIBRATION_FULL_STEP_COUNT)
                {
                    s_encoder_calibration.commanded_boundary = 201;
                    s_encoder_calibration.state.boundary_index = 200U;
                    EncoderCalibration_EnterState(
                        ENCODER_CALIBRATION_STATE_MOVE_OVERSHOOT_FORWARD,
                        now_ms);
                }
                else
                {
                    s_encoder_calibration.commanded_boundary =
                        (int16_t)(index + 1U);
                    s_encoder_calibration.state.boundary_index = index + 1U;
                    EncoderCalibration_EnterState(
                        ENCODER_CALIBRATION_STATE_MOVE_FORWARD, now_ms);
                }
            }
            break;

        case ENCODER_CALIBRATION_STATE_MOVE_OVERSHOOT_FORWARD:
        {
            float progress = (float)elapsed_ms /
                             (float)ENCODER_CALIBRATION_MOVE_STEP_MS;

            if (progress > 1.0f)
            {
                progress = 1.0f;
            }
            if (EncoderCalibration_SetVector(progress,
                                             ENCODER_CALIBRATION_CURRENT_MA) == 0U)
            {
                EncoderCalibration_FinishFault(
                    ENCODER_CALIBRATION_FAULT_TARGET_UPDATE);
                break;
            }
            if (elapsed_ms >= ENCODER_CALIBRATION_MOVE_STEP_MS)
            {
                EncoderCalibration_EnterState(
                    ENCODER_CALIBRATION_STATE_SETTLE_OVERSHOOT_FORWARD,
                    now_ms);
            }
        }
        break;

        case ENCODER_CALIBRATION_STATE_SETTLE_OVERSHOOT_FORWARD:
            if (elapsed_ms >= ENCODER_CALIBRATION_SETTLE_MS)
            {
                s_encoder_calibration.overshoot_position_count =
                    Encoder_GetState()->position_count;
                if (EncoderCalibration_IntervalIsValid(
                        s_encoder_calibration.overshoot_position_count -
                        s_encoder_calibration.forward_count[
                            ENCODER_CALIBRATION_FULL_STEP_COUNT]) == 0U)
                {
                    EncoderCalibration_FinishFault(
                        ENCODER_CALIBRATION_FAULT_FORWARD_INTERVAL);
                    break;
                }
                s_encoder_calibration.commanded_boundary = 200;
                s_encoder_calibration.state.boundary_index = 200U;
                EncoderCalibration_EnterState(
                    ENCODER_CALIBRATION_STATE_MOVE_REVERSE, now_ms);
            }
            break;

        case ENCODER_CALIBRATION_STATE_MOVE_REVERSE:
        {
            float progress = (float)elapsed_ms /
                             (float)ENCODER_CALIBRATION_MOVE_STEP_MS;
            float start_quarter = (float)(
                EncoderCalibration_QuarterModulo(
                    s_encoder_calibration.commanded_boundary + 1));

            if (progress > 1.0f)
            {
                progress = 1.0f;
            }
            if (EncoderCalibration_SetVector(start_quarter - progress,
                                             ENCODER_CALIBRATION_CURRENT_MA) == 0U)
            {
                EncoderCalibration_FinishFault(
                    ENCODER_CALIBRATION_FAULT_TARGET_UPDATE);
                break;
            }
            if (elapsed_ms >= ENCODER_CALIBRATION_MOVE_STEP_MS)
            {
                EncoderCalibration_EnterState(
                    ENCODER_CALIBRATION_STATE_SETTLE_REVERSE, now_ms);
            }
        }
        break;

        case ENCODER_CALIBRATION_STATE_SETTLE_REVERSE:
            if (elapsed_ms >= ENCODER_CALIBRATION_SETTLE_MS)
            {
                uint16_t index =
                    (uint16_t)s_encoder_calibration.commanded_boundary;
                int32_t interval_count;

                s_encoder_calibration.reverse_count[index] =
                    Encoder_GetState()->position_count;
                interval_count = (index ==
                                  ENCODER_CALIBRATION_FULL_STEP_COUNT) ?
                    (s_encoder_calibration.overshoot_position_count -
                     s_encoder_calibration.reverse_count[index]) :
                    (s_encoder_calibration.reverse_count[index + 1U] -
                     s_encoder_calibration.reverse_count[index]);
                if (EncoderCalibration_IntervalIsValid(interval_count) == 0U)
                {
                    EncoderCalibration_FinishFault(
                        ENCODER_CALIBRATION_FAULT_REVERSE_INTERVAL);
                    break;
                }

                if (index == 0U)
                {
                    EncoderCalibration_FinishSuccess();
                }
                else
                {
                    s_encoder_calibration.commanded_boundary =
                        (int16_t)(index - 1U);
                    s_encoder_calibration.state.boundary_index =
                        (uint16_t)(index - 1U);
                    EncoderCalibration_EnterState(
                        ENCODER_CALIBRATION_STATE_MOVE_REVERSE, now_ms);
                }
            }
            break;

        default:
            EncoderCalibration_FinishFault(
                ENCODER_CALIBRATION_FAULT_TARGET_UPDATE);
            break;
    }
}

/* 监测母线稳定条件，满足后自动启动上电转子对齐。 */
static void RotorAlignment_UpdateAutoStart(void)
{
    const CurrentSense_State_t *current;
    const PowerMonitor_State_t *power;
    uint32_t now_ms;

    if ((s_rotor_alignment.state.auto_start_pending == 0U) ||
        (RotorAlignment_IsRunning() != 0U))
    {
        return;
    }

    current = CurrentSense_GetState();
    power = PowerMonitor_GetState();
    if ((current->offset_valid == 0U) ||
        (power->bus_voltage_mv < ROTOR_ALIGNMENT_BUS_MIN_MV) ||
        (power->bus_voltage_mv > ROTOR_ALIGNMENT_BUS_MAX_MV))
    {
        s_rotor_alignment.auto_bus_ready = 0U;
        return;
    }

    now_ms = HAL_GetTick();
    if (s_rotor_alignment.auto_bus_ready == 0U)
    {
        s_rotor_alignment.auto_bus_ready = 1U;
        s_rotor_alignment.auto_ready_start_tick_ms = now_ms;
        return;
    }

    if ((now_ms - s_rotor_alignment.auto_ready_start_tick_ms) <
        ROTOR_ALIGNMENT_AUTO_BUS_STABLE_MS)
    {
        return;
    }

    /* 仅尝试一次；失败后保留故障，禁止无人值守地反复驱动电机。 */
    (void)RotorAlignment_Start();
}

/* 初始化对齐与编码器标定状态；不立即输出电流。 */
void RotorAlignment_Init(void)
{
    s_rotor_alignment.state.state = ROTOR_ALIGNMENT_STATE_IDLE;
    s_rotor_alignment.state.fault = ROTOR_ALIGNMENT_FAULT_NONE;
    s_rotor_alignment.state.first_position_count = 0L;
    s_rotor_alignment.state.zero_position_count = 0L;
    s_rotor_alignment.state.movement_count = 0L;
    s_rotor_alignment.state.expected_abs_movement_count =
        MOTOR_ENCODER_COUNTS_PER_ELECTRICAL_CYCLE / 4L;
    s_rotor_alignment.state.alignment_offset_count = 0L;
    s_rotor_alignment.state.target_a_ma = 0L;
    s_rotor_alignment.state.target_b_ma = 0L;
    s_rotor_alignment.state.hold_90_mean_current_a_ma = 0L;
    s_rotor_alignment.state.hold_90_mean_current_b_ma = 0L;
    s_rotor_alignment.state.hold_zero_mean_current_a_ma = 0L;
    s_rotor_alignment.state.hold_zero_mean_current_b_ma = 0L;
    s_rotor_alignment.state.peak_abs_current_a_ma = 0L;
    s_rotor_alignment.state.peak_abs_current_b_ma = 0L;
    s_rotor_alignment.state.peak_abs_voltage_a_mv = 0L;
    s_rotor_alignment.state.peak_abs_voltage_b_mv = 0L;
    s_rotor_alignment.state.current_sample_count = 0U;
    s_rotor_alignment.state.saturated_sample_count = 0U;
    s_rotor_alignment.state.phase_b_axis_sign = 0;
    s_rotor_alignment.state.auto_start_pending = 1U;
    s_rotor_alignment.state.valid = 0U;
    s_rotor_alignment.position_sum = 0LL;
    s_rotor_alignment.current_a_sum = 0LL;
    s_rotor_alignment.current_b_sum = 0LL;
    s_rotor_alignment.position_sample_count = 0U;
    s_rotor_alignment.state_start_tick_ms = HAL_GetTick();
    s_rotor_alignment.last_average_tick_ms = s_rotor_alignment.state_start_tick_ms;
    s_rotor_alignment.auto_ready_start_tick_ms = 0U;
    s_rotor_alignment.auto_bus_ready = 0U;

    s_encoder_calibration.state.state = ENCODER_CALIBRATION_STATE_IDLE;
    s_encoder_calibration.state.fault = ENCODER_CALIBRATION_FAULT_NONE;
    s_encoder_calibration.state.boundary_index = 0U;
    s_encoder_calibration.state.start_position_count = 0L;
    s_encoder_calibration.state.forward_total_count = 0L;
    s_encoder_calibration.state.reverse_total_count = 0L;
    s_encoder_calibration.state.closure_error_q1 = 0L;
    s_encoder_calibration.state.minimum_interval_q1 = 0L;
    s_encoder_calibration.state.maximum_interval_q1 = 0L;
    s_encoder_calibration.state.maximum_hysteresis_count = 0L;
    s_encoder_calibration.state.saturated_sample_count = 0U;
    s_encoder_calibration.state.data_valid = 0U;
    s_encoder_calibration.state_start_tick_ms = 0U;
    s_encoder_calibration.preload_position_count = 0L;
    s_encoder_calibration.overshoot_position_count = 0L;
    s_encoder_calibration.commanded_boundary = 0U;
}

/* 在安全母线电压和空闲状态下启动转子电角零点对齐。 */
uint8_t RotorAlignment_Start(void)
{
    const CurrentSense_State_t *current = CurrentSense_GetState();
    const PowerMonitor_State_t *power;
    PhaseCurrentLoop_Config_t loop_config;
    uint32_t now_ms;

    if (RotorAlignment_IsRunning() != 0U)
    {
        return 0U;
    }

    /* 手动或自动启动一旦发生，本次上电周期均不再重复自动尝试。 */
    s_rotor_alignment.state.auto_start_pending = 0U;
    s_rotor_alignment.auto_bus_ready = 0U;

    /* 每次尝试都先撤销旧零偏，禁止沿用可能已经失效的增量编码器位置。 */
    ElectricalAngle_Invalidate();
    ElectricalAngle_InvalidateNonlinearityCalibration();
    s_rotor_alignment.state.fault = ROTOR_ALIGNMENT_FAULT_NONE;
    s_rotor_alignment.state.first_position_count = 0L;
    s_rotor_alignment.state.zero_position_count = 0L;
    s_rotor_alignment.state.movement_count = 0L;
    s_rotor_alignment.state.alignment_offset_count = 0L;
    s_rotor_alignment.state.target_a_ma = 0L;
    s_rotor_alignment.state.target_b_ma = 0L;
    s_rotor_alignment.state.hold_90_mean_current_a_ma = 0L;
    s_rotor_alignment.state.hold_90_mean_current_b_ma = 0L;
    s_rotor_alignment.state.hold_zero_mean_current_a_ma = 0L;
    s_rotor_alignment.state.hold_zero_mean_current_b_ma = 0L;
    s_rotor_alignment.state.peak_abs_current_a_ma = 0L;
    s_rotor_alignment.state.peak_abs_current_b_ma = 0L;
    s_rotor_alignment.state.peak_abs_voltage_a_mv = 0L;
    s_rotor_alignment.state.peak_abs_voltage_b_mv = 0L;
    s_rotor_alignment.state.current_sample_count = 0U;
    s_rotor_alignment.state.saturated_sample_count = 0U;
    s_rotor_alignment.state.phase_b_axis_sign = 0;
    s_rotor_alignment.state.valid = 0U;

    PowerMonitor_Update();
    power = PowerMonitor_GetState();
    if ((current->offset_valid == 0U) ||
        (power->bus_voltage_mv < ROTOR_ALIGNMENT_BUS_MIN_MV) ||
        (power->bus_voltage_mv > ROTOR_ALIGNMENT_BUS_MAX_MV))
    {
        s_rotor_alignment.state.state = ROTOR_ALIGNMENT_STATE_FAULT;
        s_rotor_alignment.state.fault = ROTOR_ALIGNMENT_FAULT_PRECONDITION;
        s_rotor_alignment.state.valid = 0U;
        return 0U;
    }

    loop_config.bandwidth_hz = ROTOR_ALIGNMENT_CURRENT_BANDWIDTH_HZ;
    loop_config.sample_frequency_hz = ROTOR_ALIGNMENT_CURRENT_LOOP_HZ;
    loop_config.maximum_voltage_mv = ROTOR_ALIGNMENT_VOLTAGE_LIMIT_MV;
    loop_config.maximum_target_current_ma = ROTOR_ALIGNMENT_CURRENT_MA;
    loop_config.overcurrent_limit_ma = ROTOR_ALIGNMENT_OVERCURRENT_MA;
    loop_config.bus_voltage_mv = power->bus_voltage_mv;
    if (PhaseCurrentLoop_Start(&loop_config) == 0U)
    {
        s_rotor_alignment.state.state = ROTOR_ALIGNMENT_STATE_FAULT;
        s_rotor_alignment.state.fault = ROTOR_ALIGNMENT_FAULT_PRECONDITION;
        s_rotor_alignment.state.valid = 0U;
        return 0U;
    }

    now_ms = HAL_GetTick();
    RotorAlignment_EnterState(ROTOR_ALIGNMENT_STATE_RAMP_TO_90_DEG, now_ms);
    if (RotorAlignment_SetTargets(0L, 0L) == 0U)
    {
        RotorAlignment_FinishFault(ROTOR_ALIGNMENT_FAULT_TARGET_UPDATE);
        return 0U;
    }
    return 1U;
}

/* 由主循环周期调用，推进转子对齐及编码器标定状态机。 */
void RotorAlignment_Update(void)
{
    const volatile PhaseCurrentLoop_State_t *loop;
    uint32_t now_ms;
    uint32_t elapsed_ms;

    RotorAlignment_UpdateAutoStart();
    EncoderCalibration_Update();
    if (RotorAlignment_IsEncoderCalibrationRunning() != 0U)
    {
        return;
    }
    if (RotorAlignment_IsRunning() == 0U)
    {
        return;
    }

    loop = PhaseCurrentLoop_GetState();
    if (loop->fault == PHASE_CURRENT_LOOP_FAULT_OVERCURRENT)
    {
        RotorAlignment_FinishFault(ROTOR_ALIGNMENT_FAULT_OVERCURRENT);
        return;
    }

    now_ms = HAL_GetTick();
    elapsed_ms = now_ms - s_rotor_alignment.state_start_tick_ms;
    switch (s_rotor_alignment.state.state)
    {
        case ROTOR_ALIGNMENT_STATE_RAMP_TO_90_DEG:
        {
            int32_t target_b_ma = (int32_t)(
                ((uint32_t)ROTOR_ALIGNMENT_CURRENT_MA * elapsed_ms) /
                ROTOR_ALIGNMENT_CURRENT_RAMP_MS);

            if (elapsed_ms >= ROTOR_ALIGNMENT_CURRENT_RAMP_MS)
            {
                target_b_ma = ROTOR_ALIGNMENT_CURRENT_MA;
                RotorAlignment_EnterState(
                    ROTOR_ALIGNMENT_STATE_HOLD_90_DEG, now_ms);
            }
            if (RotorAlignment_SetTargets(0L, target_b_ma) == 0U)
            {
                RotorAlignment_FinishFault(
                    ROTOR_ALIGNMENT_FAULT_TARGET_UPDATE);
            }
        }
        break;

        case ROTOR_ALIGNMENT_STATE_HOLD_90_DEG:
            if (elapsed_ms >=
                (ROTOR_ALIGNMENT_HOLD_MS - ROTOR_ALIGNMENT_AVERAGE_WINDOW_MS))
            {
                RotorAlignment_AccumulateMeasurements(now_ms);
            }
            if (elapsed_ms >= ROTOR_ALIGNMENT_HOLD_MS)
            {
                s_rotor_alignment.state.first_position_count =
                    RotorAlignment_AveragePosition();
                s_rotor_alignment.state.hold_90_mean_current_a_ma =
                    RotorAlignment_AverageCurrent(
                        s_rotor_alignment.current_a_sum);
                s_rotor_alignment.state.hold_90_mean_current_b_ma =
                    RotorAlignment_AverageCurrent(
                        s_rotor_alignment.current_b_sum);
                RotorAlignment_EnterState(
                    ROTOR_ALIGNMENT_STATE_ROTATE_TO_ZERO, now_ms);
            }
            break;

        case ROTOR_ALIGNMENT_STATE_ROTATE_TO_ZERO:
        {
            float progress = (float)elapsed_ms /
                             (float)ROTOR_ALIGNMENT_ROTATION_MS;
            float angle_rad;
            int32_t target_a_ma;
            int32_t target_b_ma;

            if (progress > 1.0f)
            {
                progress = 1.0f;
            }
            angle_rad = ROTOR_ALIGNMENT_HALF_PI_RAD * (1.0f - progress);
            target_a_ma = RotorAlignment_RoundFloat(
                (float)ROTOR_ALIGNMENT_CURRENT_MA * cosf(angle_rad));
            target_b_ma = RotorAlignment_RoundFloat(
                (float)ROTOR_ALIGNMENT_CURRENT_MA * sinf(angle_rad));
            if (elapsed_ms >= ROTOR_ALIGNMENT_ROTATION_MS)
            {
                target_a_ma = ROTOR_ALIGNMENT_CURRENT_MA;
                target_b_ma = 0L;
                RotorAlignment_EnterState(
                    ROTOR_ALIGNMENT_STATE_HOLD_ZERO, now_ms);
            }
            if (RotorAlignment_SetTargets(target_a_ma, target_b_ma) == 0U)
            {
                RotorAlignment_FinishFault(
                    ROTOR_ALIGNMENT_FAULT_TARGET_UPDATE);
            }
        }
        break;

        case ROTOR_ALIGNMENT_STATE_HOLD_ZERO:
            if (elapsed_ms >=
                (ROTOR_ALIGNMENT_HOLD_MS - ROTOR_ALIGNMENT_AVERAGE_WINDOW_MS))
            {
                RotorAlignment_AccumulateMeasurements(now_ms);
            }
            if (elapsed_ms >= ROTOR_ALIGNMENT_HOLD_MS)
            {
                int32_t abs_movement;
                int32_t minimum_movement;
                int32_t maximum_movement;

                s_rotor_alignment.state.zero_position_count =
                    RotorAlignment_AveragePosition();
                s_rotor_alignment.state.hold_zero_mean_current_a_ma =
                    RotorAlignment_AverageCurrent(
                        s_rotor_alignment.current_a_sum);
                s_rotor_alignment.state.hold_zero_mean_current_b_ma =
                    RotorAlignment_AverageCurrent(
                        s_rotor_alignment.current_b_sum);
                s_rotor_alignment.state.movement_count =
                    RotorAlignment_WrapMechanicalDelta(
                        s_rotor_alignment.state.zero_position_count -
                        s_rotor_alignment.state.first_position_count);
                abs_movement = RotorAlignment_Absolute(
                    s_rotor_alignment.state.movement_count);
                minimum_movement =
                    s_rotor_alignment.state.expected_abs_movement_count -
                    ROTOR_ALIGNMENT_MOVEMENT_TOLERANCE;
                maximum_movement =
                    s_rotor_alignment.state.expected_abs_movement_count +
                    ROTOR_ALIGNMENT_MOVEMENT_TOLERANCE;
                if ((abs_movement < minimum_movement) ||
                    (abs_movement > maximum_movement))
                {
                    RotorAlignment_FinishFault(
                        ROTOR_ALIGNMENT_FAULT_MOVEMENT);
                }
                else
                {
                    RotorAlignment_FinishSuccess();
                }
            }
            break;

        default:
            RotorAlignment_FinishFault(ROTOR_ALIGNMENT_FAULT_TARGET_UPDATE);
            break;
    }
}

/* 中止正在进行的转子对齐，并关闭相电流输出。 */
void RotorAlignment_Abort(void)
{
    if (RotorAlignment_IsRunning() == 0U)
    {
        return;
    }
    RotorAlignment_FinishFault(ROTOR_ALIGNMENT_FAULT_ABORTED);
}

/* 查询主转子对齐状态机是否仍在运行。 */
uint8_t RotorAlignment_IsRunning(void)
{
    RotorAlignment_StateCode_t state = s_rotor_alignment.state.state;

    return ((state == ROTOR_ALIGNMENT_STATE_RAMP_TO_90_DEG) ||
            (state == ROTOR_ALIGNMENT_STATE_HOLD_90_DEG) ||
            (state == ROTOR_ALIGNMENT_STATE_ROTATE_TO_ZERO) ||
            (state == ROTOR_ALIGNMENT_STATE_HOLD_ZERO)) ? 1U : 0U;
}

/* 返回主对齐诊断状态快照的只读指针。 */
const RotorAlignment_State_t *RotorAlignment_GetState(void)
{
    return &s_rotor_alignment.state;
}

/* 启动编码器整圈边界/非线性采样标定。 */
uint8_t RotorAlignment_StartEncoderCalibration(void)
{
    const CurrentSense_State_t *current = CurrentSense_GetState();
    const PowerMonitor_State_t *power;
    PhaseCurrentLoop_Config_t loop_config;

    if ((RotorAlignment_IsEncoderCalibrationRunning() != 0U) ||
        (RotorAlignment_IsRunning() != 0U) ||
        (s_rotor_alignment.state.valid == 0U) ||
        (MotorPWM_IsEnabled() != 0U))
    {
        return 0U;
    }

    PowerMonitor_Update();
    power = PowerMonitor_GetState();
    if ((current->offset_valid == 0U) ||
        (power->bus_voltage_mv < ROTOR_ALIGNMENT_BUS_MIN_MV) ||
        (power->bus_voltage_mv > ROTOR_ALIGNMENT_BUS_MAX_MV))
    {
        s_encoder_calibration.state.state =
            ENCODER_CALIBRATION_STATE_FAULT;
        s_encoder_calibration.state.fault =
            ENCODER_CALIBRATION_FAULT_PRECONDITION;
        s_encoder_calibration.state.data_valid = 0U;
        return 0U;
    }

    /* 新一轮采集开始即撤销旧表，任何中止都回退到原始角度。 */
    ElectricalAngle_InvalidateNonlinearityCalibration();

    loop_config.bandwidth_hz = ROTOR_ALIGNMENT_CURRENT_BANDWIDTH_HZ;
    loop_config.sample_frequency_hz = ROTOR_ALIGNMENT_CURRENT_LOOP_HZ;
    loop_config.maximum_voltage_mv = ROTOR_ALIGNMENT_VOLTAGE_LIMIT_MV;
    loop_config.maximum_target_current_ma = ENCODER_CALIBRATION_CURRENT_MA;
    loop_config.overcurrent_limit_ma = ROTOR_ALIGNMENT_OVERCURRENT_MA;
    loop_config.bus_voltage_mv = power->bus_voltage_mv;
    if (PhaseCurrentLoop_Start(&loop_config) == 0U)
    {
        s_encoder_calibration.state.state =
            ENCODER_CALIBRATION_STATE_FAULT;
        s_encoder_calibration.state.fault =
            ENCODER_CALIBRATION_FAULT_PRECONDITION;
        s_encoder_calibration.state.data_valid = 0U;
        return 0U;
    }

    s_encoder_calibration.state.fault = ENCODER_CALIBRATION_FAULT_NONE;
    s_encoder_calibration.state.boundary_index = 0U;
    s_encoder_calibration.state.start_position_count = 0L;
    s_encoder_calibration.state.forward_total_count = 0L;
    s_encoder_calibration.state.reverse_total_count = 0L;
    s_encoder_calibration.state.closure_error_q1 = 0L;
    s_encoder_calibration.state.minimum_interval_q1 = 0L;
    s_encoder_calibration.state.maximum_interval_q1 = 0L;
    s_encoder_calibration.state.maximum_hysteresis_count = 0L;
    s_encoder_calibration.state.saturated_sample_count = 0U;
    s_encoder_calibration.state.data_valid = 0U;
    s_encoder_calibration.preload_position_count = 0L;
    s_encoder_calibration.overshoot_position_count = 0L;
    s_encoder_calibration.commanded_boundary = 0U;
    EncoderCalibration_EnterState(
        ENCODER_CALIBRATION_STATE_RAMP_CURRENT, HAL_GetTick());
    if (EncoderCalibration_SetVector(0.0f, 0L) == 0U)
    {
        EncoderCalibration_FinishFault(
            ENCODER_CALIBRATION_FAULT_TARGET_UPDATE);
        return 0U;
    }
    return 1U;
}

/* 中止编码器标定并停止用于扫描的相电流。 */
void RotorAlignment_AbortEncoderCalibration(void)
{
    if (RotorAlignment_IsEncoderCalibrationRunning() != 0U)
    {
        EncoderCalibration_FinishFault(
            ENCODER_CALIBRATION_FAULT_ABORTED);
    }
}

/* 查询编码器标定状态机是否正在运行。 */
uint8_t RotorAlignment_IsEncoderCalibrationRunning(void)
{
    EncoderCalibration_StateCode_t state =
        s_encoder_calibration.state.state;

    return ((state == ENCODER_CALIBRATION_STATE_RAMP_CURRENT) ||
            (state == ENCODER_CALIBRATION_STATE_HOLD_START) ||
            (state == ENCODER_CALIBRATION_STATE_MOVE_PRELOAD_REVERSE) ||
            (state == ENCODER_CALIBRATION_STATE_SETTLE_PRELOAD_REVERSE) ||
            (state == ENCODER_CALIBRATION_STATE_RETURN_FORWARD_ZERO) ||
            (state == ENCODER_CALIBRATION_STATE_SETTLE_FORWARD_ZERO) ||
            (state == ENCODER_CALIBRATION_STATE_MOVE_FORWARD) ||
            (state == ENCODER_CALIBRATION_STATE_SETTLE_FORWARD) ||
            (state == ENCODER_CALIBRATION_STATE_MOVE_OVERSHOOT_FORWARD) ||
            (state == ENCODER_CALIBRATION_STATE_SETTLE_OVERSHOOT_FORWARD) ||
            (state == ENCODER_CALIBRATION_STATE_MOVE_REVERSE) ||
            (state == ENCODER_CALIBRATION_STATE_SETTLE_REVERSE)) ? 1U : 0U;
}

const EncoderCalibration_State_t *
/* 返回编码器标定过程和结果的只读诊断状态。 */
RotorAlignment_GetEncoderCalibrationState(void)
{
    return &s_encoder_calibration.state;
}

/* 读取指定边界点的正反向编码器采样值，供串口导出分析。 */
uint8_t RotorAlignment_GetEncoderCalibrationPoint(
    uint16_t index,
    int32_t *forward_count,
    int32_t *reverse_count,
    int32_t *midpoint_q1)
{
    if ((index >= ENCODER_CALIBRATION_BOUNDARY_COUNT) ||
        (forward_count == 0) ||
        (reverse_count == 0) ||
        (midpoint_q1 == 0) ||
        (s_encoder_calibration.state.data_valid == 0U))
    {
        return 0U;
    }

    *forward_count = s_encoder_calibration.forward_count[index];
    *reverse_count = s_encoder_calibration.reverse_count[index];
    *midpoint_q1 = s_encoder_calibration.midpoint_q1[index];
    return 1U;
}
