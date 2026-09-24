#include "debug.h"

#include "current_sense.h"
#include "dq_current_loop.h"
#include "electrical_angle.h"
#include "encoder.h"
#include "motor_pwm.h"
#include "motor_test.h"
#include "power_monitor.h"
#include "position_loop.h"
#include "pulse_input.h"
#include "rotor_alignment.h"
#include "speed_loop.h"
#include "usbd_cdc_if.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#define DEBUG_RX_BUFFER_SIZE       (64U)
#define DEBUG_TX_BUFFER_SIZE       (576U)
#define DEBUG_STREAM_PERIOD_MS     (100U)
#define DEBUG_SPEED_STREAM_PERIOD_MS (20U)
#define DEBUG_VOFA_CHANNEL_COUNT   (8U)
#define DEBUG_VOFA_FRAME_SIZE      ((DEBUG_VOFA_CHANNEL_COUNT + 1U) * sizeof(float))
#define DEBUG_VOFA_SPEED_CHANNEL_COUNT (16U)
#define DEBUG_VOFA_SPEED_FRAME_SIZE \
    ((DEBUG_VOFA_SPEED_CHANNEL_COUNT + 1U) * sizeof(float))
#define DEBUG_VOFA_SPEED_BASIC_CHANNEL_COUNT (5U)
#define DEBUG_VOFA_SPEED_BASIC_FRAME_SIZE \
    ((DEBUG_VOFA_SPEED_BASIC_CHANNEL_COUNT + 1U) * sizeof(float))
#define DEBUG_VOFA_POSITION_CHANNEL_COUNT (5U)
#define DEBUG_VOFA_POSITION_FRAME_SIZE \
    ((DEBUG_VOFA_POSITION_CHANNEL_COUNT + 1U) * sizeof(float))
#define DEBUG_VOFA_POSITION_DIAGNOSTIC_CHANNEL_COUNT (16U)
#define DEBUG_VOFA_POSITION_DIAGNOSTIC_FRAME_SIZE \
    ((DEBUG_VOFA_POSITION_DIAGNOSTIC_CHANNEL_COUNT + 1U) * sizeof(float))
/* 脉冲模式实时诊断：时间、序号、位置/速度/电流及饱和状态，共19路。 */
#define DEBUG_VOFA_PULSE_CHANNEL_COUNT (19U)
#define DEBUG_VOFA_PULSE_FRAME_SIZE \
    ((DEBUG_VOFA_PULSE_CHANNEL_COUNT + 1U) * sizeof(float))
/* 位置采集帧含时间、序号和到位状态，供PC端可靠结束长行程采集。 */
#define DEBUG_VOFA_POSITION_CAPTURE_CHANNEL_COUNT (9U)
#define DEBUG_VOFA_POSITION_CAPTURE_FRAME_SIZE \
    ((DEBUG_VOFA_POSITION_CAPTURE_CHANNEL_COUNT + 1U) * sizeof(float))
#define DEBUG_VOFA_FRAMES_PER_USB_PACKET (1U)
/* 发送缓存必须覆盖所有JustFloat帧；脉冲诊断帧是当前最长帧。 */
#define DEBUG_VOFA_TX_BUFFER_SIZE  (DEBUG_VOFA_PULSE_FRAME_SIZE)
#define DEBUG_SPEED_TEST_TARGET_MRPM       (60000L)
#define DEBUG_SPEED_TEST_CURRENT_VECTOR_LIMIT_MA (3500L)
/*
 * 高速弱磁验证开放至铭牌参考目标3000rpm；运行保护额外保留100rpm，
 * 每一档仍须读取实时数据后再决定是否进入下一档。
 */
#define DEBUG_SPEED_COMMAND_MAXIMUM_RPM    (3000L)
#define DEBUG_SPEED_TEST_MAXIMUM_MRPM      (3100000L)
/* 延长速度环测试时间，以便观察瞬时波动及振荡是否持续或衰减。 */
#define DEBUG_SPEED_TEST_DURATION_MS       (20000U)
#define DEBUG_SPEED_TEST_TORQUE_SIGN       (1)
/* 真实阶跃测试先在初始速度稳定3s，再切换到目标速度。 */
#define DEBUG_SPEED_STEP_PRECONDITION_MS   (3000U)
/*
 * 编码器坐标已经在边界统一为“从输出轴端看逆时针为正”。本硬件正Iq
 * 产生逆时针转矩，速度环到电流环不需要额外取反。
 */
#define DEBUG_POSITION_TEST_MAXIMUM_MRPM   (200000L)
#define DEBUG_PULSE_INPUT_MAXIMUM_MRPM     (3000000L)
#define DEBUG_POSITION_TEST_MAXIMUM_ACCELERATION_MRPM_PER_S (10000000L)
#define DEBUG_POSITION_TRACKING_RESERVE_MIN_MRPM (20000L)
/*
 * 高加速度轨迹中，位置P项必须有足够的速度追赶余量。
 * 参考工程的全局速度限幅约为其梯形轨迹峰值的两倍；此处采用同量级的
 * 100%余量，最终仍受3.1krpm总速度限幅和速度环电流圆保护。
 */
#define DEBUG_POSITION_TRACKING_RESERVE_PERMILLE (1000L)
/* 已完成200~3000rpm位置验证的默认位置PD。 */
#define DEBUG_POSITION_TEST_KP_MRPM_PER_COUNT (2400L)
#define DEBUG_POSITION_TEST_KD_MRPM_PER_RPM   (50L)
/* 高加速度终点整定允许在不改变控制结构的前提下扫描位置P增益。 */
#define DEBUG_POSITION_KP_MAX_MRPM_PER_COUNT (2400L)
#define DEBUG_POSITION_KD_MAX_MRPM_PER_RPM   (2000L)
#define DEBUG_POSITION_SPEED_LIMIT_MIN_MRPM  (1000L)
#define DEBUG_POSITION_SPEED_LIMIT_MAX_MRPM  (3000000L)
#define DEBUG_POSITION_TEST_TOLERANCE_COUNT (4L)
#define DEBUG_POSITION_TEST_SETTLE_SPEED_MRPM (30000L)
#define DEBUG_POSITION_TEST_SETTLE_TIME_MS  (50U)
#define DEBUG_POSITION_TEST_MAXIMUM_ERROR_COUNT (1000L)
#define DEBUG_POSITION_TEST_ERROR_TIMEOUT_MS (100U)
/* 位置保持需要持续运行，底层速度环仍保留有限的溢出保护时限。 */
#define DEBUG_POSITION_TEST_MAXIMUM_RUN_TIME_MS (UINT32_MAX)
#define DEBUG_SPEED_KP_MAX_UA_PER_RPM       (20000L)
#define DEBUG_SPEED_KI_MAX_UA_PER_RPM_S     (500000L)
/* 1000rpm恒Id/Iq诊断：7s锁存、保持2s，越出800~1200rpm立即停机。 */
#define DEBUG_SPEED_HOLD_START_MS            (7000U)
#define DEBUG_SPEED_HOLD_DURATION_MS         (2000U)
#define DEBUG_SPEED_HOLD_MINIMUM_MRPM        (800000L)
#define DEBUG_SPEED_HOLD_MAXIMUM_MRPM       (1200000L)
#define DEBUG_ELECTRICAL_PHASE_INDEXES         (1024L)
#define DEBUG_ELECTRICAL_HALF_PHASE_INDEXES     (512L)
#define DEBUG_ELECTRICAL_CIRCLE_MDEG         (360000L)

typedef struct
{
    char rx_buffer[DEBUG_RX_BUFFER_SIZE];
    volatile uint16_t rx_length;
    volatile uint8_t command_ready;
    volatile uint8_t stop_requested;
    uint8_t stream_enabled;
    uint32_t last_stream_tick_ms;
    uint16_t trace_dump_index;
    uint16_t trace_dump_count;
    uint8_t trace_dump_active;
    uint8_t trace_header_sent;
    uint8_t trace_perf_pending;
    uint16_t calibration_dump_index;
    uint8_t calibration_dump_active;
    uint8_t calibration_header_sent;
    char trace_perf_buffer[DEBUG_TX_BUFFER_SIZE];
    char trace_fall_buffer[DEBUG_TX_BUFFER_SIZE];
    char pending_text[DEBUG_TX_BUFFER_SIZE];
    uint8_t pending_text_valid;
} Debug_Context_t;

typedef struct
{
    uint8_t tx_buffer[DEBUG_VOFA_TX_BUFFER_SIZE];
    uint16_t trace_index;
    uint16_t trace_count;
    uint8_t test_fault;
    uint8_t active;
    uint8_t live_mode;
    uint8_t speed_live_mode;
    uint8_t speed_position_mode;
    uint8_t speed_basic_mode;
    uint8_t speed_step_active;
    uint32_t speed_step_switch_ms;
    int32_t speed_step_target_mrpm;
    uint8_t position_live_mode;
    uint8_t position_diagnostic_mode;
    uint8_t position_capture_mode;
    uint32_t position_capture_sequence;
    uint8_t pulse_live_mode;
    uint32_t pulse_sequence;
    uint32_t pulse_last_sample_tick_ms;
    uint8_t transfer_in_progress;
} Debug_VofaContext_t;

typedef struct
{
    int32_t kp_ua_per_rpm;
    int32_t ki_ua_per_rpm_s;
    int32_t target_speed_mrpm;
} Debug_SpeedTuneContext_t;

typedef struct
{
    int32_t kp_mrpm_per_count;
    int32_t kd_mrpm_per_rpm;
    int32_t maximum_speed_mrpm;
} Debug_PositionTuneContext_t;

static Debug_Context_t s_debug;
static Debug_VofaContext_t s_vofa;
static Debug_SpeedTuneContext_t s_speed_tune;
static Debug_PositionTuneContext_t s_position_tune;
static char s_debug_tx_buffer[DEBUG_TX_BUFFER_SIZE];

static const char *Debug_GetAngleObserverModeName(
    ElectricalAngle_ObserverMode_t mode)
{
    switch (mode)
    {
        case ELECTRICAL_ANGLE_OBSERVER_OFF:
            return "off";

        case ELECTRICAL_ANGLE_OBSERVER_SHADOW:
            return "shadow";

        case ELECTRICAL_ANGLE_OBSERVER_ON:
            return "on";

        case ELECTRICAL_ANGLE_OBSERVER_CALIBRATED_PREDICTIVE:
            return "calibrated";

        default:
            return "invalid";
    }
}

/*
 * Shadow观测器不参与当前控制，只用于量化整数编码器电角度相对连续预测角的
 * 瞬时差值。该值越大，整数控制角在高速下的离散误差越明显。
 */
static int32_t Debug_CalculateAngleQuantizationErrorMdeg(
    const volatile DqCurrentLoop_State_t *current)
{
    int32_t phase_error = current->raw_phase_index -
                          current->observer_phase_index;

    if (phase_error > DEBUG_ELECTRICAL_HALF_PHASE_INDEXES)
    {
        phase_error -= DEBUG_ELECTRICAL_PHASE_INDEXES;
    }
    else if (phase_error < -DEBUG_ELECTRICAL_HALF_PHASE_INDEXES)
    {
        phase_error += DEBUG_ELECTRICAL_PHASE_INDEXES;
    }

    return (int32_t)(((int64_t)phase_error *
                      DEBUG_ELECTRICAL_CIRCLE_MDEG) /
                     DEBUG_ELECTRICAL_PHASE_INDEXES);
}

static int32_t Debug_PhaseIndexToMdeg(int32_t phase_index)
{
    return (int32_t)(((int64_t)phase_index *
                      DEBUG_ELECTRICAL_CIRCLE_MDEG) /
                     DEBUG_ELECTRICAL_PHASE_INDEXES);
}

static void Debug_VofaWriteFloat(uint8_t *destination, float value)
{
    /* memcpy规避指针强制转换带来的严格别名和未对齐访问问题。 */
    (void)memcpy(destination, &value, sizeof(value));
}

static void Debug_VofaBuildFrame(uint8_t *destination,
                                 const MotorTest_DqTraceSample_t *sample,
                                 uint8_t fault)
{
    static const uint8_t just_float_tail[sizeof(float)] =
        {0x00U, 0x00U, 0x80U, 0x7FU};

    Debug_VofaWriteFloat(&destination[0U * sizeof(float)],
                         (float)sample->target_d_ma);
    Debug_VofaWriteFloat(&destination[1U * sizeof(float)],
                         (float)sample->measured_d_ma);
    Debug_VofaWriteFloat(&destination[2U * sizeof(float)],
                         (float)sample->target_q_ma);
    Debug_VofaWriteFloat(&destination[3U * sizeof(float)],
                         (float)sample->measured_q_ma);
    Debug_VofaWriteFloat(&destination[4U * sizeof(float)],
                         (float)sample->output_d_mv);
    Debug_VofaWriteFloat(&destination[5U * sizeof(float)],
                         (float)sample->output_q_mv);
    Debug_VofaWriteFloat(&destination[6U * sizeof(float)],
                         (float)sample->voltage_saturated);
    Debug_VofaWriteFloat(&destination[7U * sizeof(float)],
                         (float)fault);
    (void)memcpy(&destination[DEBUG_VOFA_CHANNEL_COUNT * sizeof(float)],
                 just_float_tail,
                 sizeof(just_float_tail));
}

static void Debug_VofaBuildSpeedFrame(
    uint8_t *destination,
    const SpeedLoop_TraceSample_t *sample,
    uint8_t position_mode)
{
    static const uint8_t just_float_tail[sizeof(float)] =
        {0x00U, 0x00U, 0x80U, 0x7FU};

    /*
     * 速度帧诊断阶段固定为16路：通道2为速度PI实际使用的固定5ms速度，
     * 通道4保留固定10ms测速，两路逐点对照可检查测速窗口的瞬态差异；通道5/9
     * 为d/q前馈电压，结合通道10/11总电压可还原PI纠偏量。
     * 位置相关性模式下，通道14改为机械一圈内计数（0~3999）。
     * 通道15为d轴优先分配的累计触发次数。
     */
    Debug_VofaWriteFloat(&destination[0U * sizeof(float)],
                         (float)sample->elapsed_ms);
    Debug_VofaWriteFloat(&destination[1U * sizeof(float)],
                         (float)sample->target_speed_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[2U * sizeof(float)],
                         (float)sample->fixed_5ms_speed_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[3U * sizeof(float)],
                         (float)sample->current_loop_iq_target_ma);
    Debug_VofaWriteFloat(&destination[4U * sizeof(float)],
                         (float)sample->fixed_10ms_speed_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[5U * sizeof(float)],
                         (float)sample->feedforward_d_mv);
    Debug_VofaWriteFloat(&destination[6U * sizeof(float)],
                         (float)sample->iq_target_ma);
    Debug_VofaWriteFloat(&destination[7U * sizeof(float)],
                         (float)sample->iq_measured_ma);
    Debug_VofaWriteFloat(&destination[8U * sizeof(float)],
                         (float)sample->id_measured_ma);
    Debug_VofaWriteFloat(&destination[9U * sizeof(float)],
                         (float)sample->feedforward_q_mv);
    Debug_VofaWriteFloat(&destination[10U * sizeof(float)],
                         (float)sample->output_d_mv);
    Debug_VofaWriteFloat(&destination[11U * sizeof(float)],
                         (float)sample->output_q_mv);
    Debug_VofaWriteFloat(&destination[12U * sizeof(float)],
                         (float)sample->voltage_saturated_sample_count);
    Debug_VofaWriteFloat(&destination[13U * sizeof(float)],
                         (float)sample->id_target_ma);
    Debug_VofaWriteFloat(&destination[14U * sizeof(float)],
                         (position_mode != 0U) ?
                         (float)sample->mechanical_count_in_revolution :
                         (float)sample->field_weakening_voltage_correction_mv);
    Debug_VofaWriteFloat(&destination[15U * sizeof(float)],
                         (float)sample->d_axis_priority_sample_count);
    (void)memcpy(&destination[DEBUG_VOFA_SPEED_CHANNEL_COUNT * sizeof(float)],
                 just_float_tail,
                 sizeof(just_float_tail));
}

/*
 * 速度-电流双闭环JustFloat固定五路：Ia、Ib、Iq、Speed_ref、Speed_fdb。
 * 电流单位为A，速度单位为rpm；均为控制时刻的实时快照而非平均值。
 */
static void Debug_VofaBuildSpeedBasicFrame(uint8_t *destination)
{
    const CurrentSense_State_t *phase_current = CurrentSense_GetState();
    const volatile DqCurrentLoop_State_t *dq_current = DqCurrentLoop_GetState();
    const volatile SpeedLoop_State_t *speed = SpeedLoop_GetState();
    static const uint8_t just_float_tail[sizeof(float)] =
        {0x00U, 0x00U, 0x80U, 0x7FU};

    Debug_VofaWriteFloat(&destination[0U * sizeof(float)],
                         (float)phase_current->current_a_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[1U * sizeof(float)],
                         (float)phase_current->current_b_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[2U * sizeof(float)],
                         (float)dq_current->measured_q_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[3U * sizeof(float)],
                         (float)speed->target_speed_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[4U * sizeof(float)],
                         (float)speed->measured_speed_mrpm / 1000.0f);
    (void)memcpy(&destination[DEBUG_VOFA_SPEED_BASIC_CHANNEL_COUNT * sizeof(float)],
                 just_float_tail,
                 sizeof(just_float_tail));
}

/*
 * 位置环JustFloat固定五路：轨迹Pos_ref、Pos_fdb、Speed_ref、Speed_fdb、
 * Iq_ref。位置单位为编码器计数；速度单位为rpm；Iq单位为A。
 */
static void Debug_VofaBuildPositionFrame(uint8_t *destination)
{
    const volatile PositionLoop_State_t *position = PositionLoop_GetState();
    const volatile SpeedLoop_State_t *speed = SpeedLoop_GetState();
    static const uint8_t just_float_tail[sizeof(float)] =
        {0x00U, 0x00U, 0x80U, 0x7FU};

    Debug_VofaWriteFloat(&destination[0U * sizeof(float)],
                         (float)position->reference_position_count);
    Debug_VofaWriteFloat(&destination[1U * sizeof(float)],
                         (float)position->measured_position_count);
    Debug_VofaWriteFloat(&destination[2U * sizeof(float)],
                         (float)position->speed_target_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[3U * sizeof(float)],
                         (float)speed->measured_speed_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[4U * sizeof(float)],
                         (float)speed->current_loop_iq_target_ma / 1000.0f);
    (void)memcpy(&destination[DEBUG_VOFA_POSITION_CHANNEL_COUNT * sizeof(float)],
                 just_float_tail,
                 sizeof(just_float_tail));
}

/*
 * 位置轨迹动态诊断帧（16路JustFloat）：
 * 0~4为标准位置帧；5~15依次为Iq目标/实测、Id目标/实测、Ud、Uq、
 * 电压饱和、弱磁、等效Iq、位置误差、速度误差。仅用于观察，不参与控制。
 */
static void Debug_VofaBuildPositionDiagnosticFrame(uint8_t *destination)
{
    const volatile PositionLoop_State_t *position = PositionLoop_GetState();
    const volatile SpeedLoop_State_t *speed = SpeedLoop_GetState();
    const volatile DqCurrentLoop_State_t *current = DqCurrentLoop_GetState();
    static const uint8_t just_float_tail[sizeof(float)] =
        {0x00U, 0x00U, 0x80U, 0x7FU};

    Debug_VofaWriteFloat(&destination[0U * sizeof(float)],
                         (float)position->reference_position_count);
    Debug_VofaWriteFloat(&destination[1U * sizeof(float)],
                         (float)position->measured_position_count);
    Debug_VofaWriteFloat(&destination[2U * sizeof(float)],
                         (float)position->speed_target_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[3U * sizeof(float)],
                         (float)speed->measured_speed_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[4U * sizeof(float)],
                         (float)speed->logical_iq_target_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[5U * sizeof(float)],
                         (float)current->target_q_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[6U * sizeof(float)],
                         (float)current->measured_q_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[7U * sizeof(float)],
                         (float)current->target_d_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[8U * sizeof(float)],
                         (float)current->measured_d_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[9U * sizeof(float)],
                         (float)current->output_d_mv / 1000.0f);
    Debug_VofaWriteFloat(&destination[10U * sizeof(float)],
                         (float)current->output_q_mv / 1000.0f);
    Debug_VofaWriteFloat(&destination[11U * sizeof(float)],
                         (float)current->voltage_saturated);
    Debug_VofaWriteFloat(&destination[12U * sizeof(float)],
                         (float)speed->field_weakening_active);
    Debug_VofaWriteFloat(&destination[13U * sizeof(float)],
                         (float)speed->torque_equivalent_iq_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[14U * sizeof(float)],
                         (float)position->position_error_count);
    Debug_VofaWriteFloat(&destination[15U * sizeof(float)],
                         (float)speed->speed_error_mrpm / 1000.0f);
    (void)memcpy(&destination[DEBUG_VOFA_POSITION_DIAGNOSTIC_CHANNEL_COUNT *
                                  sizeof(float)],
                 just_float_tail,
                 sizeof(just_float_tail));
}

/*
 * 脉冲模式专用实时帧，固定1kHz发送。
 * 通道：time_ms、seq、Pos_ref、Pos_fdb、Pos_err、PulseSpeedRaw、
 * PulseSpeedFF、Speed_ref、Speed_fdb、Speed_err、Iq_logic、Iq_ref、Iq_fdb、
 * SpeedPI_P、SpeedPI_I、SpeedPI_sat、Voltage_sat、FW_active、accepted_pulse_count。
 */
static void Debug_VofaBuildPulseFrame(uint8_t *destination)
{
    const volatile PulseInput_State_t *pulse = PulseInput_GetState();
    const volatile PositionLoop_State_t *position = PositionLoop_GetState();
    const volatile SpeedLoop_State_t *speed = SpeedLoop_GetState();
    const volatile DqCurrentLoop_State_t *current = DqCurrentLoop_GetState();
    static const uint8_t just_float_tail[sizeof(float)] =
        {0x00U, 0x00U, 0x80U, 0x7FU};

    Debug_VofaWriteFloat(&destination[0U * sizeof(float)], (float)HAL_GetTick());
    Debug_VofaWriteFloat(&destination[1U * sizeof(float)],
                         (float)s_vofa.pulse_sequence);
    Debug_VofaWriteFloat(&destination[2U * sizeof(float)],
                         (float)position->reference_position_count);
    Debug_VofaWriteFloat(&destination[3U * sizeof(float)],
                         (float)position->measured_position_count);
    Debug_VofaWriteFloat(&destination[4U * sizeof(float)],
                         (float)position->position_error_count);
    Debug_VofaWriteFloat(&destination[5U * sizeof(float)],
                         (float)pulse->raw_reference_speed_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[6U * sizeof(float)],
                         (float)pulse->reference_speed_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[7U * sizeof(float)],
                         (float)position->speed_target_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[8U * sizeof(float)],
                         (float)speed->measured_speed_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[9U * sizeof(float)],
                         (float)speed->speed_error_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[10U * sizeof(float)],
                         (float)speed->logical_iq_target_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[11U * sizeof(float)],
                         (float)current->target_q_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[12U * sizeof(float)],
                         (float)current->measured_q_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[13U * sizeof(float)],
                         (float)speed->proportional_output_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[14U * sizeof(float)],
                         (float)speed->integral_output_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[15U * sizeof(float)],
                         (float)speed->output_saturated);
    Debug_VofaWriteFloat(&destination[16U * sizeof(float)],
                         (float)current->voltage_saturated);
    Debug_VofaWriteFloat(&destination[17U * sizeof(float)],
                         (float)speed->field_weakening_active);
    Debug_VofaWriteFloat(&destination[18U * sizeof(float)],
                         (float)pulse->accepted_pulse_count);
    (void)memcpy(&destination[DEBUG_VOFA_PULSE_CHANNEL_COUNT * sizeof(float)],
                 just_float_tail,
                 sizeof(just_float_tail));
}

/* 通道：time_ms、seq、Pos_ref、Pos_fdb、Speed_ref、Speed_fdb、Iq_ref、reached、fault。 */
static void Debug_VofaBuildPositionCaptureFrame(uint8_t *destination)
{
    const volatile PositionLoop_State_t *position = PositionLoop_GetState();
    const volatile SpeedLoop_State_t *speed = SpeedLoop_GetState();
    static const uint8_t just_float_tail[sizeof(float)] =
        {0x00U, 0x00U, 0x80U, 0x7FU};

    Debug_VofaWriteFloat(&destination[0U * sizeof(float)], (float)HAL_GetTick());
    Debug_VofaWriteFloat(&destination[1U * sizeof(float)],
                         (float)s_vofa.position_capture_sequence++);
    Debug_VofaWriteFloat(&destination[2U * sizeof(float)],
                         (float)position->reference_position_count);
    Debug_VofaWriteFloat(&destination[3U * sizeof(float)],
                         (float)position->measured_position_count);
    Debug_VofaWriteFloat(&destination[4U * sizeof(float)],
                         (float)position->speed_target_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[5U * sizeof(float)],
                         (float)speed->measured_speed_mrpm / 1000.0f);
    Debug_VofaWriteFloat(&destination[6U * sizeof(float)],
                         (float)speed->current_loop_iq_target_ma / 1000.0f);
    Debug_VofaWriteFloat(&destination[7U * sizeof(float)], (float)position->reached);
    Debug_VofaWriteFloat(&destination[8U * sizeof(float)], (float)position->fault);
    (void)memcpy(&destination[DEBUG_VOFA_POSITION_CAPTURE_CHANNEL_COUNT * sizeof(float)],
                 just_float_tail, sizeof(just_float_tail));
}

static void Debug_VofaStop(void)
{
    s_vofa.active = 0U;
    s_vofa.transfer_in_progress = 0U;
    s_vofa.trace_index = 0U;
    s_vofa.trace_count = 0U;
    s_vofa.live_mode = 0U;
    s_vofa.speed_live_mode = 0U;
    s_vofa.speed_position_mode = 0U;
    s_vofa.speed_basic_mode = 0U;
    s_vofa.speed_step_active = 0U;
    s_vofa.speed_step_switch_ms = 0U;
    s_vofa.speed_step_target_mrpm = 0L;
    s_vofa.position_live_mode = 0U;
    s_vofa.position_diagnostic_mode = 0U;
    s_vofa.position_capture_mode = 0U;
    s_vofa.pulse_live_mode = 0U;
    s_vofa.pulse_sequence = 0U;
    s_vofa.pulse_last_sample_tick_ms = 0U;
}

static uint8_t Debug_StartPulseVofa(void)
{
    if ((PulseInput_GetState()->enabled == 0U) ||
        (PositionLoop_GetState()->running == 0U))
    {
        return 0U;
    }

    Debug_VofaStop();
    s_vofa.position_live_mode = 0U;
    s_vofa.pulse_live_mode = 1U;
    s_vofa.pulse_sequence = 0U;
    /* 允许启动命令后同一个SysTick周期立即发送第一帧。 */
    s_vofa.pulse_last_sample_tick_ms = HAL_GetTick() - 1U;
    s_vofa.transfer_in_progress = 0U;
    s_vofa.active = 1U;
    s_debug.stream_enabled = 0U;
    s_debug.trace_dump_active = 0U;
    s_debug.trace_perf_pending = 0U;
    return 1U;
}

static void Debug_VofaStartDqTrace(uint8_t test_fault)
{
    (void)MotorTest_GetDqTrace(&s_vofa.trace_count);
    s_vofa.trace_index = 0U;
    s_vofa.test_fault = test_fault;
    s_vofa.live_mode = 0U;
    s_vofa.speed_live_mode = 0U;
    s_vofa.speed_position_mode = 0U;
    s_vofa.speed_basic_mode = 0U;
    s_vofa.position_live_mode = 0U;
    s_vofa.position_diagnostic_mode = 0U;
    s_vofa.transfer_in_progress = 0U;
    s_vofa.active = (s_vofa.trace_count > 0U) ? 1U : 0U;

    /* JustFloat期间禁止混入ASCII遥测，否则协议引擎会产生伪数据。 */
    s_debug.stream_enabled = 0U;
    s_debug.trace_dump_active = 0U;
    s_debug.trace_perf_pending = 0U;
}

static void Debug_VofaLoop(void)
{
    const MotorTest_DqTraceSample_t *trace;
    MotorTest_DqTraceSample_t live_sample;
    SpeedLoop_TraceSample_t speed_sample;
    uint16_t frame_count;
    uint16_t frame_index;
    uint16_t packet_length;
    uint32_t now_ms;

    if (s_vofa.active == 0U)
    {
        return;
    }

    if (s_vofa.transfer_in_progress != 0U)
    {
        if (CDC_TransmitReady_FS() == 0U)
        {
            return;
        }
        s_vofa.transfer_in_progress = 0U;
    }

    if (CDC_TransmitReady_FS() == 0U)
    {
        return;
    }

    if (s_vofa.pulse_live_mode != 0U)
    {
        if ((PulseInput_GetState()->enabled == 0U) ||
            (PositionLoop_GetState()->running == 0U))
        {
            Debug_VofaStop();
            return;
        }

        now_ms = HAL_GetTick();
        if (now_ms == s_vofa.pulse_last_sample_tick_ms)
        {
            return;
        }

        Debug_VofaBuildPulseFrame(s_vofa.tx_buffer);
        if (CDC_Transmit_FS(s_vofa.tx_buffer,
                            DEBUG_VOFA_PULSE_FRAME_SIZE) == USBD_OK)
        {
            s_vofa.transfer_in_progress = 1U;
            s_vofa.pulse_last_sample_tick_ms = now_ms;
            s_vofa.pulse_sequence++;
        }
        return;
    }

    if (s_vofa.position_live_mode != 0U)
    {
        if (PositionLoop_GetState()->running == 0U)
        {
            Debug_VofaStop();
            return;
        }

        if (s_vofa.position_capture_mode != 0U)
        {
            Debug_VofaBuildPositionCaptureFrame(s_vofa.tx_buffer);
        }
        else if (s_vofa.position_diagnostic_mode != 0U)
        {
            Debug_VofaBuildPositionDiagnosticFrame(s_vofa.tx_buffer);
        }
        else
        {
            Debug_VofaBuildPositionFrame(s_vofa.tx_buffer);
        }
        if (CDC_Transmit_FS(s_vofa.tx_buffer,
                            (s_vofa.position_capture_mode != 0U) ?
                            DEBUG_VOFA_POSITION_CAPTURE_FRAME_SIZE :
                            (s_vofa.position_diagnostic_mode != 0U) ?
                            DEBUG_VOFA_POSITION_DIAGNOSTIC_FRAME_SIZE :
                            DEBUG_VOFA_POSITION_FRAME_SIZE) == USBD_OK)
        {
            s_vofa.transfer_in_progress = 1U;
        }
        return;
    }

    if (s_vofa.speed_live_mode != 0U)
    {
        if ((s_vofa.speed_step_active != 0U) &&
            (SpeedLoop_GetState()->elapsed_ms >=
             s_vofa.speed_step_switch_ms))
        {
            if (SpeedLoop_SetTargetSpeedMrpm(
                    s_vofa.speed_step_target_mrpm) == 0U)
            {
                SpeedLoop_Stop();
                Debug_VofaStop();
                return;
            }
            s_vofa.speed_step_active = 0U;
        }

        if (s_vofa.speed_basic_mode != 0U)
        {
            if (SpeedLoop_GetState()->running == 0U)
            {
                Debug_VofaStop();
                return;
            }

            Debug_VofaBuildSpeedBasicFrame(s_vofa.tx_buffer);
            if (CDC_Transmit_FS(s_vofa.tx_buffer,
                                DEBUG_VOFA_SPEED_BASIC_FRAME_SIZE) == USBD_OK)
            {
                s_vofa.transfer_in_progress = 1U;
            }
            return;
        }

        if (SpeedLoop_PopTraceSample(&speed_sample) == 0U)
        {
            if (SpeedLoop_GetState()->running == 0U)
            {
                Debug_VofaStop();
            }
            return;
        }

        Debug_VofaBuildSpeedFrame(s_vofa.tx_buffer,
                                  &speed_sample,
                                  s_vofa.speed_position_mode);
        if (CDC_Transmit_FS(s_vofa.tx_buffer,
                            DEBUG_VOFA_SPEED_FRAME_SIZE) == USBD_OK)
        {
            s_vofa.transfer_in_progress = 1U;
        }
        return;
    }

    if (s_vofa.live_mode != 0U)
    {
        frame_count = 0U;
        while ((frame_count < DEBUG_VOFA_FRAMES_PER_USB_PACKET) &&
               (MotorTest_PopDqCycleSample(&live_sample) != 0U))
        {
            Debug_VofaBuildFrame(
                &s_vofa.tx_buffer[frame_count * DEBUG_VOFA_FRAME_SIZE],
                &live_sample,
                MotorTest_GetDqCycleFault());
            frame_count++;
        }

        if (frame_count == 0U)
        {
            if (MotorTest_DqCycleIsRunning() == 0U)
            {
                Debug_VofaStop();
            }
            return;
        }

        packet_length = frame_count * DEBUG_VOFA_FRAME_SIZE;
        if (CDC_Transmit_FS(s_vofa.tx_buffer, packet_length) == USBD_OK)
        {
            s_vofa.transfer_in_progress = 1U;
        }
        return;
    }

    if (s_vofa.trace_index >= s_vofa.trace_count)
    {
        Debug_VofaStop();
        return;
    }

    frame_count = s_vofa.trace_count - s_vofa.trace_index;
    if (frame_count > DEBUG_VOFA_FRAMES_PER_USB_PACKET)
    {
        frame_count = DEBUG_VOFA_FRAMES_PER_USB_PACKET;
    }

    trace = MotorTest_GetDqTrace(0);
    for (frame_index = 0U; frame_index < frame_count; frame_index++)
    {
        Debug_VofaBuildFrame(
            &s_vofa.tx_buffer[frame_index * DEBUG_VOFA_FRAME_SIZE],
            &trace[s_vofa.trace_index + frame_index],
            s_vofa.test_fault);
    }
    packet_length = frame_count * DEBUG_VOFA_FRAME_SIZE;

    if (CDC_Transmit_FS(s_vofa.tx_buffer, packet_length) == USBD_OK)
    {
        s_vofa.trace_index += frame_count;
        s_vofa.transfer_in_progress = 1U;
    }
}

static void Debug_SendText(const char *text)
{
    uint16_t length;

    if (text == 0)
    {
        return;
    }

    length = (uint16_t)strlen(text);
    if (CDC_Transmit_FS((uint8_t *)text, length) == USBD_OK)
    {
        return;
    }

    /* 控制文本只有单条在途；USB暂忙时保存一条并由主循环重试。 */
    if (s_debug.pending_text_valid == 0U)
    {
        (void)snprintf(s_debug.pending_text,
                       sizeof(s_debug.pending_text),
                       "%s",
                       text);
        s_debug.pending_text_valid = 1U;
    }
}

static void Debug_ProcessPendingText(void)
{
    uint16_t length;

    if ((s_debug.pending_text_valid == 0U) ||
        (CDC_TransmitReady_FS() == 0U))
    {
        return;
    }

    length = (uint16_t)strlen(s_debug.pending_text);
    if (CDC_Transmit_FS((uint8_t *)s_debug.pending_text, length) == USBD_OK)
    {
        s_debug.pending_text_valid = 0U;
    }
}

static uint8_t Debug_TrySendText(const char *text)
{
    uint16_t length;

    if (text == 0)
    {
        return 0U;
    }

    length = (uint16_t)strlen(text);
    return (CDC_Transmit_FS((uint8_t *)text, length) == USBD_OK) ? 1U : 0U;
}

static void Debug_StartDqTraceDump(void)
{
    (void)MotorTest_GetDqTrace(&s_debug.trace_dump_count);
    s_debug.trace_dump_index = 0U;
    s_debug.trace_header_sent = 0U;
    s_debug.trace_dump_active =
        (s_debug.trace_dump_count > 0U) ? 1U : 0U;
}

static void Debug_DumpDqTraceLoop(void)
{
    const MotorTest_DqTraceSample_t *trace;
    const MotorTest_DqTraceSample_t *sample;

    if (s_debug.trace_perf_pending != 0U)
    {
        const char *pending_text =
            (s_debug.trace_perf_pending == 2U) ?
            s_debug.trace_perf_buffer : s_debug.trace_fall_buffer;

        if (Debug_TrySendText(pending_text) != 0U)
        {
            s_debug.trace_perf_pending--;
        }
        return;
    }

    if (s_debug.trace_header_sent == 0U)
    {
        if (Debug_TrySendText(
                "DQT,index,time_us,target_d_ma,target_q_ma,id_ma,iq_ma,"
                "ia_ma,ib_ma,ud_mv,uq_mv,ecount,sat\r\n") != 0U)
        {
            s_debug.trace_header_sent = 1U;
        }
        return;
    }

    if (s_debug.trace_dump_index >= s_debug.trace_dump_count)
    {
        if (Debug_TrySendText("DQT_END\r\n") != 0U)
        {
            s_debug.trace_dump_active = 0U;
        }
        return;
    }

    trace = MotorTest_GetDqTrace(0);
    sample = &trace[s_debug.trace_dump_index];
    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "DQT,%u,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u\r\n",
                   (unsigned int)s_debug.trace_dump_index,
                   (unsigned long)s_debug.trace_dump_index * 50UL,
                   (int)sample->target_d_ma,
                   (int)sample->target_q_ma,
                   (int)sample->measured_d_ma,
                   (int)sample->measured_q_ma,
                   (int)sample->measured_phase_a_ma,
                   (int)sample->measured_phase_b_ma,
                   (int)sample->output_d_mv,
                   (int)sample->output_q_mv,
                   (int)sample->electrical_count,
                   (unsigned int)sample->voltage_saturated);
    if (Debug_TrySendText(s_debug_tx_buffer) != 0U)
    {
        s_debug.trace_dump_index++;
    }
}

static void Debug_SendEncoderState(void)
{
    const Encoder_State_t *encoder = Encoder_GetState();

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "ENC raw=%ld pos=%ld rev=%ld delta=%ld "
                   "raw_speed=%ldmr/min speed=%ldmr/min "
                   "window=%ums valid=%u pwm=%u\r\n",
                   (long)encoder->raw_count,
                   (long)encoder->position_count,
                   (long)encoder->count_in_revolution,
                   (long)encoder->delta_count,
                   (long)encoder->raw_speed_mrpm,
                   (long)encoder->filtered_speed_mrpm,
                   (unsigned int)encoder->speed_measurement_period_ms,
                   (unsigned int)encoder->speed_valid,
                   (unsigned int)MotorPWM_IsEnabled());
    Debug_SendText(s_debug_tx_buffer);
}

static void Debug_SendSpeedLoopState(void)
{
    const volatile SpeedLoop_State_t *speed = SpeedLoop_GetState();
    const volatile DqCurrentLoop_State_t *current =
        DqCurrentLoop_GetState();
    int32_t angle_quantization_error_mdeg =
        Debug_CalculateAngleQuantizationErrorMdeg(current);
    int32_t peak_angle_quantization_error_mdeg =
        Debug_PhaseIndexToMdeg(
            current->peak_abs_observer_phase_error_index);
    int32_t raw_phase_step_mdeg =
        Debug_PhaseIndexToMdeg(current->raw_phase_step_index);
    int32_t peak_raw_phase_step_mdeg =
        Debug_PhaseIndexToMdeg(current->peak_abs_raw_phase_step_index);

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "SPEED target=%ldmrpm actual=%ldmrpm error=%ldmrpm "
                   "p=%ldmA i=%ldmA kp_eff=%lduA/rpm "
                   "iq=%ldmA id=%ldmA raw_iq=%ldmA "
                   "torque_iq=%ldmA "
                   "measured_iq=%ldmA measured_id=%ldmA "
                   "iq_err=%ldmA id_err=%ldmA "
                   "sat=%u fw=%u vd=%ldmV vq=%ldmV "
                   "vdff=%ldmV vqff=%ldmV "
                   "vbus_f=%lumV vbus_fault=%lumV/raw=%u/limit=%u "
                   "vlimit=%ldmV fw_ratio=%u/1000 fw_corr=%ldmV "
                   "fw_fixed=%u/%dmA "
                   "hold=%u/%u/%u "
                   "angle_adv=%luus/%ldcnt "
                   "angle_qerr=%ldmdeg qerr_pk=%ldmdeg qerr_n=%lu "
                   "angle_step=%ldmdeg astep_pk=%ldmdeg astep_n=%lu "
                   "vpeak_d=%ldmV vpeak_q=%ldmV "
                   "vsat=%u vsat_n=%lu dprio=%u/%lu "
                   "elapsed=%lums running=%u fault=%u "
                   "trace_drop=%lu\r\n",
                   (long)speed->target_speed_mrpm,
                   (long)speed->measured_speed_mrpm,
                   (long)speed->speed_error_mrpm,
                   (long)speed->proportional_output_ma,
                   (long)speed->integral_output_ma,
                   (long)speed->effective_kp_ua_per_rpm,
                   (long)speed->logical_iq_target_ma,
                   (long)speed->current_loop_id_target_ma,
                   (long)speed->current_loop_iq_target_ma,
                   (long)speed->torque_equivalent_iq_ma,
                   (long)current->measured_q_ma,
                   (long)current->measured_d_ma,
                   (long)(speed->current_loop_iq_target_ma -
                          current->measured_q_ma),
                   (long)(speed->current_loop_id_target_ma -
                          current->measured_d_ma),
                   (unsigned int)speed->output_saturated,
                   (unsigned int)speed->field_weakening_active,
                   (long)current->output_d_mv,
                   (long)current->output_q_mv,
                   (long)current->feedforward_d_mv,
                   (long)current->feedforward_q_mv,
                   (unsigned long)speed->filtered_bus_voltage_mv,
                   (unsigned long)speed->fault_bus_voltage_mv,
                   (unsigned int)speed->fault_bus_voltage_raw,
                   (unsigned int)speed->fault_bus_voltage_limit,
                   (long)speed->current_voltage_limit_mv,
                   (unsigned int)
                       speed->field_weakening_voltage_ratio_permille,
                   (long)speed->field_weakening_voltage_correction_mv,
                   (unsigned int)speed->fixed_weakening_enabled,
                   (int)speed->fixed_weakening_id_ma,
                   (unsigned int)speed->current_hold_enabled,
                   (unsigned int)speed->current_hold_active,
                   (unsigned int)speed->current_hold_completed,
                   (unsigned long)current->angle_advance_delay_us,
                   (long)current->angle_advance_count,
                   (long)angle_quantization_error_mdeg,
                   (long)peak_angle_quantization_error_mdeg,
                   (unsigned long)
                       current->observer_phase_error_exceed_sample_count,
                   (long)raw_phase_step_mdeg,
                   (long)peak_raw_phase_step_mdeg,
                   (unsigned long)current->raw_phase_step_exceed_sample_count,
                   (long)current->peak_abs_output_d_mv,
                   (long)current->peak_abs_output_q_mv,
                   (unsigned int)current->voltage_saturated,
                   (unsigned long)current->saturated_sample_count,
                   (unsigned int)current->d_axis_priority_active,
                   (unsigned long)current->d_axis_priority_sample_count,
                   (unsigned long)speed->elapsed_ms,
                   (unsigned int)speed->running,
                   (unsigned int)speed->fault,
                   (unsigned long)SpeedLoop_GetTraceDroppedCount());
    Debug_SendText(s_debug_tx_buffer);
}

static uint8_t Debug_StartSpeedTestCore(int32_t target_speed_mrpm,
                                        uint8_t current_hold_enabled)
{
    SpeedLoop_Config_t config;

    config.kp_ua_per_rpm = s_speed_tune.kp_ua_per_rpm;
    config.ki_ua_per_rpm_s = s_speed_tune.ki_ua_per_rpm_s;
    config.current_vector_limit_ma =
        DEBUG_SPEED_TEST_CURRENT_VECTOR_LIMIT_MA;
    config.maximum_speed_mrpm = DEBUG_SPEED_TEST_MAXIMUM_MRPM;
    config.maximum_run_time_ms = DEBUG_SPEED_TEST_DURATION_MS;
    config.torque_direction_sign = DEBUG_SPEED_TEST_TORQUE_SIGN;
    config.direction_check_enabled = 1U;
    config.hold_start_ms = (current_hold_enabled != 0U) ?
        DEBUG_SPEED_HOLD_START_MS : 0U;
    config.hold_duration_ms = (current_hold_enabled != 0U) ?
        DEBUG_SPEED_HOLD_DURATION_MS : 0U;
    config.hold_minimum_abs_speed_mrpm = (current_hold_enabled != 0U) ?
        DEBUG_SPEED_HOLD_MINIMUM_MRPM : 0L;
    config.hold_maximum_abs_speed_mrpm = (current_hold_enabled != 0U) ?
        DEBUG_SPEED_HOLD_MAXIMUM_MRPM : 0L;

    if (SpeedLoop_Start(&config, target_speed_mrpm) == 0U)
    {
        return 0U;
    }

    return 1U;
}

static void Debug_StartSpeedTest(int32_t target_speed_mrpm)
{
    if (Debug_StartSpeedTestCore(target_speed_mrpm, 0U) == 0U)
    {
        Debug_SendText("err: speed loop start failed\r\n");
        Debug_SendSpeedLoopState();
        return;
    }

    s_debug.stream_enabled = 1U;
    s_debug.last_stream_tick_ms = HAL_GetTick();
    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "ok: speed run target=%ldmrpm kp=%lduA/rpm "
                   "ki=%lduA/(rpm*s)\r\n",
                   (long)target_speed_mrpm,
                   (long)s_speed_tune.kp_ua_per_rpm,
                   (long)s_speed_tune.ki_ua_per_rpm_s);
    Debug_SendText(s_debug_tx_buffer);
}

/*
 * 位置环首轮验证只允许低速小角度动作。编码器没有Z相，所有位置均是本次
 * 上电后的多圈相对坐标；绝对机械坐标需要后续回零开关建立。
 */
static uint8_t Debug_StartPositionTestWithSpeedLimit(
    int32_t target_position_count,
    int32_t maximum_speed_mrpm)
{
    PositionLoop_Config_t config;
    int32_t tracking_reserve_mrpm;

    config.kp_mrpm_per_count = s_position_tune.kp_mrpm_per_count;
    config.kd_mrpm_per_rpm = s_position_tune.kd_mrpm_per_rpm;
    config.maximum_speed_mrpm = maximum_speed_mrpm;
    tracking_reserve_mrpm = config.maximum_speed_mrpm /
                            (1000L / DEBUG_POSITION_TRACKING_RESERVE_PERMILLE);
    if (tracking_reserve_mrpm < DEBUG_POSITION_TRACKING_RESERVE_MIN_MRPM)
    {
        tracking_reserve_mrpm = DEBUG_POSITION_TRACKING_RESERVE_MIN_MRPM;
    }
    config.maximum_tracking_speed_mrpm = config.maximum_speed_mrpm +
                                         tracking_reserve_mrpm;
    if (config.maximum_tracking_speed_mrpm > DEBUG_SPEED_TEST_MAXIMUM_MRPM)
    {
        config.maximum_tracking_speed_mrpm = DEBUG_SPEED_TEST_MAXIMUM_MRPM;
    }
    config.maximum_acceleration_mrpm_per_s =
        DEBUG_POSITION_TEST_MAXIMUM_ACCELERATION_MRPM_PER_S;
    config.position_tolerance_count = DEBUG_POSITION_TEST_TOLERANCE_COUNT;
    config.settle_speed_mrpm = DEBUG_POSITION_TEST_SETTLE_SPEED_MRPM;
    config.settle_time_ms = DEBUG_POSITION_TEST_SETTLE_TIME_MS;
    config.maximum_position_error_count =
        DEBUG_POSITION_TEST_MAXIMUM_ERROR_COUNT;
    config.position_error_timeout_ms = DEBUG_POSITION_TEST_ERROR_TIMEOUT_MS;
    config.speed_config.kp_ua_per_rpm = s_speed_tune.kp_ua_per_rpm;
    config.speed_config.ki_ua_per_rpm_s = s_speed_tune.ki_ua_per_rpm_s;
    config.speed_config.current_vector_limit_ma =
        DEBUG_SPEED_TEST_CURRENT_VECTOR_LIMIT_MA;
    config.speed_config.maximum_speed_mrpm = DEBUG_SPEED_TEST_MAXIMUM_MRPM;
    config.speed_config.maximum_run_time_ms =
        DEBUG_POSITION_TEST_MAXIMUM_RUN_TIME_MS;
    config.speed_config.torque_direction_sign = DEBUG_SPEED_TEST_TORQUE_SIGN;
    config.speed_config.direction_check_enabled = 0U;
    config.speed_config.hold_start_ms = 0U;
    config.speed_config.hold_duration_ms = 0U;
    config.speed_config.hold_minimum_abs_speed_mrpm = 0L;
    config.speed_config.hold_maximum_abs_speed_mrpm = 0L;

    return PositionLoop_Start(&config, target_position_count);
}

static uint8_t Debug_StartPositionTest(int32_t target_position_count)
{
    return Debug_StartPositionTestWithSpeedLimit(
        target_position_count,
        s_position_tune.maximum_speed_mrpm);
}

static uint8_t Debug_StartPulseInput(void)
{
    /* 产品态接口由App在校准完成后自动启动；此命令仅兼容旧调试流程。 */
    return PulseInput_Start();
}

static void Debug_SendPulseInputState(void)
{
    const volatile PulseInput_State_t *pulse = PulseInput_GetState();

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "PULSE mode=position_pd_r1 step_per_rev=800 accepted=%lu ignored=%lu ref=%ld "
                   "raw_speed=%ld ref_speed=%ld stream=%u edge=%lu/%ld/%ld/err=%ld last_step=%lums/%ld/%ld/err=%ld "
                   "speed_ref=%ld/speed_meas=%ld peak_pos_err=%ld "
                   "peak_speed_err=%ld samples=%lu first=%lu/%ld/%ld/%ld "
                   "peak=%lu/%ld/%ld/%ld speed_peak=%lu/%ld/%ld/%ld "
                   "enabled=%u en_input=%u\\r\\n",
                   (unsigned long)pulse->accepted_pulse_count,
                   (unsigned long)pulse->ignored_pulse_count,
                   (long)pulse->reference_position_count,
                   (long)pulse->raw_reference_speed_mrpm,
                   (long)pulse->reference_speed_mrpm,
                   (unsigned int)pulse->pulse_stream_active,
                   (unsigned long)pulse->last_edge_capture_cycle_count,
                   (long)pulse->last_edge_reference_position_count,
                   (long)pulse->last_edge_measured_position_count,
                   (long)(pulse->last_edge_reference_position_count -
                          pulse->last_edge_measured_position_count),
                   (unsigned long)pulse->last_step_tick_ms,
                   (long)pulse->last_step_reference_position_count,
                   (long)pulse->last_step_measured_position_count,
                   (long)pulse->last_step_error_count,
                   (long)pulse->last_step_reference_speed_mrpm,
                   (long)pulse->last_step_measured_speed_mrpm,
                   (long)pulse->maximum_abs_position_error_count,
                   (long)pulse->maximum_abs_speed_error_mrpm,
                   (unsigned long)pulse->motion_sample_count,
                   (unsigned long)pulse->first_error_pulse_count,
                   (long)pulse->first_error_position_count,
                   (long)pulse->first_error_reference_speed_mrpm,
                   (long)pulse->first_error_measured_speed_mrpm,
                   (unsigned long)pulse->peak_error_pulse_count,
                   (long)pulse->peak_error_position_count,
                   (long)pulse->peak_error_reference_speed_mrpm,
                   (long)pulse->peak_error_measured_speed_mrpm,
                   (unsigned long)pulse->peak_speed_error_pulse_count,
                   (long)pulse->peak_speed_error_position_count,
                   (long)pulse->peak_speed_error_reference_mrpm,
                   (long)pulse->peak_speed_error_measured_mrpm,
                   (unsigned int)pulse->enabled,
                   (unsigned int)pulse->input_enabled);
    Debug_SendText(s_debug_tx_buffer);
}

static uint8_t Debug_StartVofaPositionTest(int32_t relative_position_count,
                                           uint8_t diagnostic_mode,
                                           uint8_t capture_mode)
{
    int64_t target_position_count =
        (int64_t)Encoder_GetPositionCountFast() + relative_position_count;

    if ((target_position_count > INT32_MAX) ||
        (target_position_count < INT32_MIN))
    {
        return 0U;
    }

    Debug_VofaStop();
    if (Debug_StartPositionTest((int32_t)target_position_count) == 0U)
    {
        return 0U;
    }

    s_vofa.trace_index = 0U;
    s_vofa.trace_count = 0U;
    s_vofa.test_fault = 0U;
    s_vofa.live_mode = 0U;
    s_vofa.speed_live_mode = 0U;
    s_vofa.speed_position_mode = 0U;
    s_vofa.speed_basic_mode = 0U;
    s_vofa.position_live_mode = 1U;
    s_vofa.position_diagnostic_mode = diagnostic_mode;
    s_vofa.position_capture_mode = capture_mode;
    s_vofa.position_capture_sequence = 0U;
    s_vofa.transfer_in_progress = 0U;
    s_vofa.active = 1U;
    s_debug.stream_enabled = 0U;
    s_debug.trace_dump_active = 0U;
    s_debug.trace_perf_pending = 0U;
    return 1U;
}

static void Debug_SendPositionLoopState(void)
{
    const volatile PositionLoop_State_t *position = PositionLoop_GetState();

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "POS target=%ld measured=%ld error=%ld speed_cmd=%ld "
                   "reached=%u running=%u fault=%u\\r\\n",
                   (long)position->target_position_count,
                   (long)position->measured_position_count,
                   (long)position->position_error_count,
                   (long)position->speed_target_mrpm,
                   (unsigned int)position->reached,
                   (unsigned int)position->running,
                   (unsigned int)position->fault);
    Debug_SendText(s_debug_tx_buffer);
}

static void Debug_StartVofaSpeedTest(int32_t target_speed_mrpm,
                                     uint8_t current_hold_enabled,
                                     uint8_t position_mode,
                                     uint8_t basic_mode)
{
    Debug_VofaStop();
    if (Debug_StartSpeedTestCore(target_speed_mrpm,
                                 current_hold_enabled) == 0U)
    {
        Debug_SendText("err: speed loop start failed\r\n");
        Debug_SendSpeedLoopState();
        return;
    }

    s_vofa.trace_index = 0U;
    s_vofa.trace_count = 0U;
    s_vofa.test_fault = 0U;
    s_vofa.live_mode = 1U;
    s_vofa.speed_live_mode = 1U;
    s_vofa.speed_position_mode = position_mode;
    s_vofa.speed_basic_mode = basic_mode;
    s_vofa.speed_step_active = 0U;
    s_vofa.speed_step_switch_ms = 0U;
    s_vofa.speed_step_target_mrpm = 0L;
    s_vofa.transfer_in_progress = 0U;
    s_vofa.active = 1U;
    s_debug.stream_enabled = 0U;
    s_debug.trace_dump_active = 0U;
    s_debug.trace_perf_pending = 0U;
}

/* 仅用于速度PI阶跃整定：先稳定初始速度，再在同一速度环实例内切换目标。 */
static void Debug_StartVofaSpeedStepTest(int32_t initial_speed_mrpm,
                                         int32_t target_speed_mrpm)
{
    Debug_StartVofaSpeedTest(initial_speed_mrpm, 0U, 0U, 0U);
    if (s_vofa.speed_live_mode == 0U)
    {
        return;
    }

    s_vofa.speed_step_active = 1U;
    s_vofa.speed_step_switch_ms = DEBUG_SPEED_STEP_PRECONDITION_MS;
    s_vofa.speed_step_target_mrpm = target_speed_mrpm;
}

static void Debug_SetSpeedTunings(int32_t kp_ua_per_rpm,
                                  int32_t ki_ua_per_rpm_s)
{
    if ((kp_ua_per_rpm < 0L) ||
        (ki_ua_per_rpm_s < 0L) ||
        (kp_ua_per_rpm > DEBUG_SPEED_KP_MAX_UA_PER_RPM) ||
        (ki_ua_per_rpm_s > DEBUG_SPEED_KI_MAX_UA_PER_RPM_S))
    {
        Debug_SendText("err: speed tuning out of range\r\n");
        return;
    }

    s_speed_tune.kp_ua_per_rpm = kp_ua_per_rpm;
    s_speed_tune.ki_ua_per_rpm_s = ki_ua_per_rpm_s;
    if (SpeedLoop_SetTunings(kp_ua_per_rpm,
                             ki_ua_per_rpm_s) == 0U)
    {
        Debug_SendText("err: speed tuning update failed\r\n");
        return;
    }

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "ok: speed tune kp=%lduA/rpm ki=%lduA/(rpm*s)\r\n",
                   (long)kp_ua_per_rpm,
                   (long)ki_ua_per_rpm_s);
    Debug_SendText(s_debug_tx_buffer);
}

/*
 * 位置环增益允许在运行中更新，用于固定运动指令下的Kp/Kd扫描。
 * 停止时仅保存参数，下一次位置命令启动时生效。
 */
static void Debug_SetPositionTunings(int32_t kp_mrpm_per_count,
                                     int32_t kd_mrpm_per_rpm)
{
    if ((kp_mrpm_per_count <= 0L) || (kd_mrpm_per_rpm < 0L) ||
        (kp_mrpm_per_count > DEBUG_POSITION_KP_MAX_MRPM_PER_COUNT) ||
        (kd_mrpm_per_rpm > DEBUG_POSITION_KD_MAX_MRPM_PER_RPM))
    {
        Debug_SendText("err: position tuning out of range\r\n");
        return;
    }

    if ((PositionLoop_GetState()->running != 0U) &&
        (PositionLoop_SetGains(kp_mrpm_per_count,
                               kd_mrpm_per_rpm) == 0U))
    {
        Debug_SendText("err: position tuning update failed\r\n");
        return;
    }

    s_position_tune.kp_mrpm_per_count = kp_mrpm_per_count;
    s_position_tune.kd_mrpm_per_rpm = kd_mrpm_per_rpm;
    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "ok: pos tune kp=%ldmrpm/count kd=%ldmrpm/rpm\r\n",
                   (long)kp_mrpm_per_count,
                   (long)kd_mrpm_per_rpm);
    Debug_SendText(s_debug_tx_buffer);
}

/* 位置速度上限是轨迹规划前的安全边界，可在线调试且受速度环能力约束。 */
static void Debug_SetPositionSpeedLimit(int32_t maximum_speed_mrpm)
{
    if ((maximum_speed_mrpm < DEBUG_POSITION_SPEED_LIMIT_MIN_MRPM) ||
        (maximum_speed_mrpm > DEBUG_POSITION_SPEED_LIMIT_MAX_MRPM))
    {
        Debug_SendText("err: position speed limit out of range\r\n");
        return;
    }

    if ((PositionLoop_GetState()->running != 0U) &&
        (PositionLoop_SetMaximumSpeedMrpm(maximum_speed_mrpm) == 0U))
    {
        Debug_SendText("err: position speed limit update failed\r\n");
        return;
    }

    s_position_tune.maximum_speed_mrpm = maximum_speed_mrpm;
    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "ok: pos limit=%ldrpm\r\n",
                   (long)(maximum_speed_mrpm / 1000L));
    Debug_SendText(s_debug_tx_buffer);
}

static void Debug_SendCurrentState(void)
{
    const CurrentSense_State_t *current = CurrentSense_GetState();

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "ADC a=%u b=%u off_a=%u off_b=%u ia=%ldmA ib=%ldmA "
                   "span_a=%u span_b=%u valid=%u sync=%u samples=%lu\r\n",
                   (unsigned int)current->raw_a,
                   (unsigned int)current->raw_b,
                   (unsigned int)current->offset_a,
                   (unsigned int)current->offset_b,
                   (long)current->current_a_ma,
                   (long)current->current_b_ma,
                   (unsigned int)current->calibration_span_a,
                   (unsigned int)current->calibration_span_b,
                   (unsigned int)current->offset_valid,
                   (unsigned int)current->synchronized,
                   (unsigned long)current->synchronized_sample_count);
    Debug_SendText(s_debug_tx_buffer);
}

static void Debug_SendBusVoltage(void)
{
    const PowerMonitor_State_t *power = PowerMonitor_GetState();

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "BUS raw=%u voltage=%lumV\r\n",
                   (unsigned int)power->bus_voltage_raw,
                   (unsigned long)power->bus_voltage_mv);
    Debug_SendText(s_debug_tx_buffer);
}

static const char *Debug_GetAlignmentStateName(RotorAlignment_StateCode_t state)
{
    switch (state)
    {
        case ROTOR_ALIGNMENT_STATE_IDLE:
            return "idle";
        case ROTOR_ALIGNMENT_STATE_RAMP_TO_90_DEG:
            return "ramp90";
        case ROTOR_ALIGNMENT_STATE_HOLD_90_DEG:
            return "hold90";
        case ROTOR_ALIGNMENT_STATE_ROTATE_TO_ZERO:
            return "rotate0";
        case ROTOR_ALIGNMENT_STATE_HOLD_ZERO:
            return "hold0";
        case ROTOR_ALIGNMENT_STATE_COMPLETE:
            return "complete";
        case ROTOR_ALIGNMENT_STATE_FAULT:
            return "fault";
        default:
            return "unknown";
    }
}

static void Debug_SendAlignmentState(void)
{
    const RotorAlignment_State_t *alignment = RotorAlignment_GetState();

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "ALIGN state=%s fault=%u valid=%u auto=%u first=%ld zero=%ld delta=%ld "
                   "expected=%ld offset=%ld beta_sign=%d target_a=%ldmA target_b=%ldmA samples=%lu "
                   "sat=%lu i90_a=%ldmA i90_b=%ldmA i0_a=%ldmA i0_b=%ldmA "
                   "ipeak_a=%ldmA ipeak_b=%ldmA vpeak_a=%ldmV vpeak_b=%ldmV\r\n",
                   Debug_GetAlignmentStateName(alignment->state),
                   (unsigned int)alignment->fault,
                   (unsigned int)alignment->valid,
                   (unsigned int)alignment->auto_start_pending,
                   (long)alignment->first_position_count,
                   (long)alignment->zero_position_count,
                   (long)alignment->movement_count,
                   (long)alignment->expected_abs_movement_count,
                   (long)alignment->alignment_offset_count,
                   (int)alignment->phase_b_axis_sign,
                   (long)alignment->target_a_ma,
                   (long)alignment->target_b_ma,
                   (unsigned long)alignment->current_sample_count,
                   (unsigned long)alignment->saturated_sample_count,
                   (long)alignment->hold_90_mean_current_a_ma,
                   (long)alignment->hold_90_mean_current_b_ma,
                   (long)alignment->hold_zero_mean_current_a_ma,
                   (long)alignment->hold_zero_mean_current_b_ma,
                   (long)alignment->peak_abs_current_a_ma,
                   (long)alignment->peak_abs_current_b_ma,
                   (long)alignment->peak_abs_voltage_a_mv,
                   (long)alignment->peak_abs_voltage_b_mv);
    Debug_SendText(s_debug_tx_buffer);
}

static const char *Debug_GetEncoderCalibrationStateName(
    EncoderCalibration_StateCode_t state)
{
    switch (state)
    {
        case ENCODER_CALIBRATION_STATE_IDLE:
            return "idle";
        case ENCODER_CALIBRATION_STATE_RAMP_CURRENT:
            return "ramp";
        case ENCODER_CALIBRATION_STATE_HOLD_START:
            return "hold";
        case ENCODER_CALIBRATION_STATE_MOVE_PRELOAD_REVERSE:
            return "preload_reverse";
        case ENCODER_CALIBRATION_STATE_SETTLE_PRELOAD_REVERSE:
            return "preload_settle";
        case ENCODER_CALIBRATION_STATE_RETURN_FORWARD_ZERO:
            return "return_zero";
        case ENCODER_CALIBRATION_STATE_SETTLE_FORWARD_ZERO:
            return "zero_settle";
        case ENCODER_CALIBRATION_STATE_MOVE_FORWARD:
            return "forward_move";
        case ENCODER_CALIBRATION_STATE_SETTLE_FORWARD:
            return "forward_settle";
        case ENCODER_CALIBRATION_STATE_MOVE_OVERSHOOT_FORWARD:
            return "overshoot_forward";
        case ENCODER_CALIBRATION_STATE_SETTLE_OVERSHOOT_FORWARD:
            return "overshoot_settle";
        case ENCODER_CALIBRATION_STATE_MOVE_REVERSE:
            return "reverse_move";
        case ENCODER_CALIBRATION_STATE_SETTLE_REVERSE:
            return "reverse_settle";
        case ENCODER_CALIBRATION_STATE_COMPLETE:
            return "complete";
        case ENCODER_CALIBRATION_STATE_FAULT:
            return "fault";
        default:
            return "unknown";
    }
}

static void Debug_SendEncoderCalibrationState(void)
{
    const EncoderCalibration_State_t *calibration =
        RotorAlignment_GetEncoderCalibrationState();

    (void)snprintf(
        s_debug_tx_buffer,
        sizeof(s_debug_tx_buffer),
        "CAL state=%s fault=%u index=%u valid=%u start=%ld "
        "forward_total=%ld reverse_total=%ld closure_q1=%ld "
        "interval_q1=%ld..%ld "
        "hysteresis_max=%ld sat=%lu\r\n",
        Debug_GetEncoderCalibrationStateName(calibration->state),
        (unsigned int)calibration->fault,
        (unsigned int)calibration->boundary_index,
        (unsigned int)calibration->data_valid,
        (long)calibration->start_position_count,
        (long)calibration->forward_total_count,
        (long)calibration->reverse_total_count,
        (long)calibration->closure_error_q1,
        (long)calibration->minimum_interval_q1,
        (long)calibration->maximum_interval_q1,
        (long)calibration->maximum_hysteresis_count,
        (unsigned long)calibration->saturated_sample_count);
    Debug_SendText(s_debug_tx_buffer);
}

static void Debug_StartEncoderCalibrationDump(void)
{
    if (RotorAlignment_GetEncoderCalibrationState()->data_valid == 0U)
    {
        Debug_SendText("err: no valid calibration data\r\n");
        return;
    }

    s_debug.calibration_dump_index = 0U;
    s_debug.calibration_header_sent = 0U;
    s_debug.calibration_dump_active = 1U;
    s_debug.stream_enabled = 0U;
}

static void Debug_DumpEncoderCalibrationLoop(void)
{
    int32_t forward_count;
    int32_t reverse_count;
    int32_t midpoint_q1;

    if (CDC_TransmitReady_FS() == 0U)
    {
        return;
    }

    if (s_debug.calibration_header_sent == 0U)
    {
        if (Debug_TrySendText(
                "CALDATA,index,forward_count,reverse_count,midpoint_q1\r\n") != 0U)
        {
            s_debug.calibration_header_sent = 1U;
        }
        return;
    }

    if (s_debug.calibration_dump_index >=
        ENCODER_CALIBRATION_BOUNDARY_COUNT)
    {
        if (Debug_TrySendText("CALDATA_END\r\n") != 0U)
        {
            s_debug.calibration_dump_active = 0U;
        }
        return;
    }

    if (RotorAlignment_GetEncoderCalibrationPoint(
            s_debug.calibration_dump_index,
            &forward_count,
            &reverse_count,
            &midpoint_q1) == 0U)
    {
        s_debug.calibration_dump_active = 0U;
        return;
    }

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "CALDATA,%u,%ld,%ld,%ld\r\n",
                   (unsigned int)s_debug.calibration_dump_index,
                   (long)forward_count,
                   (long)reverse_count,
                   (long)midpoint_q1);
    if (Debug_TrySendText(s_debug_tx_buffer) != 0U)
    {
        s_debug.calibration_dump_index++;
    }
}

static void Debug_SendElectricalAngle(void)
{
    const ElectricalAngle_State_t *angle = ElectricalAngle_GetState();

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "EANGLE aligned=%u offset=%ld offset_mod80=%ld beta_sign=%d "
                   "count=%ld angle=%ldmdeg observer=%s init=%u "
                   "raw_idx=%ld corrected_idx=%ld correction_idx=%ld "
                   "cal=%u/%u anchor_q1=%ld "
                   "obs_idx=%ld err_q16=%ld vel_q16_tick=%ld "
                   "resync=%lu bw=%luHz alpha_q20=%lu beta_q20=%lu\r\n",
                   (unsigned int)angle->aligned,
                   (long)angle->alignment_offset_count,
                   (long)angle->offset_in_electrical_cycle,
                   (int)angle->phase_b_axis_sign,
                   (long)angle->electrical_count,
                   (long)angle->electrical_angle_mdeg,
                   Debug_GetAngleObserverModeName(angle->observer_mode),
                   (unsigned int)angle->observer_initialized,
                   (long)angle->raw_phase_index,
                   (long)angle->corrected_phase_index,
                   (long)angle->calibration_correction_phase_index,
                   (unsigned int)angle->calibration_valid,
                   (unsigned int)angle->calibration_enabled,
                   (long)angle->calibration_anchor_q1,
                   (long)angle->observer_phase_index,
                   (long)angle->observer_error_q16,
                   (long)angle->observer_velocity_q16_per_tick,
                   (unsigned long)angle->observer_resync_count,
                   (unsigned long)angle->observer_bandwidth_hz,
                   (unsigned long)angle->observer_alpha_q20,
                   (unsigned long)angle->observer_beta_q20);
    Debug_SendText(s_debug_tx_buffer);
}

static void Debug_RunMotorTest(MotorPWM_Phase_t phase, int8_t polarity)
{
    MotorTest_Result_t result;

    if (MotorTest_RunPhaseCurrent(phase, polarity, &result) == 0U)
    {
        Debug_SendText("err: motor test precondition failed\r\n");
        return;
    }

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "TEST phase=%c polarity=%c mean_a=%ldmA mean_b=%ldmA peak_a=%ldmA "
                   "peak_b=%ldmA samples=%lu elapsed=%lums trip=%u\r\n",
                   (phase == MOTOR_PWM_PHASE_A) ? 'A' : 'B',
                   (polarity > 0) ? '+' : '-',
                   (long)result.mean_current_a_ma,
                   (long)result.mean_current_b_ma,
                   (long)result.peak_abs_current_a_ma,
                   (long)result.peak_abs_current_b_ma,
                   (unsigned long)result.synchronized_sample_count,
                   (unsigned long)result.elapsed_ms,
                   (unsigned int)result.overcurrent);
    Debug_SendText(s_debug_tx_buffer);
}

static void Debug_RunCurrentNoiseTest(void)
{
    MotorTest_Result_t result;

    if (MotorTest_RunCurrentNoise(&result) == 0U)
    {
        Debug_SendText("err: current noise test precondition failed\r\n");
        return;
    }

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "NOISE mean_a=%ldmA mean_b=%ldmA pp_a=%ldmA pp_b=%ldmA "
                   "rms_a=%lumA rms_b=%lumA samples=%lu elapsed=%lums trip=%u\r\n",
                   (long)result.mean_current_a_ma,
                   (long)result.mean_current_b_ma,
                   (long)result.peak_to_peak_a_ma,
                   (long)result.peak_to_peak_b_ma,
                   (unsigned long)result.noise_rms_a_ma,
                   (unsigned long)result.noise_rms_b_ma,
                   (unsigned long)result.synchronized_sample_count,
                   (unsigned long)result.elapsed_ms,
                   (unsigned int)result.overcurrent);
    Debug_SendText(s_debug_tx_buffer);
}

static void Debug_RunClosedLoopCurrentTest(int32_t target_a_ma,
                                           int32_t target_b_ma)
{
    MotorTest_Result_t result;

    if (MotorTest_RunClosedLoopCurrent(target_a_ma, target_b_ma, &result) == 0U)
    {
        Debug_SendText("err: current loop test precondition failed\r\n");
        return;
    }

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "ILOOP bw=800Hz target_a=%ldmA target_b=%ldmA mean_a=%ldmA "
                   "mean_b=%ldmA peak_a=%ldmA peak_b=%ldmA vpeak_a=%ldmV "
                   "vpeak_b=%ldmV sat=%lu samples=%lu elapsed=%lums trip=%u\r\n",
                   (long)target_a_ma,
                   (long)target_b_ma,
                   (long)result.mean_current_a_ma,
                   (long)result.mean_current_b_ma,
                   (long)result.peak_abs_current_a_ma,
                   (long)result.peak_abs_current_b_ma,
                   (long)result.peak_abs_voltage_a_mv,
                   (long)result.peak_abs_voltage_b_mv,
                   (unsigned long)result.saturated_sample_count,
                   (unsigned long)result.synchronized_sample_count,
                   (unsigned long)result.elapsed_ms,
                   (unsigned int)result.overcurrent);
    Debug_SendText(s_debug_tx_buffer);
}

static void Debug_RunDqCurrentTest(int32_t target_d_ma,
                                   int32_t target_q_ma)
{
    MotorTest_DqResult_t result;

    if (MotorTest_RunDqCurrent(target_d_ma, target_q_ma, &result) == 0U)
    {
        Debug_SendText("err: dq current test precondition failed\r\n");
        return;
    }

    (void)snprintf(s_debug_tx_buffer,
                   sizeof(s_debug_tx_buffer),
                   "DQSTEP bw=%luHz vlimit=%ldmV target_d=%ldmA target_q=%ldmA "
                   "move=%ld ecount=%ld "
                   "ipeak_a=%ldmA ipeak_b=%ldmA "
                   "vpeak_d=%ldmV vpeak_q=%ldmV sat=%lu samples=%lu elapsed=%lums fault=%u\r\n",
                   (unsigned long)result.bandwidth_hz,
                   (long)result.voltage_limit_mv,
                   (long)result.target_d_ma,
                   (long)result.target_q_ma,
                   (long)result.movement_count,
                   (long)result.final_electrical_count,
                   (long)result.peak_abs_phase_current_a_ma,
                   (long)result.peak_abs_phase_current_b_ma,
                   (long)result.peak_abs_output_d_mv,
                   (long)result.peak_abs_output_q_mv,
                   (unsigned long)result.saturated_sample_count,
                   (unsigned long)result.synchronized_sample_count,
                   (unsigned long)result.elapsed_ms,
                   (unsigned int)result.fault);
    Debug_SendText(s_debug_tx_buffer);

    (void)snprintf(s_debug.trace_perf_buffer,
                   sizeof(s_debug.trace_perf_buffer),
                   "DQPERF delay10=%ldus rise10_90=%ldus cross=%ldus enter5=%ldus "
                   "stable5_1ms=%ldus peak=%ldmA peak_t=%ldus overshoot=%lupermille "
                   "steady_axis=%ldmA steady_orth=%ldmA err_rms=%lumA orth_rms=%lumA "
                   "ripple_axis_pp=%ldmA orth_abs_peak=%ldmA\r\n",
                   (long)result.response_delay_10_us,
                   (long)result.rise_time_10_to_90_us,
                   (long)result.first_target_cross_us,
                   (long)result.first_enter_5_percent_us,
                   (long)result.first_stable_5_percent_us,
                   (long)result.peak_axis_ma,
                   (long)result.peak_time_us,
                   (unsigned long)result.overshoot_permille,
                   (long)result.steady_axis_ma,
                   (long)result.steady_orthogonal_ma,
                   (unsigned long)result.steady_axis_error_rms_ma,
                   (unsigned long)result.steady_orthogonal_rms_ma,
                   (long)result.steady_axis_ripple_pp_ma,
                   (long)result.steady_orthogonal_abs_peak_ma);
    (void)snprintf(s_debug.trace_fall_buffer,
                   sizeof(s_debug.trace_fall_buffer),
                   "DQFALL fall90_10=%ldus enter5=%ldus stable5_1ms=%ldus "
                   "min_axis=%ldmA zero_axis_rms=%lumA zero_orth_rms=%lumA\r\n",
                   (long)result.fall_time_90_to_10_us,
                   (long)result.fall_first_enter_5_percent_us,
                   (long)result.fall_first_stable_5_percent_us,
                   (long)result.fall_min_axis_ma,
                   (unsigned long)result.fall_axis_rms_ma,
                   (unsigned long)result.fall_orthogonal_rms_ma);
    s_debug.trace_perf_pending = 2U;
    Debug_StartDqTraceDump();
}

static void Debug_RunVofaDqTest(int32_t target_d_ma,
                                int32_t target_q_ma)
{
    MotorTest_DqResult_t result;

    /*
     * 测试期间20kHz数据仅写入RAM；完成后才启动USB回放，
     * USB忙不会延长或打断电流环中断。
    */
    Debug_VofaStop();
    if (MotorTest_RunDqCurrent(target_d_ma, target_q_ma, &result) == 0U)
    {
        return;
    }

    Debug_VofaStartDqTrace(result.fault);
}

static void Debug_RunVofaDqCycle(int32_t target_d_ma,
                                 int32_t target_q_ma)
{
    Debug_VofaStop();
    if (MotorTest_StartDqCycle(target_d_ma,
                               target_q_ma,
                               500U,
                               10000U) == 0U)
    {
        return;
    }

    s_vofa.trace_index = 0U;
    s_vofa.trace_count = 0U;
    s_vofa.test_fault = 0U;
    s_vofa.live_mode = 1U;
    s_vofa.speed_live_mode = 0U;
    s_vofa.speed_position_mode = 0U;
    s_vofa.speed_basic_mode = 0U;
    s_vofa.transfer_in_progress = 0U;
    s_vofa.active = 1U;
    s_debug.stream_enabled = 0U;
    s_debug.trace_dump_active = 0U;
    s_debug.trace_perf_pending = 0U;
}

static void Debug_ProcessCommand(const char *command)
{
    long angle_advance_us;
    long angle_observer_bandwidth_hz;
    long speed_kp_ua;
    long speed_ki_ua;
    long speed_fw_ratio_permille;
    long speed_fixed_id_ma;
    long initial_speed_rpm;
    long target_speed_rpm;
    long target_position_count;
    long position_kp_mrpm_per_count;
    long position_kd_mrpm_per_rpm;
    long position_limit_rpm;
    if ((s_vofa.active != 0U) &&
        (strcmp(command, "vofa stop") != 0) &&
        (strcmp(command, "pulse vofa off") != 0) &&
        /* 速度PI整定必须能在JustFloat采样期间在线生效。 */
        (strncmp(command, "speed tune ", 11U) != 0) &&
        !((s_vofa.position_live_mode != 0U) &&
          (strncmp(command, "vofa pos ", 9U) == 0)))
    {
        /* JustFloat连续输出期间不发送ASCII错误文本。 */
        return;
    }

    if ((SpeedLoop_GetState()->running != 0U) &&
        (strcmp(command, "speed stop") != 0) &&
        (strcmp(command, "speed status") != 0) &&
        /*
         * 位置 VOFA 实时模式下，允许继续下发相对位置步进命令。
         * 该命令只更新位置环目标，不会重启速度环或改变其 PI 参数。
         */
        !((s_vofa.position_live_mode != 0U) &&
          (strncmp(command, "vofa pos ", 9U) == 0)) &&
        (strncmp(command, "pos ", 4U) != 0) &&
        /* 脉冲模式运行期间必须允许查看计数，且可安全关闭接口。 */
        (strncmp(command, "pulse ", 6U) != 0) &&
        (strcmp(command, "angle observer status") != 0) &&
        (strncmp(command, "speed tune ", 11U) != 0) &&
        (strcmp(command, "enc") != 0) &&
        (strcmp(command, "adc") != 0) &&
        (strcmp(command, "bus") != 0) &&
        (strcmp(command, "stream on") != 0) &&
        (strcmp(command, "stream off") != 0))
    {
        Debug_SendText("err: speed loop busy\r\n");
        return;
    }

    if ((RotorAlignment_IsRunning() != 0U) &&
        (strcmp(command, "align status") != 0) &&
        (strcmp(command, "align abort") != 0) &&
        (strcmp(command, "adc") != 0) &&
        (strcmp(command, "bus") != 0) &&
        (strcmp(command, "enc") != 0) &&
        (strcmp(command, "angle") != 0))
    {
        Debug_SendText("err: rotor alignment busy\r\n");
        return;
    }

    if ((RotorAlignment_IsEncoderCalibrationRunning() != 0U) &&
        (strcmp(command, "cal status") != 0) &&
        (strcmp(command, "cal abort") != 0) &&
        (strcmp(command, "adc") != 0) &&
        (strcmp(command, "bus") != 0) &&
        (strcmp(command, "enc") != 0))
    {
        Debug_SendText("err: encoder calibration busy\r\n");
        return;
    }

    if (strcmp(command, "help") == 0)
    {
        Debug_SendText("help | enc | zero | adc | adc zero | bus | test noise | test a+ | test a- | "
                       "test b+ | test b- | test iloop a+ | test iloop a- | test iloop b+ | "
                       "test iloop b- | test dq id 300 | test dq id 500 | test dq id 1000 | test dq iq 500 | test dq iq 1000 | "
                       "trace dump | vofa id 300 | vofa id 500 | vofa id 1000 | vofa iq 500 | vofa iq 1000 | "
                       "vofa id cycle 500 | vofa iq cycle 500 | vofa speed RPM | vofa speed step FROM TO | vofa speed loop RPM | "
                       "vofa speed hold 1000 | vofa speed position 400 | "
                       "vofa pos COUNT | vofa stop | "
                       "align start | align status | align abort | "
                       "cal start | cal status | cal abort | cal dump | "
                       "angle | angle advance US | angle cal on|off|status | "
                       "angle observer off|shadow|on|calibrated|status | angle observer bw HZ | "
                       "speed tune KP_uA KI_uA | speed fw ratio PERMILLE | "
                       "speed fw fixed ID_mA | speed fw auto | "
                       "speed run RPM | speed start | "
                       "speed stop | speed status | "
                       "pos tune KP KD | pos limit RPM | pos rel COUNT | pos abs COUNT | pos status | pos stop | "
                       "pulse on | pulse off | pulse status | pulse vofa on|off | "
                       "stream on | stream off\r\n");
    }
    else if (strcmp(command, "enc") == 0)
    {
        Debug_SendEncoderState();
    }
    else if (strcmp(command, "zero") == 0)
    {
        if ((MotorPWM_IsEnabled() != 0U) ||
            (ElectricalAngle_GetState()->aligned != 0U))
        {
            Debug_SendText("err: encoder zero locked after electrical alignment\r\n");
        }
        else
        {
            Encoder_SetZero();
            Debug_SendText("ok: encoder zero set\r\n");
        }
    }
    else if (strcmp(command, "adc") == 0)
    {
        Debug_SendCurrentState();
    }
    else if (strcmp(command, "bus") == 0)
    {
        Debug_SendBusVoltage();
    }
    else if (strcmp(command, "angle") == 0)
    {
        Debug_SendElectricalAngle();
    }
    else if (strcmp(command, "angle observer status") == 0)
    {
        Debug_SendElectricalAngle();
    }
    else if ((strcmp(command, "angle cal status") == 0) ||
             (strcmp(command, "angle cal on") == 0) ||
             (strcmp(command, "angle cal off") == 0))
    {
        uint8_t enable =
            (strcmp(command, "angle cal on") == 0) ? 1U : 0U;

        if (strcmp(command, "angle cal status") == 0)
        {
            Debug_SendElectricalAngle();
        }
        else if ((MotorPWM_IsEnabled() != 0U) ||
                 (ElectricalAngle_EnableNonlinearityCalibration(enable) == 0U))
        {
            Debug_SendText(
                "err: angle calibration requires stopped loop, valid table "
                "and observer not on\r\n");
        }
        else
        {
            Debug_SendElectricalAngle();
        }
    }
    else if ((strcmp(command, "angle observer off") == 0) ||
             (strcmp(command, "angle observer shadow") == 0) ||
             (strcmp(command, "angle observer on") == 0) ||
             (strcmp(command, "angle observer calibrated") == 0))
    {
        ElectricalAngle_ObserverMode_t mode =
            ELECTRICAL_ANGLE_OBSERVER_OFF;

        if ((DqCurrentLoop_GetState()->running != 0U) ||
            (MotorPWM_IsEnabled() != 0U))
        {
            Debug_SendText(
                "err: angle observer mode requires stopped loop\r\n");
        }
        else
        {
            if (strcmp(command, "angle observer shadow") == 0)
            {
                mode = ELECTRICAL_ANGLE_OBSERVER_SHADOW;
            }
            else if (strcmp(command, "angle observer on") == 0)
            {
                mode = ELECTRICAL_ANGLE_OBSERVER_ON;
            }
            else if (strcmp(command, "angle observer calibrated") == 0)
            {
                mode = ELECTRICAL_ANGLE_OBSERVER_CALIBRATED_PREDICTIVE;
            }
            if (ElectricalAngle_SetObserverMode(mode) == 0U)
            {
                Debug_SendText(
                    "err: observer on conflicts with angle calibration\r\n");
            }
            else
            {
                ElectricalAngle_ResetObserver(
                    Encoder_GetState()->position_count);
                Debug_SendElectricalAngle();
            }
        }
    }
    else if (sscanf(command,
                    "angle observer bw %ld",
                    &angle_observer_bandwidth_hz) == 1)
    {
        if ((angle_observer_bandwidth_hz < 0L) ||
            (DqCurrentLoop_GetState()->running != 0U) ||
            (MotorPWM_IsEnabled() != 0U) ||
            (ElectricalAngle_SetObserverBandwidthHz(
                (uint32_t)angle_observer_bandwidth_hz) == 0U))
        {
            Debug_SendText(
                "err: observer bw requires stopped loop and 100..1500Hz\r\n");
        }
        else
        {
            ElectricalAngle_ResetObserver(
                Encoder_GetState()->position_count);
            Debug_SendElectricalAngle();
        }
    }
    else if (sscanf(command, "angle advance %ld", &angle_advance_us) == 1)
    {
        if ((angle_advance_us < 0L) ||
            (DqCurrentLoop_SetAngleAdvanceDelayUs(
                (uint32_t)angle_advance_us) == 0U))
        {
            Debug_SendText("err: angle advance requires stopped loop and 0..100us\r\n");
        }
        else
        {
            (void)snprintf(s_debug_tx_buffer,
                           sizeof(s_debug_tx_buffer),
                           "ok: angle advance=%ldus\r\n",
                           angle_advance_us);
            Debug_SendText(s_debug_tx_buffer);
        }
    }
    else if (sscanf(command,
                    "speed tune %ld %ld",
                    &speed_kp_ua,
                    &speed_ki_ua) == 2)
    {
        Debug_SetSpeedTunings((int32_t)speed_kp_ua,
                              (int32_t)speed_ki_ua);
    }
    else if (sscanf(command,
                    "speed fw ratio %ld",
                    &speed_fw_ratio_permille) == 1)
    {
        if ((speed_fw_ratio_permille < 0L) ||
            (MotorPWM_IsEnabled() != 0U) ||
            (SpeedLoop_SetFieldWeakeningVoltageRatioPermille(
                (uint32_t)speed_fw_ratio_permille) == 0U))
        {
            Debug_SendText(
                "err: speed fw ratio requires stopped loop and 750..900\r\n");
        }
        else
        {
            Debug_SendSpeedLoopState();
        }
    }
    else if (sscanf(command,
                    "speed fw fixed %ld",
                    &speed_fixed_id_ma) == 1)
    {
        if ((MotorPWM_IsEnabled() != 0U) ||
            (SpeedLoop_SetFixedWeakeningIdMa(
                (int32_t)speed_fixed_id_ma) == 0U))
        {
            Debug_SendText(
                "err: fixed Id requires stopped loop and -2500..-100mA\r\n");
        }
        else
        {
            Debug_SendSpeedLoopState();
        }
    }
    else if (strcmp(command, "speed fw auto") == 0)
    {
        if ((MotorPWM_IsEnabled() != 0U) ||
            (SpeedLoop_DisableFixedWeakening() == 0U))
        {
            Debug_SendText(
                "err: speed fw auto requires stopped loop\r\n");
        }
        else
        {
            Debug_SendSpeedLoopState();
        }
    }
    else if (sscanf(command, "speed run %ld", &target_speed_rpm) == 1)
    {
        if ((target_speed_rpm < -DEBUG_SPEED_COMMAND_MAXIMUM_RPM) ||
            (target_speed_rpm > DEBUG_SPEED_COMMAND_MAXIMUM_RPM))
        {
            Debug_SendText("err: speed target out of range\r\n");
        }
        else
        {
            s_speed_tune.target_speed_mrpm =
                (int32_t)target_speed_rpm * 1000L;
            Debug_StartSpeedTest(s_speed_tune.target_speed_mrpm);
        }
    }
    else if (strcmp(command, "speed start") == 0)
    {
        Debug_StartSpeedTest(s_speed_tune.target_speed_mrpm);
    }
    else if (strcmp(command, "speed stop") == 0)
    {
        PositionLoop_Stop();
        Debug_SendSpeedLoopState();
    }
    else if (strcmp(command, "speed status") == 0)
    {
        Debug_SendSpeedLoopState();
    }
    else if (strcmp(command, "pos status") == 0)
    {
        Debug_SendPositionLoopState();
        Debug_SendSpeedLoopState();
    }
    else if (strcmp(command, "pos stop") == 0)
    {
        PositionLoop_Stop();
        Debug_SendPositionLoopState();
    }
    else if (strcmp(command, "pulse vofa on") == 0)
    {
        if (Debug_StartPulseVofa() == 0U)
        {
            Debug_SendText("err: start pulse mode before pulse vofa on\r\n");
        }
    }
    else if (strcmp(command, "pulse vofa off") == 0)
    {
        Debug_VofaStop();
        Debug_SendText("ok: pulse vofa off\r\n");
    }
    else if (strcmp(command, "pulse on") == 0)
    {
        if (Debug_StartPulseInput() == 0U)
        {
            Debug_SendText("err: pulse input start failed\r\n");
        }
        else
        {
            Debug_SendText("ok: pulse input on, 800 step/rev\r\n");
            Debug_SendPulseInputState();
        }
    }
    else if (strcmp(command, "pulse off") == 0)
    {
        /* 产品态不允许USB关闭接口，外部PUL_EN才是唯一控制门控。 */
        Debug_SendText("err: pulse input is always armed; use PUL_EN\r\n");
        Debug_SendPulseInputState();
    }
    else if (strcmp(command, "pulse status") == 0)
    {
        Debug_SendPulseInputState();
        Debug_SendPositionLoopState();
    }
    else if (sscanf(command, "vofa pos diag %ld", &target_position_count) == 1)
    {
        if ((s_vofa.position_live_mode == 0U) &&
            (Debug_StartVofaPositionTest((int32_t)target_position_count,
                                         1U, 0U) == 0U))
        {
            Debug_SendText("err: vofa position diagnostic start failed\r\n");
        }
    }
    else if (sscanf(command, "vofa pos capture %ld", &target_position_count) == 1)
    {
        if ((s_vofa.position_live_mode == 0U) &&
            (Debug_StartVofaPositionTest((int32_t)target_position_count,
                                         0U, 1U) == 0U))
        {
            Debug_SendText("err: vofa position capture start failed\r\n");
        }
    }
    else if (sscanf(command, "vofa pos %ld", &target_position_count) == 1)
    {
        int64_t next_target;

        if ((s_vofa.position_live_mode != 0U) &&
            (PositionLoop_GetState()->running != 0U))
        {
            next_target =
                (int64_t)PositionLoop_GetState()->target_position_count +
                target_position_count;
            if ((next_target > INT32_MAX) || (next_target < INT32_MIN) ||
                (PositionLoop_SetTargetPositionCount((int32_t)next_target) == 0U))
            {
                /* JustFloat期间不能混入ASCII；无效命令直接停止输出。 */
                PositionLoop_Stop();
                Debug_VofaStop();
            }
        }
        else if (Debug_StartVofaPositionTest((int32_t)target_position_count,
                                              0U, 0U) == 0U)
        {
            Debug_SendText("err: vofa position start failed\r\n");
        }
    }
    else if (sscanf(command, "pos tune %ld %ld",
                    &position_kp_mrpm_per_count,
                    &position_kd_mrpm_per_rpm) == 2)
    {
        Debug_SetPositionTunings((int32_t)position_kp_mrpm_per_count,
                                 (int32_t)position_kd_mrpm_per_rpm);
    }
    else if (sscanf(command, "pos limit %ld", &position_limit_rpm) == 1)
    {
        if ((position_limit_rpm < 1L) ||
            (position_limit_rpm >
             (DEBUG_POSITION_SPEED_LIMIT_MAX_MRPM / 1000L)))
        {
            Debug_SendText("err: position speed limit out of range\r\n");
        }
        else
        {
            Debug_SetPositionSpeedLimit(
                (int32_t)(position_limit_rpm * 1000L));
        }
    }
    else if (sscanf(command, "pos rel %ld", &target_position_count) == 1)
    {
        int64_t requested_target;

        if (PositionLoop_GetState()->running != 0U)
        {
            requested_target =
                (int64_t)PositionLoop_GetState()->target_position_count +
                target_position_count;
            if ((requested_target > INT32_MAX) || (requested_target < INT32_MIN) ||
                (PositionLoop_SetTargetPositionCount((int32_t)requested_target) == 0U))
            {
                Debug_SendText("err: position relative target rejected\r\n");
            }
            else
            {
                Debug_SendPositionLoopState();
            }
        }
        else
        {
            requested_target =
                (int64_t)Encoder_GetPositionCountFast() + target_position_count;
            if ((requested_target > INT32_MAX) || (requested_target < INT32_MIN) ||
                (Debug_StartPositionTest((int32_t)requested_target) == 0U))
            {
                Debug_SendText("err: position loop start failed\r\n");
            }
            else
            {
                s_debug.stream_enabled = 1U;
                s_debug.last_stream_tick_ms = HAL_GetTick();
                Debug_SendPositionLoopState();
            }
        }
    }
    else if (sscanf(command, "pos abs %ld", &target_position_count) == 1)
    {
        if (((PositionLoop_GetState()->running != 0U) &&
             (PositionLoop_SetTargetPositionCount((int32_t)target_position_count) == 0U)) ||
            ((PositionLoop_GetState()->running == 0U) &&
             (Debug_StartPositionTest((int32_t)target_position_count) == 0U)))
        {
            Debug_SendText("err: position target rejected\r\n");
        }
        else
        {
            s_debug.stream_enabled = 1U;
            s_debug.last_stream_tick_ms = HAL_GetTick();
            Debug_SendPositionLoopState();
        }
    }
    else if (strcmp(command, "align start") == 0)
    {
        if (RotorAlignment_Start() == 0U)
        {
            Debug_SendText("err: alignment precondition failed\r\n");
        }
        else
        {
            Debug_SendText("ok: alignment started\r\n");
        }
    }
    else if (strcmp(command, "align status") == 0)
    {
        Debug_SendAlignmentState();
    }
    else if (strcmp(command, "align abort") == 0)
    {
        RotorAlignment_Abort();
        Debug_SendAlignmentState();
    }
    else if (strcmp(command, "cal start") == 0)
    {
        if (RotorAlignment_StartEncoderCalibration() == 0U)
        {
            Debug_SendText("err: calibration precondition failed\r\n");
            Debug_SendEncoderCalibrationState();
        }
        else
        {
            Debug_SendText("ok: encoder calibration started\r\n");
        }
    }
    else if (strcmp(command, "cal status") == 0)
    {
        Debug_SendEncoderCalibrationState();
    }
    else if (strcmp(command, "cal abort") == 0)
    {
        RotorAlignment_AbortEncoderCalibration();
        Debug_SendEncoderCalibrationState();
    }
    else if (strcmp(command, "cal dump") == 0)
    {
        Debug_StartEncoderCalibrationDump();
    }
    else if (strcmp(command, "adc zero") == 0)
    {
        if (MotorPWM_IsEnabled() != 0U)
        {
            Debug_SendText("err: stop pwm before adc calibration\r\n");
        }
        else if (CurrentSense_CalibrateOffsets() == 0U)
        {
            Debug_SendText("err: adc dma not started\r\n");
        }
        else
        {
            Debug_SendCurrentState();
        }
    }
    else if (strcmp(command, "test a+") == 0)
    {
        Debug_RunMotorTest(MOTOR_PWM_PHASE_A, 1);
    }
    else if (strcmp(command, "test noise") == 0)
    {
        Debug_RunCurrentNoiseTest();
    }
    else if (strcmp(command, "test iloop a+") == 0)
    {
        Debug_RunClosedLoopCurrentTest(300L, 0L);
    }
    else if (strcmp(command, "test iloop a-") == 0)
    {
        Debug_RunClosedLoopCurrentTest(-300L, 0L);
    }
    else if (strcmp(command, "test iloop b+") == 0)
    {
        Debug_RunClosedLoopCurrentTest(0L, 300L);
    }
    else if (strcmp(command, "test iloop b-") == 0)
    {
        Debug_RunClosedLoopCurrentTest(0L, -300L);
    }
    else if (strcmp(command, "test dq id 300") == 0)
    {
        Debug_RunDqCurrentTest(300L, 0L);
    }
    else if (strcmp(command, "test dq id 500") == 0)
    {
        Debug_RunDqCurrentTest(500L, 0L);
    }
    else if ((strcmp(command, "test dq id 1000") == 0) ||
             (strcmp(command, "test dq id") == 0))
    {
        Debug_RunDqCurrentTest(1000L, 0L);
    }
    else if (strcmp(command, "test dq iq 500") == 0)
    {
        Debug_RunDqCurrentTest(0L, 500L);
    }
    else if (strcmp(command, "test dq iq 1000") == 0)
    {
        Debug_RunDqCurrentTest(0L, 1000L);
    }
    else if (strcmp(command, "vofa id 300") == 0)
    {
        Debug_RunVofaDqTest(300L, 0L);
    }
    else if (strcmp(command, "vofa id 500") == 0)
    {
        Debug_RunVofaDqTest(500L, 0L);
    }
    else if ((strcmp(command, "vofa id 1000") == 0) ||
             (strcmp(command, "vofa id") == 0))
    {
        Debug_RunVofaDqTest(1000L, 0L);
    }
    else if (strcmp(command, "vofa iq 500") == 0)
    {
        Debug_RunVofaDqTest(0L, 500L);
    }
    else if (strcmp(command, "vofa iq 1000") == 0)
    {
        Debug_RunVofaDqTest(0L, 1000L);
    }
    else if (strcmp(command, "vofa id cycle 500") == 0)
    {
        Debug_RunVofaDqCycle(500L, 0L);
    }
    else if (strcmp(command, "vofa iq cycle 500") == 0)
    {
        Debug_RunVofaDqCycle(0L, 500L);
    }
    else if (sscanf(command,
                    "vofa speed step %ld %ld",
                    &initial_speed_rpm,
                    &target_speed_rpm) == 2)
    {
        if ((initial_speed_rpm < -DEBUG_SPEED_COMMAND_MAXIMUM_RPM) ||
            (initial_speed_rpm > DEBUG_SPEED_COMMAND_MAXIMUM_RPM) ||
            (target_speed_rpm < -DEBUG_SPEED_COMMAND_MAXIMUM_RPM) ||
            (target_speed_rpm > DEBUG_SPEED_COMMAND_MAXIMUM_RPM) ||
            (initial_speed_rpm == target_speed_rpm))
        {
            Debug_SendText("err: invalid speed step command\r\n");
        }
        else
        {
            s_speed_tune.target_speed_mrpm =
                (int32_t)target_speed_rpm * 1000L;
            Debug_StartVofaSpeedStepTest(
                (int32_t)initial_speed_rpm * 1000L,
                s_speed_tune.target_speed_mrpm);
        }
    }
    else if (sscanf(command,
                    "vofa speed hold %ld",
                    &target_speed_rpm) == 1)
    {
        if ((target_speed_rpm != 1000L) &&
            (target_speed_rpm != -1000L))
        {
            Debug_SendText(
                "err: current hold diagnostic requires +/-1000rpm\r\n");
        }
        else
        {
            s_speed_tune.target_speed_mrpm =
                (int32_t)target_speed_rpm * 1000L;
            Debug_StartVofaSpeedTest(
                s_speed_tune.target_speed_mrpm,
                1U,
                0U,
                0U);
        }
    }
    else if (sscanf(command,
                    "vofa speed position %ld",
                    &target_speed_rpm) == 1)
    {
        if ((target_speed_rpm != 400L) &&
            (target_speed_rpm != -400L))
        {
            Debug_SendText(
                "err: position diagnostic requires +/-400rpm\r\n");
        }
        else if (SpeedLoop_GetState()->fixed_weakening_enabled == 0U)
        {
            Debug_SendText(
                "err: set fixed Id before position diagnostic\r\n");
        }
        else
        {
            s_speed_tune.target_speed_mrpm =
                (int32_t)target_speed_rpm * 1000L;
            Debug_StartVofaSpeedTest(
                s_speed_tune.target_speed_mrpm,
                0U,
                1U,
                0U);
        }
    }
    else if (sscanf(command, "vofa speed loop %ld", &target_speed_rpm) == 1)
    {
        if ((target_speed_rpm < -DEBUG_SPEED_COMMAND_MAXIMUM_RPM) ||
            (target_speed_rpm > DEBUG_SPEED_COMMAND_MAXIMUM_RPM))
        {
            Debug_SendText("err: speed target out of range\r\n");
        }
        else
        {
            s_speed_tune.target_speed_mrpm =
                (int32_t)target_speed_rpm * 1000L;
            Debug_StartVofaSpeedTest(s_speed_tune.target_speed_mrpm,
                                     0U,
                                     0U,
                                     1U);
        }
    }
    else if (sscanf(command, "vofa speed %ld", &target_speed_rpm) == 1)
    {
        if ((target_speed_rpm < -DEBUG_SPEED_COMMAND_MAXIMUM_RPM) ||
            (target_speed_rpm > DEBUG_SPEED_COMMAND_MAXIMUM_RPM))
        {
            Debug_SendText("err: speed target out of range\r\n");
        }
        else
        {
            s_speed_tune.target_speed_mrpm =
                (int32_t)target_speed_rpm * 1000L;
            Debug_StartVofaSpeedTest(s_speed_tune.target_speed_mrpm,
                                     0U,
                                     0U,
                                     0U);
        }
    }
    else if (strcmp(command, "vofa stop") == 0)
    {
        if (s_vofa.speed_live_mode != 0U)
        {
            SpeedLoop_Stop();
        }
        if (s_vofa.position_live_mode != 0U)
        {
            PositionLoop_Stop();
        }
        MotorTest_StopDqCycle();
        Debug_VofaStop();
    }
    else if (strcmp(command, "trace dump") == 0)
    {
        Debug_StartDqTraceDump();
        if (s_debug.trace_dump_active == 0U)
        {
            Debug_SendText("err: no dq trace available\r\n");
        }
    }
    else if (strcmp(command, "test a-") == 0)
    {
        Debug_RunMotorTest(MOTOR_PWM_PHASE_A, -1);
    }
    else if (strcmp(command, "test b+") == 0)
    {
        Debug_RunMotorTest(MOTOR_PWM_PHASE_B, 1);
    }
    else if (strcmp(command, "test b-") == 0)
    {
        Debug_RunMotorTest(MOTOR_PWM_PHASE_B, -1);
    }
    else if (strcmp(command, "stream on") == 0)
    {
        s_debug.stream_enabled = 1U;
        Debug_SendText("ok: encoder stream on\r\n");
    }
    else if (strcmp(command, "stream off") == 0)
    {
        s_debug.stream_enabled = 0U;
        Debug_SendText("ok: encoder stream off\r\n");
    }
    else
    {
        Debug_SendText("err: unknown command\r\n");
    }
}

void Debug_Init(void)
{
    s_debug.rx_length = 0U;
    s_debug.command_ready = 0U;
    s_debug.stop_requested = 0U;
    s_debug.stream_enabled = 0U;
    s_debug.last_stream_tick_ms = HAL_GetTick();
    s_debug.trace_dump_index = 0U;
    s_debug.trace_dump_count = 0U;
    s_debug.trace_dump_active = 0U;
    s_debug.trace_header_sent = 0U;
    s_debug.trace_perf_pending = 0U;
    s_debug.calibration_dump_index = 0U;
    s_debug.calibration_dump_active = 0U;
    s_debug.calibration_header_sent = 0U;
    s_debug.trace_perf_buffer[0] = '\0';
    s_debug.trace_fall_buffer[0] = '\0';
    s_debug.pending_text[0] = '\0';
    s_debug.pending_text_valid = 0U;
    s_vofa.trace_index = 0U;
    s_vofa.trace_count = 0U;
    s_vofa.test_fault = 0U;
    s_vofa.active = 0U;
    s_vofa.live_mode = 0U;
    s_vofa.speed_live_mode = 0U;
    s_vofa.speed_position_mode = 0U;
    s_vofa.speed_basic_mode = 0U;
    s_vofa.speed_step_active = 0U;
    s_vofa.speed_step_switch_ms = 0U;
    s_vofa.speed_step_target_mrpm = 0L;
    s_vofa.position_live_mode = 0U;
    s_vofa.position_diagnostic_mode = 0U;
    s_vofa.position_capture_mode = 0U;
    s_vofa.position_capture_sequence = 0U;
    s_vofa.pulse_live_mode = 0U;
    s_vofa.pulse_sequence = 0U;
    s_vofa.pulse_last_sample_tick_ms = 0U;
    s_vofa.transfer_in_progress = 0U;
    s_speed_tune.kp_ua_per_rpm = SPEED_LOOP_DEFAULT_KP_UA_PER_RPM;
    s_speed_tune.ki_ua_per_rpm_s = SPEED_LOOP_DEFAULT_KI_UA_PER_RPM_S;
    s_speed_tune.target_speed_mrpm = DEBUG_SPEED_TEST_TARGET_MRPM;
    s_position_tune.kp_mrpm_per_count =
        DEBUG_POSITION_TEST_KP_MRPM_PER_COUNT;
    s_position_tune.kd_mrpm_per_rpm =
        DEBUG_POSITION_TEST_KD_MRPM_PER_RPM;
    s_position_tune.maximum_speed_mrpm =
        DEBUG_POSITION_TEST_MAXIMUM_MRPM;
}

void Debug_Loop(void)
{
    char command[DEBUG_RX_BUFFER_SIZE];

    if (s_vofa.active == 0U)
    {
        Debug_ProcessPendingText();
    }

    /* 停止请求优先于普通命令和连续遥测，避免调试数据阻塞停机。 */
    if (s_debug.stop_requested != 0U)
    {
        __disable_irq();
        s_debug.stop_requested = 0U;
        s_debug.command_ready = 0U;
        __enable_irq();
        PositionLoop_Stop();
        MotorTest_StopDqCycle();
        Debug_VofaStop();
        return;
    }

    if (s_debug.command_ready == 0U)
    {
        return;
    }

    __disable_irq();
    (void)strncpy(command, s_debug.rx_buffer, sizeof(command) - 1U);
    command[sizeof(command) - 1U] = '\0';
    s_debug.command_ready = 0U;
    __enable_irq();

    Debug_ProcessCommand(command);
}

void Debug_TelemetryLoop(void)
{
    uint32_t now_ms;

    if (s_vofa.active != 0U)
    {
        Debug_VofaLoop();
        return;
    }

    if (s_debug.trace_dump_active != 0U)
    {
        Debug_DumpDqTraceLoop();
        return;
    }

    if (s_debug.calibration_dump_active != 0U)
    {
        Debug_DumpEncoderCalibrationLoop();
        return;
    }

    if (s_debug.stream_enabled == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - s_debug.last_stream_tick_ms) <
        ((SpeedLoop_GetState()->running != 0U) ?
         DEBUG_SPEED_STREAM_PERIOD_MS : DEBUG_STREAM_PERIOD_MS))
    {
        return;
    }

    s_debug.last_stream_tick_ms = now_ms;
    if ((SpeedLoop_GetState()->running != 0U) ||
        (SpeedLoop_GetState()->fault != SPEED_LOOP_FAULT_NONE))
    {
        Debug_SendSpeedLoopState();
        if (PositionLoop_GetState()->running != 0U)
        {
            Debug_SendPositionLoopState();
        }
    }
    else
    {
        Debug_SendEncoderState();
    }
}

void Debug_CdcRxCallback(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    if (data == 0)
    {
        return;
    }

    for (index = 0U; index < length; index++)
    {
        uint8_t byte = data[index];

        if ((byte == '\r') || (byte == '\n'))
        {
            if (s_debug.rx_length > 0U)
            {
                s_debug.rx_buffer[s_debug.rx_length] = '\0';
                s_debug.rx_length = 0U;
                if ((strcmp(s_debug.rx_buffer, "vofa stop") == 0) ||
                    (strcmp(s_debug.rx_buffer, "speed stop") == 0) ||
                    (strcmp(s_debug.rx_buffer, "pos stop") == 0))
                {
                    /* 仅置位请求，具体停机操作在主循环执行。 */
                    s_debug.stop_requested = 1U;
                }
                else
                {
                    s_debug.command_ready = 1U;
                }
            }
        }
        else if ((s_debug.command_ready == 0U) &&
                 (s_debug.rx_length < (DEBUG_RX_BUFFER_SIZE - 1U)))
        {
            s_debug.rx_buffer[s_debug.rx_length] = (char)byte;
            s_debug.rx_length++;
        }
        else
        {
            /* 命令过长或上一条命令未处理时丢弃本帧，防止覆盖完整命令。 */
            s_debug.rx_length = 0U;
        }
    }
}
