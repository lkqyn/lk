/* 1 kHz 速度 PI、弱磁、电压预算和母线保护的实现。 */
#include "speed_loop.h"

#include "dq_current_loop.h"
#include "encoder.h"
#include "motor_parameters.h"
#include "power_monitor.h"
#include "stm32f4xx_hal.h"

#include <limits.h>
#include <math.h>

#define SPEED_LOOP_CURRENT_FREQUENCY_HZ       (20000.0f)
#define SPEED_LOOP_CURRENT_BANDWIDTH_HZ       (1200.0f)
#define SPEED_LOOP_VOLTAGE_LIMIT_MV           (20000UL)
#define SPEED_LOOP_PWM_UTILIZATION_PERMILLE    (840UL)
/* 3.5A目标电流圆与4.0A瞬时保护之间保留采样纹波余量。 */
#define SPEED_LOOP_OVERCURRENT_LIMIT_MA       (4000L)
#define SPEED_LOOP_BUS_VOLTAGE_MIN_MV         (18000UL)
/*
 * 35 V is the final hardware-fault threshold for the 24 V bus.  It is
 * intentionally above normal regeneration peaks so a braking profile is not
 * aborted at 30 V, while remaining well below the verified 100 V parts.
 */
#define SPEED_LOOP_BUS_VOLTAGE_MAX_MV         (35000UL)
#define SPEED_LOOP_REGEN_FULL_BRAKE_MV         (28000L)
#define SPEED_LOOP_REGEN_REDUCED_BRAKE_MV      (34000L)
#define SPEED_LOOP_REGEN_MINIMUM_BRAKE_MA        (600L)
#define SPEED_LOOP_DIRECTION_CHECK_DELAY_MS   (100U)
#define SPEED_LOOP_DIRECTION_CHECK_SPEED_MRPM (10000L)
#define SPEED_LOOP_UA_PER_MA                   (1000L)
#define SPEED_LOOP_TRACE_BUFFER_SIZE           (128U)
#define SPEED_LOOP_TWO_PI                      (6.2831853071795864769f)
#define SPEED_LOOP_MA_PER_A                    (1000.0f)
#define SPEED_LOOP_MV_PER_V                    (1000.0f)
#define SPEED_LOOP_MRPM_PER_RPM                (1000.0f)
#define SPEED_LOOP_SECONDS_PER_MINUTE          (60.0f)
#define SPEED_LOOP_MINIMUM_ANGULAR_SPEED_RAD_S (1.0f)
#define SPEED_LOOP_PHASE_INDEXES                (1024L)
#define SPEED_LOOP_HALF_PHASE_INDEXES           (512L)
#define SPEED_LOOP_ELECTRICAL_CIRCLE_MDEG       (360000L)
#define SPEED_LOOP_WEAKENING_SEARCH_ITERATIONS (16U)
#define SPEED_LOOP_TORQUE_LINEARIZATION_PASSES  (4U)
#define SPEED_LOOP_ANTIWINDUP_TOLERANCE_MA       (2L)
/* 1kHz下20个样本对应约20ms母线电压一阶低通时间常数。 */
#define SPEED_LOOP_BUS_FILTER_DIVISOR          (20L)
/* 弱磁模型额外使用20ms一阶速度滤波，不增加速度PI反馈延迟。 */
#define SPEED_LOOP_WEAKENING_SPEED_FILTER_DIVISOR (20L)
/*
 * 电压余量闭环只作用于1kHz弱磁规划器，不进入20kHz电流PI。
 * 50ms滤波抑制步进电机电角度离散和电压谐波对弱磁修正量的调制。
 */
#define SPEED_LOOP_FW_VOLTAGE_FILTER_DIVISOR      (50L)
#define SPEED_LOOP_FW_VOLTAGE_INCREASE_DIVISOR    (50L)
#define SPEED_LOOP_FW_VOLTAGE_RELEASE_DIVISOR     (200L)
#define SPEED_LOOP_FW_VOLTAGE_DEADBAND_MV          (100L)
#define SPEED_LOOP_PERMILLE_SCALE                  (1000LL)
#define SPEED_LOOP_FIXED_ID_MINIMUM_MA             (-2500L)
#define SPEED_LOOP_FIXED_ID_MAXIMUM_MA              (-100L)

typedef struct
{
    int32_t id_target_ma;
    int32_t iq_target_ma;
    int32_t feedforward_d_mv;
    int32_t feedforward_q_mv;
    uint8_t field_weakening_active;
} SpeedLoop_OperatingPoint_t;

typedef struct
{
    SpeedLoop_Config_t config;
    volatile SpeedLoop_State_t state;
    int64_t integral_ua;
    int32_t maximum_voltage_mv;
    int32_t filtered_bus_voltage_mv;
    int32_t filtered_weakening_speed_mrpm;
    int32_t filtered_voltage_magnitude_mv;
    int32_t field_weakening_voltage_correction_mv;
    uint32_t field_weakening_voltage_ratio_permille;
    int32_t fixed_weakening_id_ma;
    uint8_t fixed_weakening_enabled;
    int32_t held_logical_iq_ma;
    int32_t held_id_target_ma;
    int32_t held_iq_target_ma;
    int32_t held_torque_equivalent_iq_ma;
    volatile int32_t external_torque_override_ma;
    volatile uint8_t external_torque_override_active;
    volatile int32_t external_braking_current_limit_ma;
    volatile uint16_t trace_write_index;
    volatile uint16_t trace_read_index;
    volatile uint32_t trace_dropped_count;
    SpeedLoop_TraceSample_t trace_buffer[SPEED_LOOP_TRACE_BUFFER_SIZE];
} SpeedLoop_Context_t;

static SpeedLoop_Context_t s_speed_loop;

/*
 * 制动电流是向母线回馈的电流。24V附近允许完整制动；母线升高时线性
 * 降额，34V以上仅保留小制动力，避免把电压顶至35V硬件保护。
 */
/* 母线接近上限时限制与实际速度反向的回馈制动 Iq，保护直流母线。 */
static int32_t SpeedLoop_LimitRegenerativeBrakeCurrentMa(
    int32_t requested_limit_ma,
    int32_t bus_voltage_mv)
{
    int64_t reduced_limit_ma;

    if ((requested_limit_ma <= 0L) ||
        (requested_limit_ma <= SPEED_LOOP_REGEN_MINIMUM_BRAKE_MA) ||
        (bus_voltage_mv <= SPEED_LOOP_REGEN_FULL_BRAKE_MV))
    {
        return requested_limit_ma;
    }
    if (bus_voltage_mv >= SPEED_LOOP_REGEN_REDUCED_BRAKE_MV)
    {
        return SPEED_LOOP_REGEN_MINIMUM_BRAKE_MA;
    }
    reduced_limit_ma = (int64_t)requested_limit_ma -
        (((int64_t)(requested_limit_ma - SPEED_LOOP_REGEN_MINIMUM_BRAKE_MA) *
          (bus_voltage_mv - SPEED_LOOP_REGEN_FULL_BRAKE_MV)) /
         (SPEED_LOOP_REGEN_REDUCED_BRAKE_MV -
          SPEED_LOOP_REGEN_FULL_BRAKE_MV));
    return (int32_t)reduced_limit_ma;
}

/* 以千分比缩放有符号整数，供母线制动降额和弱磁比例使用。 */
static int32_t SpeedLoop_ApplyPermille(int32_t value,
                                       uint32_t ratio_permille)
{
    return (int32_t)((((int64_t)value * ratio_permille) +
                      (SPEED_LOOP_PERMILLE_SCALE / 2LL)) /
                     SPEED_LOOP_PERMILLE_SCALE);
}

/* 计算观测电角与编码器电角的相位差，供速度环诊断使用。 */
static int32_t SpeedLoop_CalculateObserverPhaseErrorMdeg(
    const volatile DqCurrentLoop_State_t *current_loop)
{
    int32_t phase_error = current_loop->raw_phase_index -
                          current_loop->observer_phase_index;

    if (phase_error > SPEED_LOOP_HALF_PHASE_INDEXES)
    {
        phase_error -= SPEED_LOOP_PHASE_INDEXES;
    }
    else if (phase_error < -SPEED_LOOP_HALF_PHASE_INDEXES)
    {
        phase_error += SPEED_LOOP_PHASE_INDEXES;
    }

    return (int32_t)(((int64_t)phase_error *
                      SPEED_LOOP_ELECTRICAL_CIRCLE_MDEG) /
                     SPEED_LOOP_PHASE_INDEXES);
}

/* 清空 1 kHz 整定追踪环形缓冲区。 */
static void SpeedLoop_ResetTrace(void)
{
    s_speed_loop.trace_write_index = 0U;
    s_speed_loop.trace_read_index = 0U;
    s_speed_loop.trace_dropped_count = 0U;
}

/* 向 RAM 环形队列写入一个速度环诊断样本，不进行串口操作。 */
static void SpeedLoop_PushTraceSample(
    const volatile DqCurrentLoop_State_t *current_loop)
{
    Encoder_SpeedDiagnostic_t encoder;
    SpeedLoop_TraceSample_t *sample;
    uint16_t write_index;
    uint16_t next_write_index;
    uint16_t next_read_index;

    Encoder_GetSpeedDiagnosticFast(&encoder);
    write_index = s_speed_loop.trace_write_index;
    next_write_index = (uint16_t)(write_index + 1U);
    if (next_write_index >= SPEED_LOOP_TRACE_BUFFER_SIZE)
    {
        next_write_index = 0U;
    }

    /* 队列满时覆盖最旧样本，优先保持串口输出的实时性。 */
    if (next_write_index == s_speed_loop.trace_read_index)
    {
        next_read_index = (uint16_t)(s_speed_loop.trace_read_index + 1U);
        if (next_read_index >= SPEED_LOOP_TRACE_BUFFER_SIZE)
        {
            next_read_index = 0U;
        }
        s_speed_loop.trace_read_index = next_read_index;
        s_speed_loop.trace_dropped_count++;
    }

    sample = &s_speed_loop.trace_buffer[write_index];
    sample->elapsed_ms = s_speed_loop.state.elapsed_ms;
    sample->target_speed_mrpm = s_speed_loop.state.target_speed_mrpm;
    sample->adaptive_speed_mrpm = encoder.adaptive_speed_mrpm;
    sample->fixed_5ms_speed_mrpm = encoder.fixed_5ms_speed_mrpm;
    sample->fixed_10ms_speed_mrpm = encoder.fixed_10ms_speed_mrpm;
    sample->encoder_delta_count = encoder.sample_delta_count;
    sample->iq_target_ma = s_speed_loop.state.logical_iq_target_ma;
    sample->iq_measured_ma = current_loop->measured_q_ma;
    sample->id_measured_ma = current_loop->measured_d_ma;
    sample->electrical_count = current_loop->electrical_count;
    sample->mechanical_count_in_revolution =
        current_loop->encoder_position_count %
        MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
    if (sample->mechanical_count_in_revolution < 0L)
    {
        sample->mechanical_count_in_revolution +=
            MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
    }
    sample->output_d_mv = current_loop->output_d_mv;
    sample->output_q_mv = current_loop->output_q_mv;
    sample->feedforward_d_mv = current_loop->feedforward_d_mv;
    sample->feedforward_q_mv = current_loop->feedforward_q_mv;
    sample->voltage_saturated_sample_count =
        current_loop->saturated_sample_count;
    sample->d_axis_priority_sample_count =
        current_loop->d_axis_priority_sample_count;
    sample->id_target_ma = s_speed_loop.state.current_loop_id_target_ma;
    sample->current_loop_iq_target_ma =
        s_speed_loop.state.current_loop_iq_target_ma;
    sample->torque_equivalent_iq_ma =
        s_speed_loop.state.torque_equivalent_iq_ma;
    sample->integral_output_ma = s_speed_loop.state.integral_output_ma;
    sample->effective_kp_ua_per_rpm =
        s_speed_loop.state.effective_kp_ua_per_rpm;
    sample->field_weakening_voltage_correction_mv =
        s_speed_loop.state.field_weakening_voltage_correction_mv;
    sample->observer_phase_error_mdeg =
        SpeedLoop_CalculateObserverPhaseErrorMdeg(current_loop);
    s_speed_loop.trace_write_index = next_write_index;
}

/* 返回饱和绝对值，供速度、电流和故障阈值判断。 */
static int32_t SpeedLoop_Absolute(int32_t value)
{
    if (value == INT32_MIN)
    {
        return INT32_MAX;
    }
    return (value < 0L) ? -value : value;
}

/* 将 64 位 PI/工作点中间值限制在指定范围。 */
static int64_t SpeedLoop_Clamp64(int64_t value,
                                 int64_t minimum,
                                 int64_t maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

/* 将微安内部量按四舍五入转换为毫安输出量。 */
static int32_t SpeedLoop_UaToMa(int64_t current_ua)
{
    if (current_ua >= 0LL)
    {
        return (int32_t)((current_ua + 500LL) / SPEED_LOOP_UA_PER_MA);
    }
    return (int32_t)((current_ua - 500LL) / SPEED_LOOP_UA_PER_MA);
}

/* 将弱磁和电压模型的浮点计算结果转换为整数。 */
static int32_t SpeedLoop_RoundFloat(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                             (int32_t)(value - 0.5f);
}

/* 将物理 d/q 工作点折算为产生同等转矩的 q 轴电流。 */
static int32_t SpeedLoop_CalculateTorqueEquivalentIqMa(
    int32_t id_target_ma,
    int32_t iq_target_ma,
    int8_t torque_direction_sign)
{
    float id_a = (float)id_target_ma / SPEED_LOOP_MA_PER_A;
    float logical_iq_a =
        ((float)iq_target_ma * (float)torque_direction_sign) /
        SPEED_LOOP_MA_PER_A;
    float effective_torque_flux_wb =
        MOTOR_FLUX_LINKAGE_WB +
        ((MOTOR_D_AXIS_INDUCTANCE_H - MOTOR_Q_AXIS_INDUCTANCE_H) * id_a);

    return SpeedLoop_RoundFloat(
        logical_iq_a * effective_torque_flux_wb /
        MOTOR_FLUX_LINKAGE_WB * SPEED_LOOP_MA_PER_A);
}

/* 根据电机模型与目标转矩，求受弱磁影响后的实际 q 轴请求。 */
static int32_t SpeedLoop_CalculatePhysicalIqRequestMa(
    int32_t torque_equivalent_iq_ma,
    int32_t id_target_ma)
{
    float id_a = (float)id_target_ma / SPEED_LOOP_MA_PER_A;
    float effective_torque_flux_wb =
        MOTOR_FLUX_LINKAGE_WB +
        ((MOTOR_D_AXIS_INDUCTANCE_H - MOTOR_Q_AXIS_INDUCTANCE_H) * id_a);
    float physical_iq_ma;

    /*
     * 当模型参数异常使等效转矩磁链接近零时禁止除法，
     * 正常弱磁区内该值始终为正。
     */
    if (effective_torque_flux_wb <
        (0.1f * MOTOR_FLUX_LINKAGE_WB))
    {
        effective_torque_flux_wb = 0.1f * MOTOR_FLUX_LINKAGE_WB;
    }

    physical_iq_ma =
        (float)torque_equivalent_iq_ma * MOTOR_FLUX_LINKAGE_WB /
        effective_torque_flux_wb;
    return SpeedLoop_RoundFloat(physical_iq_ma);
}

/* 返回两个浮点限制值中较小者。 */
static float SpeedLoop_MinFloat(float value_1, float value_2)
{
    return (value_1 < value_2) ? value_1 : value_2;
}

/* 根据实时母线电压和配置比例计算 d/q 可用电压上限。 */
static int32_t SpeedLoop_CalculateVoltageLimitMv(uint32_t bus_voltage_mv)
{
    uint32_t voltage_limit_mv =
        (bus_voltage_mv * SPEED_LOOP_PWM_UTILIZATION_PERMILLE) / 1000UL;

    if (voltage_limit_mv > SPEED_LOOP_VOLTAGE_LIMIT_MV)
    {
        voltage_limit_mv = SPEED_LOOP_VOLTAGE_LIMIT_MV;
    }
    return (int32_t)voltage_limit_mv;
}

/* 对母线电压作低通滤波，避免制动/弱磁限幅因 ADC 噪声跳变。 */
static int32_t SpeedLoop_FilterBusVoltageMv(uint32_t measured_voltage_mv)
{
    int32_t error_mv =
        (int32_t)measured_voltage_mv - s_speed_loop.filtered_bus_voltage_mv;

    if (error_mv >= 0L)
    {
        error_mv += SPEED_LOOP_BUS_FILTER_DIVISOR / 2L;
    }
    else
    {
        error_mv -= SPEED_LOOP_BUS_FILTER_DIVISOR / 2L;
    }
    s_speed_loop.filtered_bus_voltage_mv +=
        error_mv / SPEED_LOOP_BUS_FILTER_DIVISOR;
    return s_speed_loop.filtered_bus_voltage_mv;
}

/* 对弱磁规划使用的转速作滤波，降低高速换向时的指令突变。 */
static int32_t SpeedLoop_FilterWeakeningSpeedMrpm(
    int32_t measured_speed_mrpm)
{
    int32_t filter_error_mrpm = measured_speed_mrpm -
        s_speed_loop.filtered_weakening_speed_mrpm;

    /* 对正负转速采用对称舍入，避免整数除法产生方向相关偏差。 */
    if (filter_error_mrpm >= 0L)
    {
        filter_error_mrpm +=
            SPEED_LOOP_WEAKENING_SPEED_FILTER_DIVISOR / 2L;
    }
    else
    {
        filter_error_mrpm -=
            SPEED_LOOP_WEAKENING_SPEED_FILTER_DIVISOR / 2L;
    }
    s_speed_loop.filtered_weakening_speed_mrpm +=
        filter_error_mrpm /
        SPEED_LOOP_WEAKENING_SPEED_FILTER_DIVISOR;
    return s_speed_loop.filtered_weakening_speed_mrpm;
}

/*
 * 弱磁规划不能只看滞后的实测速度：高加速度位置轨迹中，电机尚未达到
 * 给定速度前，反电动势裕量已需要提前准备。加速时采用幅值更大的速度
 * 给定进行预弱磁和电压前馈；减速时实测速度幅值更大，仍以实测值确保
 * 不会过早退出弱磁。速度PI的反馈量始终保持实测速度，不受本函数影响。
 */
/* 在目标速度与实测速度之间选择弱磁电压预算的规划速度。 */
static int32_t SpeedLoop_SelectWeakeningPlanningSpeedMrpm(
    int32_t filtered_measured_speed_mrpm,
    int32_t target_speed_mrpm)
{
    if (SpeedLoop_Absolute(target_speed_mrpm) >
        SpeedLoop_Absolute(filtered_measured_speed_mrpm))
    {
        return target_speed_mrpm;
    }
    return filtered_measured_speed_mrpm;
}

/* 根据可用电压裕量修正弱磁 d 轴电流，防止高速电压饱和。 */
static void SpeedLoop_UpdateFieldWeakeningVoltageCorrection(
    const volatile DqCurrentLoop_State_t *current_loop,
    int32_t voltage_limit_mv)
{
    float voltage_magnitude_mv_float;
    int32_t voltage_magnitude_mv;
    int32_t filter_error_mv;
    int32_t target_voltage_mv;
    int32_t control_error_mv;
    int32_t maximum_correction_mv;
    int32_t adjustment_mv;

    voltage_magnitude_mv_float = sqrtf(
        ((float)current_loop->output_d_mv *
         (float)current_loop->output_d_mv) +
        ((float)current_loop->output_q_mv *
         (float)current_loop->output_q_mv));
    voltage_magnitude_mv =
        SpeedLoop_RoundFloat(voltage_magnitude_mv_float);

    if (s_speed_loop.filtered_voltage_magnitude_mv == 0L)
    {
        s_speed_loop.filtered_voltage_magnitude_mv = voltage_magnitude_mv;
    }
    else
    {
        filter_error_mv = voltage_magnitude_mv -
                          s_speed_loop.filtered_voltage_magnitude_mv;
        s_speed_loop.filtered_voltage_magnitude_mv +=
            filter_error_mv / SPEED_LOOP_FW_VOLTAGE_FILTER_DIVISOR;
    }

    target_voltage_mv = SpeedLoop_ApplyPermille(
        voltage_limit_mv,
        s_speed_loop.field_weakening_voltage_ratio_permille);
    maximum_correction_mv = target_voltage_mv - SpeedLoop_RoundFloat(
        (float)voltage_limit_mv *
        SPEED_LOOP_FIELD_WEAKENING_MIN_VOLTAGE_RATIO);
    if (maximum_correction_mv < 0L)
    {
        maximum_correction_mv = 0L;
    }

    if ((s_speed_loop.state.field_weakening_active != 0U) ||
        (s_speed_loop.field_weakening_voltage_correction_mv > 0L))
    {
        control_error_mv = s_speed_loop.filtered_voltage_magnitude_mv -
                           target_voltage_mv;
        if (control_error_mv > SPEED_LOOP_FW_VOLTAGE_DEADBAND_MV)
        {
            adjustment_mv =
                (control_error_mv - SPEED_LOOP_FW_VOLTAGE_DEADBAND_MV +
                 SPEED_LOOP_FW_VOLTAGE_INCREASE_DIVISOR - 1L) /
                SPEED_LOOP_FW_VOLTAGE_INCREASE_DIVISOR;
            s_speed_loop.field_weakening_voltage_correction_mv +=
                adjustment_mv;
        }
        else if (control_error_mv < -SPEED_LOOP_FW_VOLTAGE_DEADBAND_MV)
        {
            adjustment_mv =
                ((-control_error_mv) - SPEED_LOOP_FW_VOLTAGE_DEADBAND_MV +
                 SPEED_LOOP_FW_VOLTAGE_RELEASE_DIVISOR - 1L) /
                SPEED_LOOP_FW_VOLTAGE_RELEASE_DIVISOR;
            s_speed_loop.field_weakening_voltage_correction_mv -=
                adjustment_mv;
        }
    }

    if (s_speed_loop.field_weakening_voltage_correction_mv < 0L)
    {
        s_speed_loop.field_weakening_voltage_correction_mv = 0L;
    }
    else if (s_speed_loop.field_weakening_voltage_correction_mv >
             maximum_correction_mv)
    {
        /* 修正器到达最低规划电压后停止积分，避免自身积分饱和。 */
        s_speed_loop.field_weakening_voltage_correction_mv =
            maximum_correction_mv;
    }
    s_speed_loop.state.field_weakening_voltage_correction_mv =
        s_speed_loop.field_weakening_voltage_correction_mv;
}

/* 将机械转速换算为电角速度，供反电势前馈计算。 */
static float SpeedLoop_CalculateElectricalAngularSpeedRadS(
    int32_t mechanical_speed_mrpm)
{
    return ((float)mechanical_speed_mrpm * SPEED_LOOP_TWO_PI *
            (float)MOTOR_ELECTRICAL_CYCLES_PER_REVOLUTION) /
           (SPEED_LOOP_MRPM_PER_RPM * SPEED_LOOP_SECONDS_PER_MINUTE);
}

/* 依据电机 R/L 与电角速度估算 d/q 电压前馈。 */
static void SpeedLoop_CalculateVoltageFeedforward(
    float id_target_a,
    float iq_target_a,
    float electrical_speed_rad_s,
    SpeedLoop_OperatingPoint_t *operating_point)
{
    /*
     * d轴弱磁有效系数只用于q轴反电动势模型；d轴交叉耦合必须使用
     * 已验证的物理Lq，二者都不参与电流PI参数或磁阻转矩计算。
     */
    operating_point->feedforward_d_mv = SpeedLoop_RoundFloat(
        ((MOTOR_PHASE_RESISTANCE_OHM * id_target_a) -
         (electrical_speed_rad_s *
          MOTOR_Q_AXIS_INDUCTANCE_H *
          iq_target_a)) * SPEED_LOOP_MV_PER_V);
    operating_point->feedforward_q_mv = SpeedLoop_RoundFloat(
        ((MOTOR_PHASE_RESISTANCE_OHM * iq_target_a) +
         (electrical_speed_rad_s *
          ((MOTOR_FW_D_AXIS_FLUX_COEFFICIENT_H * id_target_a) +
           MOTOR_FLUX_LINKAGE_WB))) * SPEED_LOOP_MV_PER_V);
}

/* 生成未启用物理模型约束时的固定 d/q 电流工作点。 */
static void SpeedLoop_CalculateFixedCurrentOperatingPoint(
    int32_t id_target_ma,
    int32_t iq_target_ma,
    int32_t measured_speed_mrpm,
    SpeedLoop_OperatingPoint_t *operating_point)
{
    float id_target_a = (float)id_target_ma / SPEED_LOOP_MA_PER_A;
    float iq_target_a = (float)iq_target_ma / SPEED_LOOP_MA_PER_A;
    float electrical_speed_rad_s =
        SpeedLoop_CalculateElectricalAngularSpeedRadS(
            measured_speed_mrpm);

    operating_point->id_target_ma = id_target_ma;
    operating_point->iq_target_ma = iq_target_ma;
    /*
     * 诊断只锁存电流目标。前馈仍按实时转速更新，否则人为引入的
     * 反电动势误差会掩盖待定位的换相/机械扰动。
     */
    SpeedLoop_CalculateVoltageFeedforward(
        id_target_a,
        iq_target_a,
        electrical_speed_rad_s,
        operating_point);
    operating_point->field_weakening_active =
        (uint8_t)((id_target_ma < 0L) ? 1U : 0U);
}

/* 在给定 d 轴电流和电压预算下计算 q 轴电流能力平方。 */
static float SpeedLoop_CalculateVoltageIqCapacitySquared(
    float id_a,
    float voltage_per_angular_speed)
{
    float effective_flux =
        (MOTOR_FW_D_AXIS_FLUX_COEFFICIENT_H * id_a) +
        MOTOR_FLUX_LINKAGE_WB;
    float remaining_squared =
        (voltage_per_angular_speed * voltage_per_angular_speed) -
        (effective_flux * effective_flux);

    if (remaining_squared <= 0.0f)
    {
        return 0.0f;
    }
    return remaining_squared /
           (MOTOR_Q_AXIS_INDUCTANCE_H *
            MOTOR_Q_AXIS_INDUCTANCE_H);
}

/* 依据电机模型、电压约束及弱磁策略求实际 d/q 运行点。 */
static void SpeedLoop_CalculatePhysicalOperatingPoint(
    int32_t requested_physical_iq_ma,
    int32_t measured_speed_mrpm,
    int32_t target_speed_mrpm,
    int8_t torque_direction_sign,
    SpeedLoop_OperatingPoint_t *operating_point)
{
    float electrical_speed_rad_s =
        SpeedLoop_CalculateElectricalAngularSpeedRadS(
            measured_speed_mrpm);
    float abs_electrical_speed_rad_s = fabsf(electrical_speed_rad_s);
    float current_limit_a =
        (float)s_speed_loop.config.current_vector_limit_ma /
        SPEED_LOOP_MA_PER_A;
    float requested_iq_a =
        (float)SpeedLoop_Absolute(requested_physical_iq_ma) /
        SPEED_LOOP_MA_PER_A;
    float id_lower_a =
        (float)SPEED_LOOP_FIELD_WEAKENING_MINIMUM_ID_MA /
        SPEED_LOOP_MA_PER_A;
    float id_target_a = 0.0f;
    float iq_target_a = requested_iq_a;

    if (requested_iq_a > current_limit_a)
    {
        requested_iq_a = current_limit_a;
        iq_target_a = current_limit_a;
    }
    if (id_lower_a < -current_limit_a)
    {
        id_lower_a = -current_limit_a;
    }

    if (abs_electrical_speed_rad_s >=
        SPEED_LOOP_MINIMUM_ANGULAR_SPEED_RAD_S)
    {
        float weakening_voltage_mv =
            (float)SpeedLoop_ApplyPermille(
                s_speed_loop.maximum_voltage_mv,
                s_speed_loop.field_weakening_voltage_ratio_permille) -
            (float)s_speed_loop.field_weakening_voltage_correction_mv;
        float minimum_weakening_voltage_mv =
            (float)s_speed_loop.maximum_voltage_mv *
            SPEED_LOOP_FIELD_WEAKENING_MIN_VOLTAGE_RATIO;
        float weakening_voltage_v;

        if (weakening_voltage_mv < minimum_weakening_voltage_mv)
        {
            weakening_voltage_mv = minimum_weakening_voltage_mv;
        }
        weakening_voltage_v =
            weakening_voltage_mv / SPEED_LOOP_MV_PER_V;
        float voltage_per_angular_speed =
            weakening_voltage_v / abs_electrical_speed_rad_s;
        float voltage_radial_squared =
            (voltage_per_angular_speed * voltage_per_angular_speed) -
            ((MOTOR_Q_AXIS_INDUCTANCE_H * requested_iq_a) *
             (MOTOR_Q_AXIS_INDUCTANCE_H * requested_iq_a));
        float required_id_a = id_lower_a;
        uint8_t requested_point_is_feasible = 0U;

        if (voltage_radial_squared >= 0.0f)
        {
            required_id_a =
                (sqrtf(voltage_radial_squared) -
                 MOTOR_FLUX_LINKAGE_WB) /
                MOTOR_FW_D_AXIS_FLUX_COEFFICIENT_H;
            if (required_id_a > 0.0f)
            {
                required_id_a = 0.0f;
            }
            if ((SpeedLoop_Absolute(target_speed_mrpm) <
                 SPEED_LOOP_FULL_TORQUE_WEAKENING_START_MRPM) &&
                (required_id_a >= id_lower_a) &&
                (((required_id_a * required_id_a) +
                  (requested_iq_a * requested_iq_a)) <=
                 (current_limit_a * current_limit_a)))
            {
                requested_point_is_feasible = 1U;
            }
        }

        if (requested_point_is_feasible != 0U)
        {
            id_target_a = required_id_a;
        }
        else
        {
            float lower_id_a = id_lower_a;
            float upper_id_a = 0.0f;
            float current_limit_squared = current_limit_a * current_limit_a;
            float lower_difference =
                SpeedLoop_CalculateVoltageIqCapacitySquared(
                    lower_id_a,
                    voltage_per_angular_speed) -
                (current_limit_squared - (lower_id_a * lower_id_a));
            float upper_difference =
                SpeedLoop_CalculateVoltageIqCapacitySquared(
                    upper_id_a,
                    voltage_per_angular_speed) - current_limit_squared;
            uint32_t iteration;

            if (lower_difference <= 0.0f)
            {
                id_target_a = lower_id_a;
            }
            else if (upper_difference >= 0.0f)
            {
                id_target_a = 0.0f;
            }
            else
            {
                /*
                 * 二分求电压椭圆与电流圆的交点，使弱磁后可用Iq最大，
                 * 循环次数固定，保证1kHz速度环执行时间可预测。
                 */
                for (iteration = 0U;
                     iteration < SPEED_LOOP_WEAKENING_SEARCH_ITERATIONS;
                     iteration++)
                {
                    float middle_id_a =
                        0.5f * (lower_id_a + upper_id_a);
                    float difference =
                        SpeedLoop_CalculateVoltageIqCapacitySquared(
                            middle_id_a,
                            voltage_per_angular_speed) -
                        (current_limit_squared -
                         (middle_id_a * middle_id_a));

                    if (difference > 0.0f)
                    {
                        lower_id_a = middle_id_a;
                    }
                    else
                    {
                        upper_id_a = middle_id_a;
                    }
                }
                id_target_a = 0.5f * (lower_id_a + upper_id_a);
            }

            {
                float current_iq_capacity_squared =
                    current_limit_squared -
                    (id_target_a * id_target_a);
                float voltage_iq_capacity_squared =
                    SpeedLoop_CalculateVoltageIqCapacitySquared(
                        id_target_a,
                        voltage_per_angular_speed);
                float iq_capacity_squared = SpeedLoop_MinFloat(
                    current_iq_capacity_squared,
                    voltage_iq_capacity_squared);

                if (iq_capacity_squared <= 0.0f)
                {
                    iq_target_a = 0.0f;
                }
                else
                {
                    iq_target_a = sqrtf(iq_capacity_squared);
                    if (iq_target_a > requested_iq_a)
                    {
                        iq_target_a = requested_iq_a;
                    }
                }
            }
        }
    }

    if (SpeedLoop_Absolute(target_speed_mrpm) >=
        SPEED_LOOP_HIGH_SPEED_PREWEAKENING_MRPM)
    {
        float weakening_voltage_mv =
            (float)SpeedLoop_ApplyPermille(
                s_speed_loop.maximum_voltage_mv,
                s_speed_loop.field_weakening_voltage_ratio_permille) -
            (float)s_speed_loop.field_weakening_voltage_correction_mv;
        float minimum_weakening_voltage_mv =
            (float)s_speed_loop.maximum_voltage_mv *
            SPEED_LOOP_FIELD_WEAKENING_MIN_VOLTAGE_RATIO;
        float voltage_per_angular_speed;
        float current_iq_capacity_squared;
        float voltage_iq_capacity_squared;
        float iq_capacity_squared;

        if (weakening_voltage_mv < minimum_weakening_voltage_mv)
        {
            weakening_voltage_mv = minimum_weakening_voltage_mv;
        }
        if (abs_electrical_speed_rad_s > 0.0f)
        {
            voltage_per_angular_speed =
                (weakening_voltage_mv / SPEED_LOOP_MV_PER_V) /
                abs_electrical_speed_rad_s;
            id_target_a = id_lower_a;
            current_iq_capacity_squared =
                (current_limit_a * current_limit_a) -
                (id_target_a * id_target_a);
            voltage_iq_capacity_squared =
                SpeedLoop_CalculateVoltageIqCapacitySquared(
                    id_target_a,
                    voltage_per_angular_speed);
            iq_capacity_squared = SpeedLoop_MinFloat(
                current_iq_capacity_squared,
                voltage_iq_capacity_squared);
            if (iq_capacity_squared <= 0.0f)
            {
                iq_target_a = 0.0f;
            }
            else if (iq_target_a > sqrtf(iq_capacity_squared))
            {
                iq_target_a = sqrtf(iq_capacity_squared);
            }
        }
    }

    if (requested_physical_iq_ma < 0L)
    {
        iq_target_a = -iq_target_a;
    }
    iq_target_a *= (float)torque_direction_sign;

    operating_point->id_target_ma = SpeedLoop_RoundFloat(
        id_target_a * SPEED_LOOP_MA_PER_A);
    operating_point->iq_target_ma = SpeedLoop_RoundFloat(
        iq_target_a * SPEED_LOOP_MA_PER_A);
    /* 浮点转整数后再次收紧电流圆，避免四舍五入造成1mA越界。 */
    while ((((int64_t)operating_point->id_target_ma *
             operating_point->id_target_ma) +
            ((int64_t)operating_point->iq_target_ma *
             operating_point->iq_target_ma)) >
           ((int64_t)s_speed_loop.config.current_vector_limit_ma *
            s_speed_loop.config.current_vector_limit_ma))
    {
        if (operating_point->iq_target_ma > 0L)
        {
            operating_point->iq_target_ma--;
        }
        else if (operating_point->iq_target_ma < 0L)
        {
            operating_point->iq_target_ma++;
        }
        else
        {
            break;
        }
    }
    /*
     * 完整dq稳态电压前馈：电阻压降、交叉耦合和反电动势统一补偿。
     * 电流PI仅负责参数误差、采样纹波与动态误差，避免深弱磁时
     * Rs*I稳态压降持续占用PI调节余量。
     */
    SpeedLoop_CalculateVoltageFeedforward(
        id_target_a,
        iq_target_a,
        electrical_speed_rad_s,
        operating_point);
    operating_point->field_weakening_active =
        (uint8_t)((operating_point->id_target_ma < 0L) ? 1U : 0U);
}

/* 根据速度 PI、母线电压、弱磁策略和电流圆计算本周期 Id/Iq 与前馈。 */
/* 统一选择固定/物理模型工作点，输出本拍 d/q 命令和诊断量。 */
static void SpeedLoop_CalculateOperatingPoint(
    int32_t requested_torque_equivalent_iq_ma,
    int32_t measured_speed_mrpm,
    int32_t target_speed_mrpm,
    int8_t torque_direction_sign,
    SpeedLoop_OperatingPoint_t *operating_point)
{
    int32_t requested_physical_iq_ma =
        requested_torque_equivalent_iq_ma;
    uint32_t pass;

    if (s_speed_loop.fixed_weakening_enabled != 0U)
    {
        float id_target_a =
            (float)s_speed_loop.fixed_weakening_id_ma /
            SPEED_LOOP_MA_PER_A;
        int32_t physical_iq_ma = SpeedLoop_CalculatePhysicalIqRequestMa(
            requested_torque_equivalent_iq_ma,
            s_speed_loop.fixed_weakening_id_ma);
        int64_t iq_capacity_squared =
            ((int64_t)s_speed_loop.config.current_vector_limit_ma *
             s_speed_loop.config.current_vector_limit_ma) -
            ((int64_t)s_speed_loop.fixed_weakening_id_ma *
             s_speed_loop.fixed_weakening_id_ma);
        int32_t iq_capacity_ma;
        float iq_target_a;
        float electrical_speed_rad_s =
            SpeedLoop_CalculateElectricalAngularSpeedRadS(
                measured_speed_mrpm);

        if (iq_capacity_squared <= 0LL)
        {
            iq_capacity_ma = 0L;
        }
        else
        {
            iq_capacity_ma = SpeedLoop_RoundFloat(
                sqrtf((float)iq_capacity_squared));
            /*
             * sqrtf结果四舍五入后可能比电流圆多1mA。固定Id路径同样必须
             * 在整数域最终收紧，否则内层会正确拒绝越界目标并停机。
             */
            while ((((int64_t)s_speed_loop.fixed_weakening_id_ma *
                     s_speed_loop.fixed_weakening_id_ma) +
                    ((int64_t)iq_capacity_ma * iq_capacity_ma)) >
                   ((int64_t)s_speed_loop.config.current_vector_limit_ma *
                    s_speed_loop.config.current_vector_limit_ma))
            {
                iq_capacity_ma--;
            }
        }
        if (physical_iq_ma > iq_capacity_ma)
        {
            physical_iq_ma = iq_capacity_ma;
        }
        else if (physical_iq_ma < -iq_capacity_ma)
        {
            physical_iq_ma = -iq_capacity_ma;
        }
        physical_iq_ma *= (int32_t)torque_direction_sign;
        iq_target_a = (float)physical_iq_ma / SPEED_LOOP_MA_PER_A;

        operating_point->id_target_ma =
            s_speed_loop.fixed_weakening_id_ma;
        operating_point->iq_target_ma = physical_iq_ma;
        SpeedLoop_CalculateVoltageFeedforward(
            id_target_a,
            iq_target_a,
            electrical_speed_rad_s,
            operating_point);
        operating_point->field_weakening_active = 1U;
        return;
    }

    /*
     * 速度PI输出按Id=0时的等效转矩电流定义。物理Ld与Lq当前均为
     * 1.85mH，因此磁阻转矩项严格为零；保留固定遍数结构，便于以后
     * 获得可信的轴向电感测量后启用凸极转矩换算。
     */
    for (pass = 0U;
         pass < SPEED_LOOP_TORQUE_LINEARIZATION_PASSES;
         pass++)
    {
        SpeedLoop_CalculatePhysicalOperatingPoint(
            requested_physical_iq_ma,
            measured_speed_mrpm,
            target_speed_mrpm,
            torque_direction_sign,
            operating_point);
        requested_physical_iq_ma =
            SpeedLoop_CalculatePhysicalIqRequestMa(
                requested_torque_equivalent_iq_ma,
                operating_point->id_target_ma);
    }
}

/* 锁存速度环故障、撤销输出并停止下游 d/q 电流环。 */
static void SpeedLoop_EnterFault(SpeedLoop_Fault_t fault)
{
    if (fault == SPEED_LOOP_FAULT_BUS_VOLTAGE)
    {
        const PowerMonitor_State_t *power = PowerMonitor_GetState();

        s_speed_loop.state.fault_bus_voltage_mv = power->bus_voltage_mv;
        s_speed_loop.state.fault_bus_voltage_raw = power->bus_voltage_raw;
        s_speed_loop.state.fault_bus_voltage_limit =
            (power->bus_voltage_mv < SPEED_LOOP_BUS_VOLTAGE_MIN_MV) ?
            1U : 2U;
    }
    s_speed_loop.state.fault = fault;
    s_speed_loop.state.running = 0U;
    s_speed_loop.state.logical_iq_target_ma = 0L;
    s_speed_loop.state.current_loop_id_target_ma = 0L;
    s_speed_loop.state.current_loop_iq_target_ma = 0L;
    s_speed_loop.state.torque_equivalent_iq_ma = 0L;
    s_speed_loop.state.filtered_bus_voltage_mv = 0UL;
    s_speed_loop.state.current_voltage_limit_mv = 0L;
    s_speed_loop.state.field_weakening_active = 0U;
    s_speed_loop.state.current_hold_active = 0U;
    DqCurrentLoop_Stop();
}

/* 清零速度 PI、弱磁、保护和诊断队列状态。 */
void SpeedLoop_Init(void)
{
    SpeedLoop_ResetTrace();
    s_speed_loop.integral_ua = 0LL;
    s_speed_loop.maximum_voltage_mv = 0L;
    s_speed_loop.filtered_bus_voltage_mv = 0L;
    s_speed_loop.filtered_weakening_speed_mrpm = 0L;
    s_speed_loop.filtered_voltage_magnitude_mv = 0L;
    s_speed_loop.field_weakening_voltage_correction_mv = 0L;
    s_speed_loop.field_weakening_voltage_ratio_permille =
        SPEED_LOOP_FIELD_WEAKENING_DEFAULT_RATIO_PERMILLE;
    s_speed_loop.fixed_weakening_id_ma = 0L;
    s_speed_loop.fixed_weakening_enabled = 0U;
    s_speed_loop.held_logical_iq_ma = 0L;
    s_speed_loop.held_id_target_ma = 0L;
    s_speed_loop.held_iq_target_ma = 0L;
    s_speed_loop.held_torque_equivalent_iq_ma = 0L;
    s_speed_loop.external_torque_override_ma = 0L;
    s_speed_loop.external_torque_override_active = 0U;
    s_speed_loop.external_braking_current_limit_ma = 0L;
    s_speed_loop.state.target_speed_mrpm = 0L;
    s_speed_loop.state.measured_speed_mrpm = 0L;
    s_speed_loop.state.speed_error_mrpm = 0L;
    s_speed_loop.state.proportional_output_ma = 0L;
    s_speed_loop.state.integral_output_ma = 0L;
    s_speed_loop.state.effective_kp_ua_per_rpm =
        SPEED_LOOP_DEFAULT_KP_UA_PER_RPM;
    s_speed_loop.state.logical_iq_target_ma = 0L;
    s_speed_loop.state.current_loop_id_target_ma = 0L;
    s_speed_loop.state.current_loop_iq_target_ma = 0L;
    s_speed_loop.state.torque_equivalent_iq_ma = 0L;
    s_speed_loop.state.fault_bus_voltage_mv = 0UL;
    s_speed_loop.state.fault_bus_voltage_raw = 0U;
    s_speed_loop.state.fault_bus_voltage_limit = 0U;
    s_speed_loop.state.elapsed_ms = 0U;
    s_speed_loop.state.fault = SPEED_LOOP_FAULT_NONE;
    s_speed_loop.state.output_saturated = 0U;
    s_speed_loop.state.field_weakening_active = 0U;
    s_speed_loop.state.field_weakening_voltage_correction_mv = 0L;
    s_speed_loop.state.field_weakening_voltage_ratio_permille =
        (uint16_t)s_speed_loop.field_weakening_voltage_ratio_permille;
    s_speed_loop.state.fixed_weakening_id_ma = 0;
    s_speed_loop.state.fixed_weakening_enabled = 0U;
    s_speed_loop.state.current_hold_enabled = 0U;
    s_speed_loop.state.current_hold_active = 0U;
    s_speed_loop.state.current_hold_completed = 0U;
    s_speed_loop.state.running = 0U;
}

/* 校验配置并启动 d/q 电流环，随后允许 1 kHz 速度 PI 输出工作点。 */
uint8_t SpeedLoop_Start(const SpeedLoop_Config_t *config,
                        int32_t target_speed_mrpm)
{
    const PowerMonitor_State_t *power;
    DqCurrentLoop_Config_t current_config;
    uint32_t pwm_voltage_limit_mv;

    if ((config == 0) || (s_speed_loop.state.running != 0U) ||
        (config->kp_ua_per_rpm < 0L) ||
        (config->ki_ua_per_rpm_s < 0L) ||
        (config->current_vector_limit_ma <= 0L) ||
        (config->maximum_speed_mrpm <= 0L) ||
        (config->maximum_run_time_ms == 0U) ||
        ((config->hold_start_ms == 0U) !=
         (config->hold_duration_ms == 0U)) ||
        ((config->hold_start_ms != 0U) &&
         ((config->hold_minimum_abs_speed_mrpm <= 0L) ||
          (config->hold_maximum_abs_speed_mrpm <=
           config->hold_minimum_abs_speed_mrpm) ||
          (config->hold_maximum_abs_speed_mrpm >
           config->maximum_speed_mrpm) ||
          (config->hold_start_ms >= config->maximum_run_time_ms) ||
          (config->hold_duration_ms >
           (config->maximum_run_time_ms - config->hold_start_ms)))) ||
        ((config->torque_direction_sign != 1) &&
         (config->torque_direction_sign != -1)) ||
        (config->direction_check_enabled > 1U) ||
        (SpeedLoop_Absolute(target_speed_mrpm) >
         config->maximum_speed_mrpm))
    {
        s_speed_loop.state.fault = SPEED_LOOP_FAULT_START_FAILED;
        return 0U;
    }

    PowerMonitor_Update();
    power = PowerMonitor_GetState();
    if ((power->bus_voltage_mv < SPEED_LOOP_BUS_VOLTAGE_MIN_MV) ||
        (power->bus_voltage_mv > SPEED_LOOP_BUS_VOLTAGE_MAX_MV))
    {
        s_speed_loop.state.fault = SPEED_LOOP_FAULT_START_FAILED;
        return 0U;
    }

    current_config.bandwidth_hz = SPEED_LOOP_CURRENT_BANDWIDTH_HZ;
    current_config.sample_frequency_hz = SPEED_LOOP_CURRENT_FREQUENCY_HZ;
    /*
     * 电压矢量同时受绝对上限和母线利用率限制，
     * 避免桥臂逼近100%占空比而丢失自举刷新时间。
     */
    pwm_voltage_limit_mv = (uint32_t)SpeedLoop_CalculateVoltageLimitMv(
        power->bus_voltage_mv);
    current_config.maximum_voltage_mv = (int32_t)pwm_voltage_limit_mv;
    current_config.maximum_target_current_ma =
        config->current_vector_limit_ma;
    current_config.overcurrent_limit_ma = SPEED_LOOP_OVERCURRENT_LIMIT_MA;
    /* 速度模式允许转子连续旋转，不启用位置偏差保护。 */
    current_config.maximum_position_deviation_count = 0L;
    current_config.bus_voltage_mv = power->bus_voltage_mv;

    s_speed_loop.config = *config;
    SpeedLoop_ResetTrace();
    s_speed_loop.integral_ua = 0LL;
    s_speed_loop.external_torque_override_ma = 0L;
    s_speed_loop.external_torque_override_active = 0U;
    s_speed_loop.external_braking_current_limit_ma = 0L;
    s_speed_loop.maximum_voltage_mv = (int32_t)pwm_voltage_limit_mv;
    s_speed_loop.filtered_bus_voltage_mv = (int32_t)power->bus_voltage_mv;
    s_speed_loop.filtered_voltage_magnitude_mv = 0L;
    s_speed_loop.field_weakening_voltage_correction_mv = 0L;
    s_speed_loop.held_logical_iq_ma = 0L;
    s_speed_loop.held_id_target_ma = 0L;
    s_speed_loop.held_iq_target_ma = 0L;
    s_speed_loop.held_torque_equivalent_iq_ma = 0L;
    s_speed_loop.state.target_speed_mrpm = target_speed_mrpm;
    s_speed_loop.state.measured_speed_mrpm =
        Encoder_GetControlSpeedMrpmFast();
    s_speed_loop.filtered_weakening_speed_mrpm =
        s_speed_loop.state.measured_speed_mrpm;
    s_speed_loop.state.speed_error_mrpm = 0L;
    s_speed_loop.state.proportional_output_ma = 0L;
    s_speed_loop.state.integral_output_ma = 0L;
    s_speed_loop.state.effective_kp_ua_per_rpm = config->kp_ua_per_rpm;
    s_speed_loop.state.logical_iq_target_ma = 0L;
    s_speed_loop.state.current_loop_id_target_ma = 0L;
    s_speed_loop.state.current_loop_iq_target_ma = 0L;
    s_speed_loop.state.torque_equivalent_iq_ma = 0L;
    s_speed_loop.state.filtered_bus_voltage_mv = power->bus_voltage_mv;
    s_speed_loop.state.fault_bus_voltage_mv = 0UL;
    s_speed_loop.state.fault_bus_voltage_raw = 0U;
    s_speed_loop.state.fault_bus_voltage_limit = 0U;
    s_speed_loop.state.current_voltage_limit_mv =
        (int32_t)pwm_voltage_limit_mv;
    s_speed_loop.state.elapsed_ms = 0U;
    s_speed_loop.state.fault = SPEED_LOOP_FAULT_NONE;
    s_speed_loop.state.output_saturated = 0U;
    s_speed_loop.state.field_weakening_active = 0U;
    s_speed_loop.state.field_weakening_voltage_correction_mv = 0L;
    s_speed_loop.state.field_weakening_voltage_ratio_permille =
        (uint16_t)s_speed_loop.field_weakening_voltage_ratio_permille;
    s_speed_loop.state.fixed_weakening_id_ma =
        (int16_t)s_speed_loop.fixed_weakening_id_ma;
    s_speed_loop.state.fixed_weakening_enabled =
        s_speed_loop.fixed_weakening_enabled;
    s_speed_loop.state.current_hold_enabled =
        (config->hold_start_ms != 0U) ? 1U : 0U;
    s_speed_loop.state.current_hold_active = 0U;
    s_speed_loop.state.current_hold_completed = 0U;

    if (DqCurrentLoop_Start(&current_config) == 0U)
    {
        s_speed_loop.state.fault = SPEED_LOOP_FAULT_START_FAILED;
        return 0U;
    }

    s_speed_loop.state.running = 1U;
    return 1U;
}

/* 原子更新速度 PI 参数；Ki 置零时同时清积分。 */
uint8_t SpeedLoop_SetTunings(int32_t kp_ua_per_rpm,
                             int32_t ki_ua_per_rpm_s)
{
    uint32_t interrupt_mask;

    if ((kp_ua_per_rpm < 0L) || (ki_ua_per_rpm_s < 0L))
    {
        return 0U;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_speed_loop.config.kp_ua_per_rpm = kp_ua_per_rpm;
    s_speed_loop.config.ki_ua_per_rpm_s = ki_ua_per_rpm_s;
    if (ki_ua_per_rpm_s == 0L)
    {
        s_speed_loop.integral_ua = 0LL;
        s_speed_loop.state.integral_output_ma = 0L;
    }
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

/* 更新机械速度目标，不直接改变位置或脉冲参考。 */
uint8_t SpeedLoop_SetTargetSpeedMrpm(int32_t target_speed_mrpm)
{
    if ((s_speed_loop.state.running == 0U) ||
        (SpeedLoop_Absolute(target_speed_mrpm) >
         s_speed_loop.config.maximum_speed_mrpm))
    {
        return 0U;
    }
    s_speed_loop.state.target_speed_mrpm = target_speed_mrpm;
    return 1U;
}

/* 清除速度 PI 历史，供外部脉冲末端模式交接减少残余转矩。 */
uint8_t SpeedLoop_ResetIntegrator(void)
{
    uint32_t interrupt_mask;

    if (s_speed_loop.state.running == 0U)
    {
        return 0U;
    }
    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_speed_loop.integral_ua = 0LL;
    s_speed_loop.state.integral_output_ma = 0L;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

/* 设置外部位置模式在再生制动方向允许的最大 Iq。 */
uint8_t SpeedLoop_SetExternalBrakingCurrentLimitMa(int32_t limit_ma)
{
    uint32_t interrupt_mask;

    if ((s_speed_loop.state.running == 0U) || (limit_ma < 0L) ||
        (limit_ma > s_speed_loop.config.current_vector_limit_ma))
    {
        return 0U;
    }
    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_speed_loop.external_braking_current_limit_ma = limit_ma;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

/* 开关位置误差直达 Iq 的试验覆盖通道，仍受工作点保护。 */
uint8_t SpeedLoop_SetExternalTorqueOverride(int32_t torque_iq_ma,
                                            uint8_t enabled)
{
    uint32_t interrupt_mask;
    int32_t current_limit_ma;
    uint8_t next_active;

    if (s_speed_loop.state.running == 0U)
    {
        return 0U;
    }

    current_limit_ma = s_speed_loop.config.current_vector_limit_ma;
    torque_iq_ma = SpeedLoop_Clamp64(torque_iq_ma,
                                     -current_limit_ma,
                                     current_limit_ma);
    next_active = (enabled != 0U) ? 1U : 0U;
    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_speed_loop.external_torque_override_ma = torque_iq_ma;
    if (s_speed_loop.external_torque_override_active != next_active)
    {
        /* 仅在进入/退出停车制动时清积分，运行态不能每250us清零。 */
        s_speed_loop.integral_ua = 0LL;
        s_speed_loop.state.integral_output_ma = 0L;
    }
    s_speed_loop.external_torque_override_active = next_active;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

/* 由 4 kHz 外部位置环立即提交已启用的直接 Iq 请求。 */
uint8_t SpeedLoop_ApplyExternalTorqueOverrideFast(void)
{
    SpeedLoop_OperatingPoint_t operating_point;
    int32_t logical_iq_ma;
    int32_t applied_torque_equivalent_iq_ma;

    if ((s_speed_loop.state.running == 0U) ||
        (s_speed_loop.external_torque_override_active == 0U))
    {
        return 0U;
    }

    /*
     * 不直接写Iq：必须经过与1kHz路径完全相同的电流圆、弱磁和电压
     * 前馈计算。这样4kHz更新只提高命令平滑度，不绕过任何安全边界。
     */
    logical_iq_ma = s_speed_loop.external_torque_override_ma;
    SpeedLoop_CalculateOperatingPoint(
        logical_iq_ma,
        s_speed_loop.state.measured_speed_mrpm,
        s_speed_loop.state.target_speed_mrpm,
        s_speed_loop.config.torque_direction_sign,
        &operating_point);
    applied_torque_equivalent_iq_ma =
        SpeedLoop_CalculateTorqueEquivalentIqMa(
            operating_point.id_target_ma,
            operating_point.iq_target_ma,
            s_speed_loop.config.torque_direction_sign);

    if (DqCurrentLoop_SetOperatingPoint(
            operating_point.id_target_ma,
            operating_point.iq_target_ma,
            operating_point.feedforward_d_mv,
            operating_point.feedforward_q_mv) == 0U)
    {
        SpeedLoop_EnterFault(SPEED_LOOP_FAULT_TARGET_UPDATE);
        return 0U;
    }

    s_speed_loop.state.logical_iq_target_ma = logical_iq_ma;
    s_speed_loop.state.current_loop_id_target_ma =
        operating_point.id_target_ma;
    s_speed_loop.state.current_loop_iq_target_ma =
        operating_point.iq_target_ma;
    s_speed_loop.state.torque_equivalent_iq_ma =
        applied_torque_equivalent_iq_ma;
    s_speed_loop.state.field_weakening_active =
        operating_point.field_weakening_active;
    return 1U;
}


/* 设置弱磁规划可占用母线电压的千分比上限。 */
uint8_t SpeedLoop_SetFieldWeakeningVoltageRatioPermille(
    uint32_t ratio_permille)
{
    if ((s_speed_loop.state.running != 0U) ||
        (ratio_permille <
         SPEED_LOOP_FIELD_WEAKENING_MINIMUM_RATIO_PERMILLE) ||
        (ratio_permille >
         SPEED_LOOP_FIELD_WEAKENING_MAXIMUM_RATIO_PERMILLE))
    {
        return 0U;
    }

    s_speed_loop.field_weakening_voltage_ratio_permille = ratio_permille;
    s_speed_loop.field_weakening_voltage_correction_mv = 0L;
    s_speed_loop.filtered_voltage_magnitude_mv = 0L;
    s_speed_loop.state.field_weakening_voltage_ratio_permille =
        (uint16_t)ratio_permille;
    s_speed_loop.state.field_weakening_voltage_correction_mv = 0L;
    return 1U;
}

/* 启用并设置固定 d 轴弱磁电流，主要用于受控测试。 */
uint8_t SpeedLoop_SetFixedWeakeningIdMa(int32_t fixed_id_ma)
{
    if ((s_speed_loop.state.running != 0U) ||
        (fixed_id_ma < SPEED_LOOP_FIXED_ID_MINIMUM_MA) ||
        (fixed_id_ma > SPEED_LOOP_FIXED_ID_MAXIMUM_MA))
    {
        return 0U;
    }

    s_speed_loop.fixed_weakening_id_ma = fixed_id_ma;
    s_speed_loop.fixed_weakening_enabled = 1U;
    s_speed_loop.field_weakening_voltage_correction_mv = 0L;
    s_speed_loop.filtered_voltage_magnitude_mv = 0L;
    s_speed_loop.state.fixed_weakening_id_ma = (int16_t)fixed_id_ma;
    s_speed_loop.state.fixed_weakening_enabled = 1U;
    s_speed_loop.state.field_weakening_voltage_correction_mv = 0L;
    return 1U;
}

/* 退出固定弱磁测试，恢复正常弱磁规划。 */
uint8_t SpeedLoop_DisableFixedWeakening(void)
{
    if (s_speed_loop.state.running != 0U)
    {
        return 0U;
    }

    s_speed_loop.fixed_weakening_id_ma = 0L;
    s_speed_loop.fixed_weakening_enabled = 0U;
    s_speed_loop.field_weakening_voltage_correction_mv = 0L;
    s_speed_loop.filtered_voltage_magnitude_mv = 0L;
    s_speed_loop.state.fixed_weakening_id_ma = 0;
    s_speed_loop.state.fixed_weakening_enabled = 0U;
    s_speed_loop.state.field_weakening_voltage_correction_mv = 0L;
    return 1U;
}

/* 终止速度模式、清空 PI 和弱磁状态，并停止 d/q 电流环。 */
void SpeedLoop_Stop(void)
{
    if (s_speed_loop.state.running != 0U)
    {
        DqCurrentLoop_Stop();
    }
    s_speed_loop.state.target_speed_mrpm = 0L;
    s_speed_loop.state.logical_iq_target_ma = 0L;
    s_speed_loop.state.current_loop_id_target_ma = 0L;
    s_speed_loop.state.current_loop_iq_target_ma = 0L;
    s_speed_loop.state.torque_equivalent_iq_ma = 0L;
    s_speed_loop.state.field_weakening_active = 0U;
    s_speed_loop.state.current_hold_active = 0U;
    s_speed_loop.external_torque_override_ma = 0L;
    s_speed_loop.external_torque_override_active = 0U;
    s_speed_loop.external_braking_current_limit_ma = 0L;
    s_speed_loop.state.running = 0U;
}

/* 1 kHz 采样速度、执行 PI/保护/弱磁并向 20 kHz 电流环更新工作点。 */
void SpeedLoop_Tick1ms(void)
{
    const PowerMonitor_State_t *power;
    const volatile DqCurrentLoop_State_t *current_loop;
    int64_t proportional_ua;
    int64_t integral_increment_ua;
    int64_t output_ua;
    int64_t limit_ua;
    int32_t logical_iq_ma;
    int32_t applied_torque_equivalent_iq_ma;
    SpeedLoop_OperatingPoint_t operating_point;
    int32_t filtered_bus_voltage_mv;
    int32_t filtered_weakening_speed_mrpm;
    int32_t weakening_planning_speed_mrpm;
    int32_t voltage_limit_mv;
    int32_t external_braking_current_limit_ma;

    if (s_speed_loop.state.running == 0U)
    {
        return;
    }

    current_loop = DqCurrentLoop_GetState();
    if (current_loop->fault != DQ_CURRENT_LOOP_FAULT_NONE)
    {
        SpeedLoop_EnterFault(SPEED_LOOP_FAULT_CURRENT_LOOP);
        return;
    }

    power = PowerMonitor_GetState();
    if ((power->bus_voltage_mv < SPEED_LOOP_BUS_VOLTAGE_MIN_MV) ||
        (power->bus_voltage_mv > SPEED_LOOP_BUS_VOLTAGE_MAX_MV))
    {
        SpeedLoop_EnterFault(SPEED_LOOP_FAULT_BUS_VOLTAGE);
        return;
    }
    filtered_bus_voltage_mv = SpeedLoop_FilterBusVoltageMv(
        power->bus_voltage_mv);
    voltage_limit_mv = SpeedLoop_CalculateVoltageLimitMv(
        (uint32_t)filtered_bus_voltage_mv);
    if (DqCurrentLoop_UpdateVoltageBudget(
            (uint32_t)filtered_bus_voltage_mv,
            voltage_limit_mv) == 0U)
    {
        SpeedLoop_EnterFault(SPEED_LOOP_FAULT_CURRENT_LOOP);
        return;
    }
    s_speed_loop.maximum_voltage_mv = voltage_limit_mv;
    s_speed_loop.state.filtered_bus_voltage_mv =
        (uint32_t)filtered_bus_voltage_mv;
    s_speed_loop.state.current_voltage_limit_mv = voltage_limit_mv;
    if ((s_speed_loop.fixed_weakening_enabled == 0U) &&
        (s_speed_loop.state.current_hold_active == 0U))
    {
        SpeedLoop_UpdateFieldWeakeningVoltageCorrection(
            current_loop,
            voltage_limit_mv);
    }
    else
    {
        /* 固定Id诊断必须隔离电压余量闭环，避免隐藏的调度状态参与。 */
        s_speed_loop.field_weakening_voltage_correction_mv = 0L;
        s_speed_loop.state.field_weakening_voltage_correction_mv = 0L;
    }

    s_speed_loop.state.elapsed_ms++;
    s_speed_loop.state.measured_speed_mrpm =
        Encoder_GetControlSpeedMrpmFast();
    filtered_weakening_speed_mrpm =
        SpeedLoop_FilterWeakeningSpeedMrpm(
            s_speed_loop.state.measured_speed_mrpm);
    weakening_planning_speed_mrpm =
        SpeedLoop_SelectWeakeningPlanningSpeedMrpm(
            filtered_weakening_speed_mrpm,
            s_speed_loop.state.target_speed_mrpm);
    s_speed_loop.state.speed_error_mrpm =
        s_speed_loop.state.target_speed_mrpm -
        s_speed_loop.state.measured_speed_mrpm;
    external_braking_current_limit_ma =
        SpeedLoop_LimitRegenerativeBrakeCurrentMa(
            s_speed_loop.external_braking_current_limit_ma,
            filtered_bus_voltage_mv);

    if ((s_speed_loop.state.current_hold_enabled != 0U) &&
        (s_speed_loop.state.current_hold_active == 0U) &&
        (s_speed_loop.state.current_hold_completed == 0U) &&
        (s_speed_loop.state.elapsed_ms >=
         s_speed_loop.config.hold_start_ms))
    {
        /*
         * 锁存上一完整速度环周期已经实际下发的物理Id/Iq。
         * 不能锁存PI的理论输出，否则弱磁转矩线性化和限幅仍会混入诊断。
         */
        s_speed_loop.held_logical_iq_ma =
            s_speed_loop.state.logical_iq_target_ma;
        s_speed_loop.held_id_target_ma =
            s_speed_loop.state.current_loop_id_target_ma;
        s_speed_loop.held_iq_target_ma =
            s_speed_loop.state.current_loop_iq_target_ma;
        s_speed_loop.held_torque_equivalent_iq_ma =
            s_speed_loop.state.torque_equivalent_iq_ma;
        s_speed_loop.state.current_hold_active = 1U;
    }

    if (s_speed_loop.state.current_hold_active != 0U)
    {
        int32_t absolute_speed_mrpm = SpeedLoop_Absolute(
            s_speed_loop.state.measured_speed_mrpm);

        if (s_speed_loop.state.elapsed_ms >=
            (s_speed_loop.config.hold_start_ms +
             s_speed_loop.config.hold_duration_ms))
        {
            s_speed_loop.state.current_hold_completed = 1U;
            SpeedLoop_Stop();
            return;
        }
        if ((absolute_speed_mrpm <
             s_speed_loop.config.hold_minimum_abs_speed_mrpm) ||
            (absolute_speed_mrpm >
             s_speed_loop.config.hold_maximum_abs_speed_mrpm))
        {
            SpeedLoop_EnterFault(SPEED_LOOP_FAULT_HOLD_SPEED_RANGE);
            return;
        }

        SpeedLoop_CalculateFixedCurrentOperatingPoint(
            s_speed_loop.held_id_target_ma,
            s_speed_loop.held_iq_target_ma,
            filtered_weakening_speed_mrpm,
            &operating_point);
        logical_iq_ma = s_speed_loop.held_logical_iq_ma;
        applied_torque_equivalent_iq_ma =
            s_speed_loop.held_torque_equivalent_iq_ma;
        s_speed_loop.state.proportional_output_ma = 0L;
        s_speed_loop.state.integral_output_ma = logical_iq_ma;
        s_speed_loop.state.effective_kp_ua_per_rpm = 0L;
        s_speed_loop.state.output_saturated = 0U;
        goto apply_operating_point;
    }

    if (s_speed_loop.external_torque_override_active != 0U)
    {
        logical_iq_ma = s_speed_loop.external_torque_override_ma;
        s_speed_loop.integral_ua = 0LL;
        s_speed_loop.state.proportional_output_ma = 0L;
        s_speed_loop.state.integral_output_ma = 0L;
        s_speed_loop.state.effective_kp_ua_per_rpm = 0L;
        s_speed_loop.state.output_saturated = 0U;
        SpeedLoop_CalculateOperatingPoint(
            logical_iq_ma,
            weakening_planning_speed_mrpm,
            0L,
            s_speed_loop.config.torque_direction_sign,
            &operating_point);
        applied_torque_equivalent_iq_ma =
            SpeedLoop_CalculateTorqueEquivalentIqMa(
                operating_point.id_target_ma,
                operating_point.iq_target_ma,
                s_speed_loop.config.torque_direction_sign);
        goto apply_operating_point;
    }

    proportional_ua =
        ((int64_t)s_speed_loop.config.kp_ua_per_rpm *
         s_speed_loop.state.speed_error_mrpm) / 1000LL;
    s_speed_loop.state.effective_kp_ua_per_rpm =
        s_speed_loop.config.kp_ua_per_rpm;
    /* 1ms离散积分：mrpm转rpm和ms转s后的综合分母为1e6。 */
    integral_increment_ua =
        ((int64_t)s_speed_loop.config.ki_ua_per_rpm_s *
         s_speed_loop.state.speed_error_mrpm) / 1000000LL;

    limit_ua = (int64_t)s_speed_loop.config.current_vector_limit_ma *
               SPEED_LOOP_UA_PER_MA;
    s_speed_loop.integral_ua += integral_increment_ua;
    output_ua = proportional_ua + s_speed_loop.integral_ua;
    if ((output_ua > limit_ua) || (output_ua < -limit_ua))
    {
        output_ua = SpeedLoop_Clamp64(output_ua, -limit_ua, limit_ua);
        if (s_speed_loop.config.ki_ua_per_rpm_s > 0L)
        {
            /* 反算积分项，使PI内部状态与限幅输出一致。 */
            s_speed_loop.integral_ua = output_ua - proportional_ua;
        }
        else
        {
            /* 纯P调试不允许限幅反算伪造积分项。 */
            s_speed_loop.integral_ua = 0LL;
        }
        s_speed_loop.state.output_saturated = 1U;
    }
    else
    {
        s_speed_loop.state.output_saturated = 0U;
    }

    logical_iq_ma = SpeedLoop_UaToMa(output_ua);
    /*
     * 外部STEP已经连续减速且实际转速仍高于速度目标时，旧积分绝不能
     * 继续输出同向驱动转矩。否则P项虽然已请求制动，积分仍会把它抵消，
     * 直到位置误差很大才突然反向。这里按实际速度与速度误差判定，
     * 不依赖外部板的脉冲数、行程或加减速时间。
     */
    if ((external_braking_current_limit_ma > 0L) &&
        (((int64_t)s_speed_loop.state.speed_error_mrpm *
          s_speed_loop.state.measured_speed_mrpm) < 0LL))
    {
        /* 清零积分后立刻保留P制动项，不能再用积分抵消它。 */
        s_speed_loop.integral_ua = 0LL;
        output_ua = proportional_ua;
        logical_iq_ma = SpeedLoop_UaToMa(output_ua);
        s_speed_loop.state.output_saturated = 1U;
    }
    if ((external_braking_current_limit_ma > 0L) &&
        (((int64_t)logical_iq_ma *
          s_speed_loop.state.measured_speed_mrpm) < 0LL) &&
        (SpeedLoop_Absolute(logical_iq_ma) >
         external_braking_current_limit_ma))
    {
        logical_iq_ma = (logical_iq_ma > 0L) ?
            external_braking_current_limit_ma :
            -external_braking_current_limit_ma;
        output_ua = (int64_t)logical_iq_ma * SPEED_LOOP_UA_PER_MA;
        s_speed_loop.integral_ua = output_ua - proportional_ua;
        s_speed_loop.state.output_saturated = 1U;
    }
    SpeedLoop_CalculateOperatingPoint(
        logical_iq_ma,
        weakening_planning_speed_mrpm,
        s_speed_loop.state.target_speed_mrpm,
        s_speed_loop.config.torque_direction_sign,
        &operating_point);
    applied_torque_equivalent_iq_ma =
        SpeedLoop_CalculateTorqueEquivalentIqMa(
            operating_point.id_target_ma,
            operating_point.iq_target_ma,
            s_speed_loop.config.torque_direction_sign);

    if ((SpeedLoop_Absolute(applied_torque_equivalent_iq_ma) +
         SPEED_LOOP_ANTIWINDUP_TOLERANCE_MA) <
        SpeedLoop_Absolute(logical_iq_ma))
    {
        int64_t applied_logical_iq_ua =
            (int64_t)applied_torque_equivalent_iq_ma *
            SPEED_LOOP_UA_PER_MA;

        /*
         * 按实际可实现的等效转矩回算，避免弱磁区积分过度退让。
         * 判定已排除mA整数化误差，此处只处理真实的电压或电流能力受限。
         */
        s_speed_loop.integral_ua =
            applied_logical_iq_ua - proportional_ua;
        logical_iq_ma = SpeedLoop_UaToMa(applied_logical_iq_ua);
        s_speed_loop.state.output_saturated = 1U;
    }
    s_speed_loop.state.proportional_output_ma =
        SpeedLoop_UaToMa(proportional_ua);
    s_speed_loop.state.integral_output_ma =
        SpeedLoop_UaToMa(s_speed_loop.integral_ua);

apply_operating_point:
    s_speed_loop.state.logical_iq_target_ma = logical_iq_ma;
    s_speed_loop.state.current_loop_id_target_ma =
        operating_point.id_target_ma;
    s_speed_loop.state.current_loop_iq_target_ma =
        operating_point.iq_target_ma;
    s_speed_loop.state.torque_equivalent_iq_ma =
        applied_torque_equivalent_iq_ma;
    s_speed_loop.state.field_weakening_active =
        operating_point.field_weakening_active;

    SpeedLoop_PushTraceSample(current_loop);

    /*
     * 方向保护比较速度指令与实际转向，不能使用瞬时Iq方向。
     * 电机超过目标速度时，速度环会正常输出反向Iq进行制动；
     * 若以Iq作为期望方向，会把正常减速误判为电机反转。
     */
    if ((s_speed_loop.config.direction_check_enabled != 0U) &&
        (s_speed_loop.state.elapsed_ms >=
         SPEED_LOOP_DIRECTION_CHECK_DELAY_MS) &&
        (((s_speed_loop.state.target_speed_mrpm > 0L) &&
          (s_speed_loop.state.measured_speed_mrpm <
           -SPEED_LOOP_DIRECTION_CHECK_SPEED_MRPM)) ||
         ((s_speed_loop.state.target_speed_mrpm < 0L) &&
          (s_speed_loop.state.measured_speed_mrpm >
           SPEED_LOOP_DIRECTION_CHECK_SPEED_MRPM))))
    {
        SpeedLoop_EnterFault(SPEED_LOOP_FAULT_DIRECTION);
        return;
    }

    if (SpeedLoop_Absolute(s_speed_loop.state.measured_speed_mrpm) >
        s_speed_loop.config.maximum_speed_mrpm)
    {
        SpeedLoop_EnterFault(SPEED_LOOP_FAULT_OVERSPEED);
        return;
    }

    /* 连续调试与长期位置保持不设运行时限，电压/电流等故障保护仍生效。 */
    if ((s_speed_loop.config.maximum_run_time_ms != UINT32_MAX) &&
        (s_speed_loop.state.elapsed_ms >=
         s_speed_loop.config.maximum_run_time_ms))
    {
        SpeedLoop_EnterFault(SPEED_LOOP_FAULT_TIMEOUT);
        return;
    }

    if (DqCurrentLoop_SetOperatingPoint(
            operating_point.id_target_ma,
            operating_point.iq_target_ma,
            operating_point.feedforward_d_mv,
            operating_point.feedforward_q_mv) == 0U)
    {
        SpeedLoop_EnterFault(SPEED_LOOP_FAULT_TARGET_UPDATE);
    }
}

/* 返回速度环和弱磁/故障状态快照。 */
const volatile SpeedLoop_State_t *SpeedLoop_GetState(void)
{
    return &s_speed_loop.state;
}

/* 从诊断环形队列取出一个样本，供主循环串口发送。 */
uint8_t SpeedLoop_PopTraceSample(SpeedLoop_TraceSample_t *sample)
{
    uint32_t interrupt_mask;
    uint16_t read_index;

    if (sample == 0)
    {
        return 0U;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    read_index = s_speed_loop.trace_read_index;
    if (read_index == s_speed_loop.trace_write_index)
    {
        if (interrupt_mask == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }

    *sample = s_speed_loop.trace_buffer[read_index];
    read_index++;
    if (read_index >= SPEED_LOOP_TRACE_BUFFER_SIZE)
    {
        read_index = 0U;
    }
    s_speed_loop.trace_read_index = read_index;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

/* 返回因消费者过慢而被诊断队列覆盖的样本数量。 */
uint32_t SpeedLoop_GetTraceDroppedCount(void)
{
    return s_speed_loop.trace_dropped_count;
}
