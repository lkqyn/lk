#include "electrical_angle.h"

#include "motor_parameters.h"

#if ((MOTOR_ENCODER_COUNTS_PER_REVOLUTION % \
      MOTOR_ELECTRICAL_CYCLES_PER_REVOLUTION) != 0)
#error "编码器每圈计数必须能被电机每圈电周期数整除"
#endif

#define ELECTRICAL_ANGLE_FULL_CIRCLE_MDEG  (360000L)
#define ELECTRICAL_ANGLE_PHASE_INDEXES      (1024LL)
#define ELECTRICAL_ANGLE_CALIBRATION_SECTORS (200L)
#define ELECTRICAL_ANGLE_CALIBRATION_POINTS  (201U)
#define ELECTRICAL_ANGLE_MECHANICAL_CIRCLE_Q1 \
    (2L * MOTOR_ENCODER_COUNTS_PER_REVOLUTION)
#define ELECTRICAL_ANGLE_IDEAL_SECTOR_Q1 \
    (ELECTRICAL_ANGLE_MECHANICAL_CIRCLE_Q1 / \
     ELECTRICAL_ANGLE_CALIBRATION_SECTORS)
#define ELECTRICAL_ANGLE_PHASE_PER_SECTOR    (256L)
#define ELECTRICAL_ANGLE_MINIMUM_SECTOR_Q1   (24L)
#define ELECTRICAL_ANGLE_MAXIMUM_SECTOR_Q1   (56L)
#define ELECTRICAL_ANGLE_MAXIMUM_CUMULATIVE_ERROR_Q1 (20L)
#define ELECTRICAL_ANGLE_MAXIMUM_CLOSURE_ERROR_Q1    (160L)
#define ELECTRICAL_ANGLE_OBSERVER_Q_BITS    (16U)
#define ELECTRICAL_ANGLE_OBSERVER_Q_SCALE   (1LL << ELECTRICAL_ANGLE_OBSERVER_Q_BITS)
#define ELECTRICAL_ANGLE_OBSERVER_GAIN_BITS (20U)
#define ELECTRICAL_ANGLE_OBSERVER_DEFAULT_BANDWIDTH_HZ (800U)
#define ELECTRICAL_ANGLE_OBSERVER_MINIMUM_BANDWIDTH_HZ (100U)
#define ELECTRICAL_ANGLE_OBSERVER_MAXIMUM_BANDWIDTH_HZ (1500U)
/*
 * 20kHz离散二阶位置观测器，默认固有频率800Hz、阻尼比约0.707。
 * 带宽可在停机时调整，控制中断只读取预先计算好的定点系数。
 */
/*
 * 20kHz采样、阻尼比0.707时的定点系数：
 * alpha_q20约等于465.87*带宽，beta_q20约等于0.10349*带宽平方。
 */
#define ELECTRICAL_ANGLE_OBSERVER_ALPHA_Q20_PER_HZ (466UL)
#define ELECTRICAL_ANGLE_OBSERVER_BETA_Q20_NUMERATOR (106ULL)
#define ELECTRICAL_ANGLE_OBSERVER_BETA_Q20_DENOMINATOR (1024ULL)
/* 观测误差超过16个机械计数说明状态失配，立即重同步以保证安全。 */
#define ELECTRICAL_ANGLE_OBSERVER_RESYNC_ERROR_Q16 \
    (16LL * ELECTRICAL_ANGLE_OBSERVER_Q_SCALE)

typedef struct
{
    int64_t position_q16;
    int64_t velocity_q16_per_tick;
} ElectricalAngle_ObserverContext_t;

static ElectricalAngle_State_t s_electrical_angle;
static ElectricalAngle_ObserverContext_t s_angle_observer;
static uint16_t s_calibration_boundary_q1[
    ELECTRICAL_ANGLE_CALIBRATION_POINTS];

static int32_t ElectricalAngle_PositiveModulo(int32_t value, int32_t modulus)
{
    int32_t result = value % modulus;

    if (result < 0L)
    {
        result += modulus;
    }
    return result;
}

static int64_t ElectricalAngle_PositiveModulo64(int64_t value,
                                                 int64_t modulus)
{
    int64_t result = value % modulus;

    if (result < 0LL)
    {
        result += modulus;
    }
    return result;
}

static int32_t ElectricalAngle_WrapPhaseIndex(int32_t phase_index)
{
    int32_t wrapped = phase_index % (int32_t)ELECTRICAL_ANGLE_PHASE_INDEXES;

    if (wrapped < 0L)
    {
        wrapped += (int32_t)ELECTRICAL_ANGLE_PHASE_INDEXES;
    }
    return wrapped;
}

static int32_t ElectricalAngle_WrapPhaseError(int32_t phase_error)
{
    int32_t wrapped = ElectricalAngle_WrapPhaseIndex(phase_error);

    if (wrapped >= ((int32_t)ELECTRICAL_ANGLE_PHASE_INDEXES / 2L))
    {
        wrapped -= (int32_t)ELECTRICAL_ANGLE_PHASE_INDEXES;
    }
    return wrapped;
}

static int32_t ElectricalAngle_PositionQ16ToPhaseIndex(
    int64_t encoder_position_q16)
{
    const int64_t cycle_q16 =
        (int64_t)MOTOR_ENCODER_COUNTS_PER_ELECTRICAL_CYCLE *
        ELECTRICAL_ANGLE_OBSERVER_Q_SCALE;
    int64_t relative_q16 = ElectricalAngle_PositiveModulo64(
        encoder_position_q16 -
            ((int64_t)s_electrical_angle.alignment_offset_count *
             ELECTRICAL_ANGLE_OBSERVER_Q_SCALE),
        cycle_q16);
    int64_t phase_index =
        ((relative_q16 * ELECTRICAL_ANGLE_PHASE_INDEXES) +
         (cycle_q16 / 2LL)) /
        cycle_q16;

    return (int32_t)(phase_index & (ELECTRICAL_ANGLE_PHASE_INDEXES - 1LL));
}

void ElectricalAngle_Init(void)
{
    s_electrical_angle.alignment_offset_count = 0L;
    s_electrical_angle.offset_in_electrical_cycle = 0L;
    s_electrical_angle.electrical_count = 0L;
    s_electrical_angle.electrical_angle_mdeg = 0L;
    s_electrical_angle.raw_phase_index = 0L;
    s_electrical_angle.corrected_phase_index = 0L;
    s_electrical_angle.calibration_correction_phase_index = 0L;
    s_electrical_angle.calibration_anchor_q1 = 0L;
    s_electrical_angle.observer_phase_index = 0L;
    s_electrical_angle.observer_error_q16 = 0L;
    s_electrical_angle.observer_velocity_q16_per_tick = 0L;
    s_electrical_angle.observer_resync_count = 0U;
    s_electrical_angle.observer_bandwidth_hz =
        ELECTRICAL_ANGLE_OBSERVER_DEFAULT_BANDWIDTH_HZ;
    s_electrical_angle.observer_alpha_q20 =
        ELECTRICAL_ANGLE_OBSERVER_ALPHA_Q20_PER_HZ *
        ELECTRICAL_ANGLE_OBSERVER_DEFAULT_BANDWIDTH_HZ;
    s_electrical_angle.observer_beta_q20 =
        (uint32_t)((ELECTRICAL_ANGLE_OBSERVER_BETA_Q20_NUMERATOR *
                    ELECTRICAL_ANGLE_OBSERVER_DEFAULT_BANDWIDTH_HZ *
                    ELECTRICAL_ANGLE_OBSERVER_DEFAULT_BANDWIDTH_HZ) /
                   ELECTRICAL_ANGLE_OBSERVER_BETA_Q20_DENOMINATOR);
    /* 默认走连续预测+标定校正，避免高速整数编码器角度阶梯进入电流环。 */
    s_electrical_angle.observer_mode =
        ELECTRICAL_ANGLE_OBSERVER_CALIBRATED_PREDICTIVE;
    s_electrical_angle.phase_b_axis_sign = 0;
    s_electrical_angle.observer_initialized = 0U;
    s_electrical_angle.calibration_valid = 0U;
    s_electrical_angle.calibration_enabled = 0U;
    s_electrical_angle.aligned = 0U;
    s_angle_observer.position_q16 = 0LL;
    s_angle_observer.velocity_q16_per_tick = 0LL;
}

void ElectricalAngle_Invalidate(void)
{
    s_electrical_angle.aligned = 0U;
    s_electrical_angle.phase_b_axis_sign = 0;
    s_electrical_angle.electrical_count = 0L;
    s_electrical_angle.electrical_angle_mdeg = 0L;
    s_electrical_angle.raw_phase_index = 0L;
    s_electrical_angle.corrected_phase_index = 0L;
    s_electrical_angle.calibration_correction_phase_index = 0L;
    s_electrical_angle.observer_phase_index = 0L;
    s_electrical_angle.observer_error_q16 = 0L;
    s_electrical_angle.observer_velocity_q16_per_tick = 0L;
    s_electrical_angle.observer_initialized = 0U;
}

void ElectricalAngle_SetAlignment(int32_t encoder_position_count,
                                  int8_t phase_b_axis_sign)
{
    if ((phase_b_axis_sign != 1) && (phase_b_axis_sign != -1))
    {
        ElectricalAngle_Invalidate();
        return;
    }

    s_electrical_angle.alignment_offset_count = encoder_position_count;
    s_electrical_angle.offset_in_electrical_cycle =
        ElectricalAngle_PositiveModulo(
            encoder_position_count,
            (int32_t)MOTOR_ENCODER_COUNTS_PER_ELECTRICAL_CYCLE);
    s_electrical_angle.electrical_count = 0L;
    s_electrical_angle.electrical_angle_mdeg = 0L;
    s_electrical_angle.phase_b_axis_sign = phase_b_axis_sign;
    s_electrical_angle.aligned = 1U;
    ElectricalAngle_ResetObserver(encoder_position_count);
}

uint8_t ElectricalAngle_CalculateCount(int32_t encoder_position_count,
                                       int32_t *electrical_count)
{
    int32_t relative_in_cycle;

    if ((electrical_count == 0) || (s_electrical_angle.aligned == 0U) ||
        ((s_electrical_angle.phase_b_axis_sign != 1) &&
         (s_electrical_angle.phase_b_axis_sign != -1)))
    {
        return 0U;
    }

    relative_in_cycle = ElectricalAngle_PositiveModulo(
        (int32_t)((uint32_t)encoder_position_count -
                  (uint32_t)s_electrical_angle.alignment_offset_count),
        (int32_t)MOTOR_ENCODER_COUNTS_PER_ELECTRICAL_CYCLE);
    *electrical_count = relative_in_cycle *
        (int32_t)MOTOR_ELECTRICAL_CYCLES_PER_REVOLUTION;
    return 1U;
}

uint8_t ElectricalAngle_CalculateControlPhaseIndex(
    int32_t encoder_position_count,
    int32_t *phase_index)
{
    int32_t electrical_count;
    int32_t raw_phase_index;
    int32_t relative_position_q1;
    int32_t sector;
    int32_t lower_q1;
    int32_t upper_q1;
    int32_t fraction_numerator_q1;
    int32_t corrected_phase_index;
    int64_t interpolated_phase;

    if ((phase_index == 0) ||
        (ElectricalAngle_CalculateCount(encoder_position_count,
                                        &electrical_count) == 0U))
    {
        return 0U;
    }

    raw_phase_index = ElectricalAngle_WrapPhaseIndex(
        (int32_t)((((int64_t)electrical_count *
                    ELECTRICAL_ANGLE_PHASE_INDEXES) +
                   (MOTOR_ENCODER_COUNTS_PER_REVOLUTION / 2L)) /
                  MOTOR_ENCODER_COUNTS_PER_REVOLUTION));
    s_electrical_angle.raw_phase_index = raw_phase_index;

    if ((s_electrical_angle.calibration_valid == 0U) ||
        (s_electrical_angle.calibration_enabled == 0U))
    {
        s_electrical_angle.corrected_phase_index = raw_phase_index;
        s_electrical_angle.calibration_correction_phase_index = 0L;
        *phase_index = raw_phase_index;
        return 1U;
    }

    relative_position_q1 = (int32_t)ElectricalAngle_PositiveModulo64(
        ((int64_t)encoder_position_count * 2LL) -
            s_electrical_angle.calibration_anchor_q1,
        ELECTRICAL_ANGLE_MECHANICAL_CIRCLE_Q1);

    /*
     * 理想扇区宽度为40个q1计数，先O(1)估计扇区；
     * 实测非线性远小于一整步，因此最多只需修正到相邻扇区。
     */
    sector = relative_position_q1 / ELECTRICAL_ANGLE_IDEAL_SECTOR_Q1;
    if (sector >= ELECTRICAL_ANGLE_CALIBRATION_SECTORS)
    {
        sector = ELECTRICAL_ANGLE_CALIBRATION_SECTORS - 1L;
    }
    while ((sector > 0L) &&
           (relative_position_q1 < s_calibration_boundary_q1[sector]))
    {
        sector--;
    }
    while ((sector < (ELECTRICAL_ANGLE_CALIBRATION_SECTORS - 1L)) &&
           (relative_position_q1 >=
            s_calibration_boundary_q1[sector + 1L]))
    {
        sector++;
    }

    lower_q1 = s_calibration_boundary_q1[sector];
    upper_q1 = s_calibration_boundary_q1[sector + 1L];
    fraction_numerator_q1 = relative_position_q1 - lower_q1;
    interpolated_phase =
        ((int64_t)sector * ELECTRICAL_ANGLE_PHASE_PER_SECTOR) +
        ((((int64_t)fraction_numerator_q1 *
           ELECTRICAL_ANGLE_PHASE_PER_SECTOR) +
          ((upper_q1 - lower_q1) / 2L)) /
         (upper_q1 - lower_q1));
    corrected_phase_index = ElectricalAngle_WrapPhaseIndex(
        (int32_t)interpolated_phase);

    s_electrical_angle.corrected_phase_index = corrected_phase_index;
    s_electrical_angle.calibration_correction_phase_index =
        ElectricalAngle_WrapPhaseError(
            corrected_phase_index - raw_phase_index);
    *phase_index = corrected_phase_index;
    return 1U;
}

uint8_t ElectricalAngle_SetNonlinearityCalibration(
    const int32_t *midpoint_q1,
    uint16_t point_count)
{
    int32_t anchor_q1;
    uint16_t index;

    if ((midpoint_q1 == 0) ||
        (point_count != ELECTRICAL_ANGLE_CALIBRATION_POINTS))
    {
        return 0U;
    }

    anchor_q1 = midpoint_q1[0U];
    s_calibration_boundary_q1[0U] = 0U;
    for (index = 1U;
         index < (ELECTRICAL_ANGLE_CALIBRATION_POINTS - 1U);
         index++)
    {
        int32_t relative_q1 = midpoint_q1[index] - anchor_q1;
        int32_t interval_q1 = relative_q1 -
            (int32_t)s_calibration_boundary_q1[index - 1U];
        int32_t ideal_relative_q1 =
            (int32_t)index * ELECTRICAL_ANGLE_IDEAL_SECTOR_Q1;
        int32_t cumulative_error_q1 =
            relative_q1 - ideal_relative_q1;

        if ((relative_q1 <= 0L) ||
            (relative_q1 >= ELECTRICAL_ANGLE_MECHANICAL_CIRCLE_Q1) ||
            (interval_q1 < ELECTRICAL_ANGLE_MINIMUM_SECTOR_Q1) ||
            (interval_q1 > ELECTRICAL_ANGLE_MAXIMUM_SECTOR_Q1) ||
            (cumulative_error_q1 <
             -ELECTRICAL_ANGLE_MAXIMUM_CUMULATIVE_ERROR_Q1) ||
            (cumulative_error_q1 >
             ELECTRICAL_ANGLE_MAXIMUM_CUMULATIVE_ERROR_Q1))
        {
            ElectricalAngle_InvalidateNonlinearityCalibration();
            return 0U;
        }
        s_calibration_boundary_q1[index] = (uint16_t)relative_q1;
    }

    if ((((midpoint_q1[ELECTRICAL_ANGLE_CALIBRATION_POINTS - 1U] -
           anchor_q1) - ELECTRICAL_ANGLE_MECHANICAL_CIRCLE_Q1) <
         -ELECTRICAL_ANGLE_MAXIMUM_CLOSURE_ERROR_Q1) ||
        (((midpoint_q1[ELECTRICAL_ANGLE_CALIBRATION_POINTS - 1U] -
           anchor_q1) - ELECTRICAL_ANGLE_MECHANICAL_CIRCLE_Q1) >
         ELECTRICAL_ANGLE_MAXIMUM_CLOSURE_ERROR_Q1) ||
        ((ELECTRICAL_ANGLE_MECHANICAL_CIRCLE_Q1 -
          (int32_t)s_calibration_boundary_q1[
              ELECTRICAL_ANGLE_CALIBRATION_POINTS - 2U]) <
         ELECTRICAL_ANGLE_MINIMUM_SECTOR_Q1) ||
        ((ELECTRICAL_ANGLE_MECHANICAL_CIRCLE_Q1 -
          (int32_t)s_calibration_boundary_q1[
              ELECTRICAL_ANGLE_CALIBRATION_POINTS - 2U]) >
         ELECTRICAL_ANGLE_MAXIMUM_SECTOR_Q1))
    {
        ElectricalAngle_InvalidateNonlinearityCalibration();
        return 0U;
    }

    /* 强制环形表在4000计数处闭合，避免跨圈角度跳变。 */
    s_calibration_boundary_q1[
        ELECTRICAL_ANGLE_CALIBRATION_POINTS - 1U] =
        (uint16_t)ELECTRICAL_ANGLE_MECHANICAL_CIRCLE_Q1;
    s_electrical_angle.calibration_anchor_q1 = anchor_q1;
    s_electrical_angle.calibration_valid = 1U;
    s_electrical_angle.calibration_enabled = 0U;
    s_electrical_angle.calibration_correction_phase_index = 0L;
    return 1U;
}

uint8_t ElectricalAngle_EnableNonlinearityCalibration(uint8_t enable)
{
    if (enable == 0U)
    {
        s_electrical_angle.calibration_enabled = 0U;
        s_electrical_angle.calibration_correction_phase_index = 0L;
        return 1U;
    }

    if ((s_electrical_angle.calibration_valid == 0U) ||
        (s_electrical_angle.observer_mode == ELECTRICAL_ANGLE_OBSERVER_ON))
    {
        return 0U;
    }
    s_electrical_angle.calibration_enabled = 1U;
    return 1U;
}

void ElectricalAngle_InvalidateNonlinearityCalibration(void)
{
    s_electrical_angle.calibration_valid = 0U;
    s_electrical_angle.calibration_enabled = 0U;
    s_electrical_angle.calibration_anchor_q1 = 0L;
    s_electrical_angle.calibration_correction_phase_index = 0L;
}

void ElectricalAngle_ResetObserver(int32_t encoder_position_count)
{
    s_angle_observer.position_q16 =
        (int64_t)encoder_position_count *
        ELECTRICAL_ANGLE_OBSERVER_Q_SCALE;
    s_angle_observer.velocity_q16_per_tick = 0LL;
    s_electrical_angle.raw_phase_index =
        ElectricalAngle_PositionQ16ToPhaseIndex(
            s_angle_observer.position_q16);
    s_electrical_angle.observer_phase_index =
        s_electrical_angle.raw_phase_index;
    s_electrical_angle.observer_error_q16 = 0L;
    s_electrical_angle.observer_velocity_q16_per_tick = 0L;
    s_electrical_angle.observer_initialized = 1U;
}

uint8_t ElectricalAngle_UpdateObserver(int32_t encoder_position_count,
                                       int32_t *raw_phase_index,
                                       int32_t *observer_phase_index)
{
    int64_t measured_position_q16;
    int64_t predicted_position_q16;
    int64_t error_q16;

    if ((raw_phase_index == 0) || (observer_phase_index == 0) ||
        (s_electrical_angle.aligned == 0U))
    {
        return 0U;
    }

    measured_position_q16 =
        (int64_t)encoder_position_count *
        ELECTRICAL_ANGLE_OBSERVER_Q_SCALE;
    if (s_electrical_angle.observer_initialized == 0U)
    {
        ElectricalAngle_ResetObserver(encoder_position_count);
    }

    predicted_position_q16 =
        s_angle_observer.position_q16 +
        s_angle_observer.velocity_q16_per_tick;
    error_q16 = measured_position_q16 - predicted_position_q16;

    if ((error_q16 > ELECTRICAL_ANGLE_OBSERVER_RESYNC_ERROR_Q16) ||
        (error_q16 < -ELECTRICAL_ANGLE_OBSERVER_RESYNC_ERROR_Q16))
    {
        ElectricalAngle_ResetObserver(encoder_position_count);
        s_electrical_angle.observer_resync_count++;
        error_q16 = 0LL;
    }
    else
    {
        s_angle_observer.position_q16 =
            predicted_position_q16 +
            (((int64_t)s_electrical_angle.observer_alpha_q20 * error_q16) >>
             ELECTRICAL_ANGLE_OBSERVER_GAIN_BITS);
        s_angle_observer.velocity_q16_per_tick +=
            ((int64_t)s_electrical_angle.observer_beta_q20 * error_q16) >>
            ELECTRICAL_ANGLE_OBSERVER_GAIN_BITS;
    }

    s_electrical_angle.raw_phase_index =
        ElectricalAngle_PositionQ16ToPhaseIndex(measured_position_q16);
    s_electrical_angle.observer_phase_index =
        ElectricalAngle_PositionQ16ToPhaseIndex(
            s_angle_observer.position_q16);
    s_electrical_angle.observer_error_q16 = (int32_t)error_q16;
    s_electrical_angle.observer_velocity_q16_per_tick =
        (int32_t)s_angle_observer.velocity_q16_per_tick;
    *raw_phase_index = s_electrical_angle.raw_phase_index;
    *observer_phase_index = s_electrical_angle.observer_phase_index;
    return 1U;
}

uint8_t ElectricalAngle_SetObserverMode(
    ElectricalAngle_ObserverMode_t mode)
{
    if ((mode != ELECTRICAL_ANGLE_OBSERVER_OFF) &&
        (mode != ELECTRICAL_ANGLE_OBSERVER_SHADOW) &&
        (mode != ELECTRICAL_ANGLE_OBSERVER_ON) &&
        (mode != ELECTRICAL_ANGLE_OBSERVER_CALIBRATED_PREDICTIVE))
    {
        return 0U;
    }

    if ((mode == ELECTRICAL_ANGLE_OBSERVER_ON) &&
        (s_electrical_angle.calibration_enabled != 0U))
    {
        return 0U;
    }

    s_electrical_angle.observer_mode = mode;
    return 1U;
}

uint8_t ElectricalAngle_SetObserverBandwidthHz(uint32_t bandwidth_hz)
{
    uint64_t bandwidth_squared;

    if ((bandwidth_hz < ELECTRICAL_ANGLE_OBSERVER_MINIMUM_BANDWIDTH_HZ) ||
        (bandwidth_hz > ELECTRICAL_ANGLE_OBSERVER_MAXIMUM_BANDWIDTH_HZ))
    {
        return 0U;
    }

    bandwidth_squared = (uint64_t)bandwidth_hz * bandwidth_hz;
    s_electrical_angle.observer_bandwidth_hz = bandwidth_hz;
    s_electrical_angle.observer_alpha_q20 =
        ELECTRICAL_ANGLE_OBSERVER_ALPHA_Q20_PER_HZ * bandwidth_hz;
    s_electrical_angle.observer_beta_q20 =
        (uint32_t)((ELECTRICAL_ANGLE_OBSERVER_BETA_Q20_NUMERATOR *
                    bandwidth_squared) /
                   ELECTRICAL_ANGLE_OBSERVER_BETA_Q20_DENOMINATOR);
    return 1U;
}

void ElectricalAngle_Update(int32_t encoder_position_count)
{
    int32_t electrical_count;

    if (ElectricalAngle_CalculateCount(encoder_position_count,
                                       &electrical_count) == 0U)
    {
        return;
    }

    s_electrical_angle.electrical_count = electrical_count;
    s_electrical_angle.electrical_angle_mdeg =
        (int32_t)(((int64_t)s_electrical_angle.electrical_count *
                   ELECTRICAL_ANGLE_FULL_CIRCLE_MDEG) /
                  (int64_t)MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
}

const ElectricalAngle_State_t *ElectricalAngle_GetState(void)
{
    return &s_electrical_angle;
}
