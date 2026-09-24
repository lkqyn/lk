#include "pulse_input.h"

#include "encoder.h"
#include "dq_transform.h"
#include "electrical_angle.h"
#include "main.h"
#include "phase_current_loop.h"
#include "power_monitor.h"
#include "position_loop.h"
#include "speed_loop.h"
#include "tim.h"

#include <limits.h>

/* 外部控制约定：800 STEP/rev，编码器4000 count/rev，即5 count/STEP。 */
#define PULSE_INPUT_COUNTS_PER_PULSE     (5L)
#define PULSE_INPUT_DIRECTION_INVERTED   (0U)
/* 外部板初始化时STEP有效且PC9为低，实测PUL_EN为低有效。 */
#define PULSE_INPUT_ENABLE_ACTIVE_LEVEL  GPIO_PIN_RESET
/*
 * 前馈退出以最近一个真实STEP周期为界。位置目标仅由真实STEP更新，
 * 因而即使前馈撤除也不会凭空增加位置命令。
 */
/*
 * 捕获周期有量化误差，控制节拍也与STEP不同相；保留30%相位余量避免
 * 连续脉冲被误判为结束。末端最多额外保持0.3T，且外部板已减速至低速。
 */
#define PULSE_INPUT_STREAM_TIMEOUT_PERMILLE    (1300U)
/* 位置误差达到一个外部STEP时，记录首次跟随滞后快照。 */
#define PULSE_INPUT_ERROR_START_THRESHOLD_COUNT (5L)
/* TIM3 prescaler=7: APB1 timer clock 84MHz / 8 = 10.5MHz. */
#define PULSE_INPUT_CAPTURE_TIMER_HZ (10500000UL)

/* 产品态外部脉冲位置环参数：最高3000rpm，沿用已验证速度PI与弱磁配置。 */
#define PULSE_INPUT_POSITION_KP_MRPM_PER_COUNT       (2400L)
#define PULSE_INPUT_POSITION_KD_MRPM_PER_RPM         (50L)
/* 外部STEP已确认进入连续减速时，提前提高阻尼但不改变PulsePos。 */
#define PULSE_INPUT_POSITION_DECEL_KD_MRPM_PER_RPM  (120L)
/* 最后STEP后提高阻尼，减少惯性越过目标后才反向修正的回摆。 */
#define PULSE_INPUT_POSITION_HOLD_KD_MRPM_PER_RPM   (250L)
#define PULSE_INPUT_MAXIMUM_SPEED_MRPM             (3000000L)
#define PULSE_INPUT_MAXIMUM_TRACKING_SPEED_MRPM    (3100000L)
#define PULSE_INPUT_MAXIMUM_ACCELERATION_MRPM_PER_S (10000000L)
#define PULSE_INPUT_POSITION_TOLERANCE_COUNT           (4L)
/* 诊断记录使用更严格的首次到达窗口，不改变位置环的4-count到位判据。 */
#define PULSE_INPUT_LAST_EDGE_MEASUREMENT_TOLERANCE_COUNT (1L)
#define PULSE_INPUT_LAST_EDGE_STABLE_SPEED_MRPM          (6000L)
#define PULSE_INPUT_LAST_EDGE_STABLE_TIME_US            (20000UL)
#define PULSE_INPUT_SETTLE_SPEED_MRPM               (30000L)
#define PULSE_INPUT_SETTLE_TIME_MS                     (50U)
#define PULSE_INPUT_MAXIMUM_POSITION_ERROR_COUNT     (1000L)
#define PULSE_INPUT_POSITION_ERROR_TIMEOUT_MS         (100U)
#define PULSE_INPUT_CURRENT_VECTOR_LIMIT_MA          (3500L)

/*
 * DMS式试验模式的安全起始电流。该电流是A/B电流矢量幅值而非单独
 * 某相的电流上限；与已验证的对齐/编码器标定相同，先使用2.0A。
 */
#define PULSE_INPUT_MICROSTEP_CURRENT_MA             (2000L)
#define PULSE_INPUT_MICROSTEP_CURRENT_LIMIT_MA       (3500L)
#define PULSE_INPUT_MICROSTEP_OVERCURRENT_LIMIT_MA   (4000L)
#define PULSE_INPUT_MICROSTEP_CURRENT_BANDWIDTH_HZ (1200.0f)
#define PULSE_INPUT_MICROSTEP_SAMPLE_FREQUENCY_HZ  (20000.0f)
#define PULSE_INPUT_MICROSTEP_PWM_UTILIZATION_PERMILLE (840UL)
#define PULSE_INPUT_MICROSTEP_MAXIMUM_VOLTAGE_MV    (20000L)

/*
 * PulsePos是STEP累计的绝对位置；CmdPos是给位置环的连续轨迹。
 * 4kHz跟随器只朝PulsePos靠近，绝不跨越该位置；频率仅限制可跟随速度。
 */
#define PULSE_INPUT_COMMAND_POSITION_Q16_SCALE       (65536LL)
#define PULSE_INPUT_COMMAND_FOLLOW_GAIN_PER_S         (4000L)
#define PULSE_INPUT_COMMAND_MAX_ACCELERATION_CPS2 (20000000L)
#define PULSE_INPUT_COMMAND_CONTROL_FREQUENCY_HZ       (4000L)

/*
 * 只在真实新STEP到来时评估减速。三个同向、每次至少15rpm的下降，
 * 才进入制动交接；这避免把输入捕获量化抖动误判为减速。
 */
#define PULSE_INPUT_DECEL_MINIMUM_SPEED_MRPM          (120000L)
#define PULSE_INPUT_DECEL_DROP_MRPM                    (30000L)
#define PULSE_INPUT_DECEL_CONFIRMATION_COUNT              (2U)

typedef struct
{
    volatile int32_t pending_pulse_delta;
    volatile PulseInput_State_t state;
    volatile uint32_t last_step_cycle_count;
    volatile uint32_t last_step_interval_cycles;
    volatile int8_t last_step_direction;
    volatile uint8_t last_step_cycle_valid;
    volatile uint32_t capture_timer_overflow_count;
    int64_t command_position_q16;
    /* 供TIM3捕获ISR读取的CmdPos诊断快照；32位读写在Cortex-M上原子。 */
    volatile int32_t command_position_count;
    int32_t command_velocity_cps;
    int32_t last_observed_raw_speed_mrpm;
    int8_t last_observed_raw_direction;
    uint8_t deceleration_confirmation_count;
    uint32_t last_edge_stable_start_cycle_count;
} PulseInput_Context_t;

static PulseInput_Context_t s_pulse_input;

static int32_t PulseInput_ClampInt64ToInt32(int64_t value)
{
    if (value > INT32_MAX) return INT32_MAX;
    if (value < INT32_MIN) return INT32_MIN;
    return (int32_t)value;
}

static int32_t PulseInput_AddSaturate(int32_t value, int32_t increment)
{
    if ((increment > 0L) && (value > (INT32_MAX - increment)))
    {
        return INT32_MAX;
    }
    if ((increment < 0L) && (value < (INT32_MIN - increment)))
    {
        return INT32_MIN;
    }
    return value + increment;
}

static int32_t PulseInput_Absolute(int32_t value)
{
    return (value == INT32_MIN) ? INT32_MAX : ((value < 0L) ? -value : value);
}

static int64_t PulseInput_AbsoluteInt64(int64_t value)
{
    return (value < 0LL) ? -value : value;
}

static int32_t PulseInput_CommandPositionQ16ToCount(int64_t position_q16)
{
    if (position_q16 >= 0LL)
    {
        return PulseInput_ClampInt64ToInt32(
            (position_q16 + (PULSE_INPUT_COMMAND_POSITION_Q16_SCALE / 2LL)) /
            PULSE_INPUT_COMMAND_POSITION_Q16_SCALE);
    }
    return PulseInput_ClampInt64ToInt32(
        -(((-position_q16) +
           (PULSE_INPUT_COMMAND_POSITION_Q16_SCALE / 2LL)) /
          PULSE_INPUT_COMMAND_POSITION_Q16_SCALE));
}

static uint8_t PulseInput_IsExternalEnableActive(void)
{
    return (HAL_GPIO_ReadPin(PUL_EN_GPIO_Port, PUL_EN_Pin) ==
            PULSE_INPUT_ENABLE_ACTIVE_LEVEL) ? 1U : 0U;
}

/*
 * 将绝对机械位置映射为固定在定子坐标中的两相正弦电流矢量。
 * 因为电角度来自PulsePos而非实时转子角度，转子偏离目标会自然形成
 * 恢复转矩；这正是微步驱动的基本锁位机制。
 */
static uint8_t PulseInput_SetMicrostepField(int32_t target_position_count)
{
    const ElectricalAngle_State_t *angle = ElectricalAngle_GetState();
    int32_t electrical_count;
    int32_t target_a_ma;
    int32_t target_beta_ma;
    int32_t target_b_ma;

    if ((ElectricalAngle_CalculateCount(target_position_count,
                                        &electrical_count) == 0U) ||
        ((angle->phase_b_axis_sign != 1) &&
         (angle->phase_b_axis_sign != -1)))
    {
        return 0U;
    }

    DqTransform_InversePark(PULSE_INPUT_MICROSTEP_CURRENT_MA,
                            0L,
                            electrical_count,
                            &target_a_ma,
                            &target_beta_ma);
    target_b_ma = (int32_t)angle->phase_b_axis_sign * target_beta_ma;

    return PhaseCurrentLoop_SetTargets(target_a_ma, target_b_ma);
}

static uint8_t PulseInput_StartMicrostepCurrentMode(
    int32_t initial_position_count)
{
    const PowerMonitor_State_t *power = PowerMonitor_GetState();
    PhaseCurrentLoop_Config_t config;
    int32_t maximum_voltage_mv;

    if (power->bus_voltage_mv == 0U)
    {
        return 0U;
    }

    maximum_voltage_mv = (int32_t)(
        (power->bus_voltage_mv *
         PULSE_INPUT_MICROSTEP_PWM_UTILIZATION_PERMILLE) / 1000UL);
    if (maximum_voltage_mv > PULSE_INPUT_MICROSTEP_MAXIMUM_VOLTAGE_MV)
    {
        maximum_voltage_mv = PULSE_INPUT_MICROSTEP_MAXIMUM_VOLTAGE_MV;
    }
    if (maximum_voltage_mv <= 0L)
    {
        return 0U;
    }

    config.bandwidth_hz = PULSE_INPUT_MICROSTEP_CURRENT_BANDWIDTH_HZ;
    config.sample_frequency_hz = PULSE_INPUT_MICROSTEP_SAMPLE_FREQUENCY_HZ;
    config.maximum_voltage_mv = maximum_voltage_mv;
    config.maximum_target_current_ma =
        PULSE_INPUT_MICROSTEP_CURRENT_LIMIT_MA;
    config.overcurrent_limit_ma =
        PULSE_INPUT_MICROSTEP_OVERCURRENT_LIMIT_MA;
    config.bus_voltage_mv = power->bus_voltage_mv;

    if (PhaseCurrentLoop_Start(&config) == 0U)
    {
        return 0U;
    }

    PulseInput_Enable();
    if (PulseInput_SetMicrostepField(initial_position_count) == 0U)
    {
        PhaseCurrentLoop_Stop();
        PulseInput_Disable();
        return 0U;
    }
    return 1U;
}

/* TIM3_CH2 captures the edge in hardware; EXTI latency does not affect CCR2. */
static uint32_t PulseInput_GetCapturedTimestamp(uint16_t capture_count)
{
    uint32_t overflow_count = s_pulse_input.capture_timer_overflow_count;

    /*
     * TIM3_IRQHandler先分发CH2捕获、后分发更新事件。若边沿已越过
     * 溢出点而更新回调尚未执行，UIF仍置位，低半区CCR2属于下一周期。
     */
    if (((TIM3->SR & TIM_SR_UIF) != 0U) && (capture_count < 0x8000U))
    {
        overflow_count++;
    }
    return (overflow_count << 16) | capture_count;
}

static uint32_t PulseInput_GetTimerTimestamp(void)
{
    uint32_t overflow_count = s_pulse_input.capture_timer_overflow_count;
    uint32_t counter = __HAL_TIM_GET_COUNTER(&htim3);

    if (((TIM3->SR & TIM_SR_UIF) != 0U) && (counter < 0x8000U))
    {
        overflow_count++;
    }
    return (overflow_count << 16) | counter;
}

void PulseInput_Init(void)
{
    s_pulse_input.pending_pulse_delta = 0L;
    s_pulse_input.last_step_cycle_count = 0U;
    s_pulse_input.last_step_interval_cycles = 0U;
    s_pulse_input.last_step_direction = 0;
    s_pulse_input.last_step_cycle_valid = 0U;
    s_pulse_input.capture_timer_overflow_count = 0U;
    s_pulse_input.command_position_q16 = 0LL;
    s_pulse_input.command_position_count = 0L;
    s_pulse_input.command_velocity_cps = 0L;
    s_pulse_input.last_observed_raw_speed_mrpm = 0L;
    s_pulse_input.last_observed_raw_direction = 0;
    s_pulse_input.deceleration_confirmation_count = 0U;
    s_pulse_input.state.reference_position_count = 0L;
    s_pulse_input.state.raw_reference_speed_mrpm = 0L;
    s_pulse_input.state.reference_speed_mrpm = 0L;
    s_pulse_input.state.pulse_stream_active = 0U;
    s_pulse_input.state.pulse_decelerating = 0U;
    s_pulse_input.state.accepted_pulse_count = 0U;
    s_pulse_input.state.ignored_pulse_count = 0U;
    s_pulse_input.state.last_step_tick_ms = 0U;
    s_pulse_input.state.last_edge_capture_cycle_count = 0U;
    s_pulse_input.state.last_edge_reference_position_count = 0L;
    s_pulse_input.state.last_edge_command_position_count = 0L;
    s_pulse_input.state.last_edge_measured_position_count = 0L;
    s_pulse_input.state.last_edge_reach_time_us = 0U;
    s_pulse_input.state.last_edge_reached = 0U;
    s_pulse_input.state.last_edge_stable_time_us = 0U;
    s_pulse_input.state.last_edge_stable = 0U;
    s_pulse_input.last_edge_stable_start_cycle_count = 0U;
    s_pulse_input.state.last_step_reference_position_count = 0L;
    s_pulse_input.state.last_step_measured_position_count = 0L;
    s_pulse_input.state.last_step_error_count = 0L;
    s_pulse_input.state.last_step_reference_speed_mrpm = 0L;
    s_pulse_input.state.last_step_measured_speed_mrpm = 0L;
    s_pulse_input.state.maximum_abs_position_error_count = 0L;
    s_pulse_input.state.maximum_abs_speed_error_mrpm = 0L;
    s_pulse_input.state.motion_sample_count = 0U;
    s_pulse_input.state.tracking_error_started = 0U;
    s_pulse_input.state.first_error_pulse_count = 0U;
    s_pulse_input.state.first_error_position_count = 0L;
    s_pulse_input.state.first_error_reference_speed_mrpm = 0L;
    s_pulse_input.state.first_error_measured_speed_mrpm = 0L;
    s_pulse_input.state.peak_error_pulse_count = 0U;
    s_pulse_input.state.peak_error_position_count = 0L;
    s_pulse_input.state.peak_error_reference_speed_mrpm = 0L;
    s_pulse_input.state.peak_error_measured_speed_mrpm = 0L;
    s_pulse_input.state.peak_speed_error_pulse_count = 0U;
    s_pulse_input.state.peak_speed_error_position_count = 0L;
    s_pulse_input.state.peak_speed_error_reference_mrpm = 0L;
    s_pulse_input.state.peak_speed_error_measured_mrpm = 0L;
    s_pulse_input.state.enabled = 0U;
    s_pulse_input.state.input_enabled = 0U;

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    (void)HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
    (void)HAL_TIM_Base_Start_IT(&htim3);
}

uint8_t PulseInput_Start(void)
{
    PositionLoop_Config_t config;
    int32_t initial_position_count;

    /* 已启动时绝不重置位置参考；串口观察不能改变运动状态。 */
    if (s_pulse_input.state.enabled != 0U)
    {
        return 1U;
    }

    initial_position_count = Encoder_GetPositionCountFast();

    if (PULSE_INPUT_CONTROL_MODE ==
        PULSE_INPUT_CONTROL_MODE_MICROSTEP_CURRENT)
    {
        return PulseInput_StartMicrostepCurrentMode(initial_position_count);
    }

    config.kp_mrpm_per_count = PULSE_INPUT_POSITION_KP_MRPM_PER_COUNT;
    config.kd_mrpm_per_rpm = PULSE_INPUT_POSITION_KD_MRPM_PER_RPM;
    config.external_deceleration_kd_mrpm_per_rpm =
        PULSE_INPUT_POSITION_DECEL_KD_MRPM_PER_RPM;
    config.external_hold_kd_mrpm_per_rpm =
        PULSE_INPUT_POSITION_HOLD_KD_MRPM_PER_RPM;
    config.maximum_speed_mrpm = PULSE_INPUT_MAXIMUM_SPEED_MRPM;
    config.maximum_tracking_speed_mrpm = PULSE_INPUT_MAXIMUM_TRACKING_SPEED_MRPM;
    config.maximum_acceleration_mrpm_per_s =
        PULSE_INPUT_MAXIMUM_ACCELERATION_MRPM_PER_S;
    config.position_tolerance_count = PULSE_INPUT_POSITION_TOLERANCE_COUNT;
    config.settle_speed_mrpm = PULSE_INPUT_SETTLE_SPEED_MRPM;
    config.settle_time_ms = PULSE_INPUT_SETTLE_TIME_MS;
    config.maximum_position_error_count = PULSE_INPUT_MAXIMUM_POSITION_ERROR_COUNT;
    config.position_error_timeout_ms = PULSE_INPUT_POSITION_ERROR_TIMEOUT_MS;
    config.speed_config.kp_ua_per_rpm = SPEED_LOOP_DEFAULT_KP_UA_PER_RPM;
    config.speed_config.ki_ua_per_rpm_s = SPEED_LOOP_DEFAULT_KI_UA_PER_RPM_S;
    config.speed_config.current_vector_limit_ma = PULSE_INPUT_CURRENT_VECTOR_LIMIT_MA;
    config.speed_config.maximum_speed_mrpm = PULSE_INPUT_MAXIMUM_TRACKING_SPEED_MRPM;
    config.speed_config.maximum_run_time_ms = UINT32_MAX;
    config.speed_config.torque_direction_sign = 1;
    config.speed_config.direction_check_enabled = 0U;
    config.speed_config.hold_start_ms = 0U;
    config.speed_config.hold_duration_ms = 0U;
    config.speed_config.hold_minimum_abs_speed_mrpm = 0L;
    config.speed_config.hold_maximum_abs_speed_mrpm = 0L;

    if ((PositionLoop_Start(&config, initial_position_count) == 0U) ||
        (PositionLoop_EnableExternalReference(initial_position_count) == 0U))
    {
        PositionLoop_Stop();
        return 0U;
    }

    PulseInput_Enable();
    return 1U;
}

/* TIM3_CH2捕获上下文：只采样DIR并累加，不调用控制器或浮点计算。 */
void PulseInput_OnStepCaptured(uint16_t capture_count)
{
    uint8_t direction_positive;
    int32_t position_increment;
    uint32_t current_cycle_count;

    if ((s_pulse_input.state.enabled == 0U) ||
        (PulseInput_IsExternalEnableActive() == 0U))
    {
        s_pulse_input.state.ignored_pulse_count++;
        return;
    }

    direction_positive =
        (HAL_GPIO_ReadPin(PUL_DIR_GPIO_Port, PUL_DIR_Pin) == GPIO_PIN_SET) ?
        1U : 0U;
    direction_positive ^= PULSE_INPUT_DIRECTION_INVERTED;
    position_increment = (direction_positive != 0U) ?
        PULSE_INPUT_COUNTS_PER_PULSE : -PULSE_INPUT_COUNTS_PER_PULSE;

    /*
     * 与原装STEP驱动器一致：边沿到来即推进位置参考，避免在1ms周期内
     * 聚合多个脉冲后再跳变。位置环读取该32位原子量时不会读到半更新值。
     */
    s_pulse_input.state.reference_position_count = PulseInput_AddSaturate(
        s_pulse_input.state.reference_position_count,
        position_increment);

    /*
     * 微步电流模式不等待1ms/4kHz任务：该STEP的目标磁场在TIM3捕获
     * 中断内立即推进，因而脉冲结束时没有CmdPos或SpeedFF残余。
     */
    if (PULSE_INPUT_CONTROL_MODE ==
        PULSE_INPUT_CONTROL_MODE_MICROSTEP_CURRENT)
    {
        (void)PulseInput_SetMicrostepField(
            s_pulse_input.state.reference_position_count);
    }
    current_cycle_count = PulseInput_GetCapturedTimestamp(capture_count);
    if (s_pulse_input.last_step_cycle_valid != 0U)
    {
        s_pulse_input.last_step_interval_cycles =
            current_cycle_count - s_pulse_input.last_step_cycle_count;
    }
    s_pulse_input.last_step_cycle_count = current_cycle_count;
    s_pulse_input.last_step_direction =
        (direction_positive != 0U) ? 1 : -1;
    s_pulse_input.last_step_cycle_valid = 1U;
    /*
     * 每个STEP边沿均锁存：CCR2时间戳对应真实边沿，TIM2直接读取AB
     * 编码器位置。停止后最后一份快照即为最后一个STEP的边沿数据。
     */
    s_pulse_input.state.last_edge_capture_cycle_count = current_cycle_count;
    s_pulse_input.state.last_edge_reference_position_count =
        s_pulse_input.state.reference_position_count;
    /* CmdPos是边沿刚到达前内部轨迹所处位置：用于拆分末端误差来源。 */
    s_pulse_input.state.last_edge_command_position_count =
        s_pulse_input.command_position_count;
    s_pulse_input.state.last_edge_measured_position_count =
        Encoder_GetPositionCountFast();
    s_pulse_input.state.last_edge_reach_time_us = 0U;
    s_pulse_input.state.last_edge_reached =
        (PulseInput_Absolute(PulseInput_ClampInt64ToInt32(
            (int64_t)s_pulse_input.state.last_edge_reference_position_count -
            s_pulse_input.state.last_edge_measured_position_count)) <=
         PULSE_INPUT_LAST_EDGE_MEASUREMENT_TOLERANCE_COUNT) ? 1U : 0U;
    s_pulse_input.state.last_edge_stable_time_us = 0U;
    s_pulse_input.state.last_edge_stable = 0U;
    s_pulse_input.last_edge_stable_start_cycle_count = 0U;
    s_pulse_input.pending_pulse_delta +=
        (direction_positive != 0U) ? 1L : -1L;
    s_pulse_input.state.accepted_pulse_count++;
}

void PulseInput_Enable(void)
{
    uint32_t interrupt_mask = __get_PRIMASK();

    __disable_irq();
    s_pulse_input.pending_pulse_delta = 0L;
    s_pulse_input.state.reference_position_count =
        Encoder_GetPositionCountFast();
    s_pulse_input.last_step_cycle_count = PulseInput_GetTimerTimestamp();
    s_pulse_input.last_step_interval_cycles = 0U;
    s_pulse_input.last_step_direction = 0;
    s_pulse_input.last_step_cycle_valid = 0U;
    s_pulse_input.command_position_q16 =
        (int64_t)s_pulse_input.state.reference_position_count *
        PULSE_INPUT_COMMAND_POSITION_Q16_SCALE;
    s_pulse_input.command_position_count =
        s_pulse_input.state.reference_position_count;
    s_pulse_input.command_velocity_cps = 0L;
    s_pulse_input.last_observed_raw_speed_mrpm = 0L;
    s_pulse_input.last_observed_raw_direction = 0;
    s_pulse_input.deceleration_confirmation_count = 0U;
    s_pulse_input.state.raw_reference_speed_mrpm = 0L;
    s_pulse_input.state.reference_speed_mrpm = 0L;
    s_pulse_input.state.pulse_stream_active = 0U;
    s_pulse_input.state.pulse_decelerating = 0U;
    s_pulse_input.state.accepted_pulse_count = 0U;
    s_pulse_input.state.ignored_pulse_count = 0U;
    s_pulse_input.state.last_step_tick_ms = HAL_GetTick();
    s_pulse_input.state.last_edge_capture_cycle_count =
        s_pulse_input.last_step_cycle_count;
    s_pulse_input.state.last_edge_reference_position_count =
        s_pulse_input.state.reference_position_count;
    s_pulse_input.state.last_edge_command_position_count =
        s_pulse_input.state.reference_position_count;
    s_pulse_input.state.last_edge_measured_position_count =
        s_pulse_input.state.reference_position_count;
    s_pulse_input.state.last_edge_reach_time_us = 0U;
    s_pulse_input.state.last_edge_reached = 1U;
    s_pulse_input.state.last_edge_stable_time_us = 0U;
    s_pulse_input.state.last_edge_stable = 0U;
    s_pulse_input.last_edge_stable_start_cycle_count = 0U;
    s_pulse_input.state.last_step_reference_position_count =
        s_pulse_input.state.reference_position_count;
    s_pulse_input.state.last_step_measured_position_count =
        s_pulse_input.state.reference_position_count;
    s_pulse_input.state.last_step_error_count = 0L;
    s_pulse_input.state.last_step_reference_speed_mrpm = 0L;
    s_pulse_input.state.last_step_measured_speed_mrpm = 0L;
    s_pulse_input.state.maximum_abs_position_error_count = 0L;
    s_pulse_input.state.maximum_abs_speed_error_mrpm = 0L;
    s_pulse_input.state.motion_sample_count = 0U;
    s_pulse_input.state.tracking_error_started = 0U;
    s_pulse_input.state.first_error_pulse_count = 0U;
    s_pulse_input.state.first_error_position_count = 0L;
    s_pulse_input.state.first_error_reference_speed_mrpm = 0L;
    s_pulse_input.state.first_error_measured_speed_mrpm = 0L;
    s_pulse_input.state.peak_error_pulse_count = 0U;
    s_pulse_input.state.peak_error_position_count = 0L;
    s_pulse_input.state.peak_error_reference_speed_mrpm = 0L;
    s_pulse_input.state.peak_error_measured_speed_mrpm = 0L;
    s_pulse_input.state.peak_speed_error_pulse_count = 0U;
    s_pulse_input.state.peak_speed_error_position_count = 0L;
    s_pulse_input.state.peak_speed_error_reference_mrpm = 0L;
    s_pulse_input.state.peak_speed_error_measured_mrpm = 0L;
    s_pulse_input.state.enabled = 1U;
    if (interrupt_mask == 0U) __enable_irq();
}

void PulseInput_Disable(void)
{
    s_pulse_input.state.enabled = 0U;
    s_pulse_input.state.raw_reference_speed_mrpm = 0L;
    s_pulse_input.state.reference_speed_mrpm = 0L;
    s_pulse_input.state.pulse_decelerating = 0U;
}

void PulseInput_OnCaptureTimerOverflow(void)
{
    s_pulse_input.capture_timer_overflow_count++;
}

static void PulseInput_UpdateCommandTrajectory250us(
    int32_t pulse_position_count,
    uint32_t step_interval_cycles,
    uint8_t step_cycle_valid,
    int32_t *command_position_count,
    int32_t *command_speed_mrpm)
{
    int64_t pulse_position_q16;
    int64_t position_error_q16;
    int64_t following_speed_cps;
    int64_t desired_velocity_cps;
    int64_t next_position_q16;
    int32_t pulse_speed_limit_cps = 0L;
    int32_t maximum_velocity_change_cps;

    pulse_position_q16 =
        (int64_t)pulse_position_count * PULSE_INPUT_COMMAND_POSITION_Q16_SCALE;
    position_error_q16 =
        pulse_position_q16 - s_pulse_input.command_position_q16;

    if (position_error_q16 == 0LL)
    {
        s_pulse_input.command_velocity_cps = 0L;
        *command_position_count = pulse_position_count;
        *command_speed_mrpm = 0L;
        return;
    }

    if ((step_cycle_valid != 0U) && (step_interval_cycles != 0U))
    {
        pulse_speed_limit_cps = PulseInput_ClampInt64ToInt32(
            ((int64_t)PULSE_INPUT_COUNTS_PER_PULSE *
             PULSE_INPUT_CAPTURE_TIMER_HZ) / step_interval_cycles);
    }

    following_speed_cps =
        (PulseInput_AbsoluteInt64(position_error_q16) *
         PULSE_INPUT_COMMAND_FOLLOW_GAIN_PER_S) /
        PULSE_INPUT_COMMAND_POSITION_Q16_SCALE;

    /*
     * 轨迹只可在已收到PulsePos以内运动；外部周期仅作为速度上限。
     * 这保留足够平滑度，避免初始化的短间隔脉冲直接把CmdPos强推到
     * 原始速度而超过实际轴可跟随的加速度。
     */
    if ((pulse_speed_limit_cps > 0L) &&
        (following_speed_cps > pulse_speed_limit_cps))
    {
        following_speed_cps = pulse_speed_limit_cps;
    }
    desired_velocity_cps = (position_error_q16 >= 0LL) ?
        following_speed_cps : -following_speed_cps;

    maximum_velocity_change_cps =
        PULSE_INPUT_COMMAND_MAX_ACCELERATION_CPS2 /
        PULSE_INPUT_COMMAND_CONTROL_FREQUENCY_HZ;
    if (s_pulse_input.command_velocity_cps < desired_velocity_cps)
    {
        s_pulse_input.command_velocity_cps = PulseInput_ClampInt64ToInt32(
            (int64_t)s_pulse_input.command_velocity_cps +
            maximum_velocity_change_cps);
        if (s_pulse_input.command_velocity_cps > desired_velocity_cps)
        {
            s_pulse_input.command_velocity_cps =
                (int32_t)desired_velocity_cps;
        }
    }
    else if (s_pulse_input.command_velocity_cps > desired_velocity_cps)
    {
        s_pulse_input.command_velocity_cps = PulseInput_ClampInt64ToInt32(
            (int64_t)s_pulse_input.command_velocity_cps -
            maximum_velocity_change_cps);
        if (s_pulse_input.command_velocity_cps < desired_velocity_cps)
        {
            s_pulse_input.command_velocity_cps =
                (int32_t)desired_velocity_cps;
        }
    }

    next_position_q16 = s_pulse_input.command_position_q16 +
        ((int64_t)s_pulse_input.command_velocity_cps *
         PULSE_INPUT_COMMAND_POSITION_Q16_SCALE) /
        PULSE_INPUT_COMMAND_CONTROL_FREQUENCY_HZ;

    /* PulsePos是硬位置边界：内部轨迹到边界即停，绝不越过去。 */
    if (((position_error_q16 >= 0LL) &&
         (next_position_q16 >= pulse_position_q16)) ||
        ((position_error_q16 < 0LL) &&
         (next_position_q16 <= pulse_position_q16)))
    {
        next_position_q16 = pulse_position_q16;
        s_pulse_input.command_velocity_cps = 0L;
    }
    s_pulse_input.command_position_q16 = next_position_q16;

    *command_position_count = PulseInput_CommandPositionQ16ToCount(
        s_pulse_input.command_position_q16);
    *command_speed_mrpm = PulseInput_ClampInt64ToInt32(
        (int64_t)s_pulse_input.command_velocity_cps * 15LL);
}

void PulseInput_Tick250us(void)
{
    uint32_t interrupt_mask;
    uint32_t step_interval_cycles;
    uint32_t last_step_cycle_count;
    uint32_t elapsed_cycles;
    uint8_t step_cycle_valid;
    uint8_t stream_active = 0U;
    uint8_t pulse_decelerating;
    int32_t pulse_position_count;
    int32_t last_edge_position_count;
    uint32_t last_edge_capture_cycle_count;
    uint8_t last_edge_reached;
    uint8_t last_edge_stable;
    int32_t command_position_count;
    int32_t command_speed_mrpm;

    if (s_pulse_input.state.enabled == 0U)
    {
        return;
    }

    /* 微步模式的磁场已由真实STEP边沿更新；不再向位置/速度环提交命令。 */
    if (PULSE_INPUT_CONTROL_MODE ==
        PULSE_INPUT_CONTROL_MODE_MICROSTEP_CURRENT)
    {
        /*
         * 仍保留最后STEP的硬件边沿到达诊断。它不参与任何控制，只用于
         * 对比“最后一颗脉冲时的位置误差”和首次进入±1 count的时间。
         */
        if (s_pulse_input.state.last_edge_reached == 0U)
        {
            uint32_t edge_cycle_count =
                s_pulse_input.state.last_edge_capture_cycle_count;
            int32_t edge_position_count =
                s_pulse_input.state.last_edge_reference_position_count;
            int32_t measured_position_count = Encoder_GetPositionCountFast();
            int32_t position_error_count = PulseInput_ClampInt64ToInt32(
                (int64_t)edge_position_count - measured_position_count);

            if (PulseInput_Absolute(position_error_count) <=
                PULSE_INPUT_LAST_EDGE_MEASUREMENT_TOLERANCE_COUNT)
            {
                uint32_t elapsed_cycles =
                    PulseInput_GetTimerTimestamp() - edge_cycle_count;
                uint32_t measurement_interrupt_mask = __get_PRIMASK();

                __disable_irq();
                if ((s_pulse_input.state.last_edge_capture_cycle_count ==
                     edge_cycle_count) &&
                    (s_pulse_input.state.last_edge_reached == 0U))
                {
                    s_pulse_input.state.last_edge_reach_time_us =
                        PulseInput_ClampInt64ToInt32(
                            ((int64_t)elapsed_cycles * 1000000LL) /
                            PULSE_INPUT_CAPTURE_TIMER_HZ);
                    s_pulse_input.state.last_edge_reached = 1U;
                }
                if (measurement_interrupt_mask == 0U) __enable_irq();
            }
        }
        return;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    step_interval_cycles = s_pulse_input.last_step_interval_cycles;
    last_step_cycle_count = s_pulse_input.last_step_cycle_count;
    step_cycle_valid = s_pulse_input.last_step_cycle_valid;
    pulse_position_count = s_pulse_input.state.reference_position_count;
    pulse_decelerating = s_pulse_input.state.pulse_decelerating;
    last_edge_capture_cycle_count =
        s_pulse_input.state.last_edge_capture_cycle_count;
    last_edge_position_count =
        s_pulse_input.state.last_edge_reference_position_count;
    last_edge_reached = s_pulse_input.state.last_edge_reached;
    last_edge_stable = s_pulse_input.state.last_edge_stable;
    if (interrupt_mask == 0U) __enable_irq();

    elapsed_cycles = PulseInput_GetTimerTimestamp() - last_step_cycle_count;

    if ((PulseInput_IsExternalEnableActive() != 0U) &&
        (step_cycle_valid != 0U) && (step_interval_cycles != 0U) &&
        ((uint64_t)elapsed_cycles * 1000ULL <=
         (uint64_t)step_interval_cycles * PULSE_INPUT_STREAM_TIMEOUT_PERMILLE))
    {
        stream_active = 1U;
    }
    s_pulse_input.state.pulse_stream_active = stream_active;

    /*
     * 每个STEP边沿已经在ISR中重置候选终点。若没有下一STEP覆盖它，此处
     * 锁存的就是最后STEP；到位时间从CCR2对应的真实边沿而非串口采样计时。
     */
    if (last_edge_reached == 0U)
    {
        int32_t measured_position_count = Encoder_GetPositionCountFast();
        int32_t position_error_count = PulseInput_ClampInt64ToInt32(
            (int64_t)last_edge_position_count - measured_position_count);

        if (PulseInput_Absolute(position_error_count) <=
            PULSE_INPUT_LAST_EDGE_MEASUREMENT_TOLERANCE_COUNT)
        {
            uint32_t current_cycle_count = PulseInput_GetTimerTimestamp();
            uint32_t elapsed_cycles =
                current_cycle_count - last_edge_capture_cycle_count;
            uint32_t measurement_interrupt_mask = __get_PRIMASK();

            __disable_irq();
            if ((s_pulse_input.state.last_edge_capture_cycle_count ==
                 last_edge_capture_cycle_count) &&
                (s_pulse_input.state.last_edge_reached == 0U))
            {
                s_pulse_input.state.last_edge_reach_time_us =
                    PulseInput_ClampInt64ToInt32(
                        ((int64_t)elapsed_cycles * 1000000LL) /
                        PULSE_INPUT_CAPTURE_TIMER_HZ);
                s_pulse_input.state.last_edge_reached = 1U;
            }
            if (measurement_interrupt_mask == 0U) __enable_irq();
        }
    }

    /*
     * 首次进入±1 count仅代表经过目标；稳定到位还必须在无STEP、低速且
     * 位置持续满足窗口20ms后才成立。新STEP会在ISR中清除本候选记录。
     */
    if (last_edge_stable == 0U)
    {
        int32_t measured_position_count = Encoder_GetPositionCountFast();
        int32_t position_error_count = PulseInput_ClampInt64ToInt32(
            (int64_t)last_edge_position_count - measured_position_count);
        int32_t measured_speed_mrpm =
            SpeedLoop_GetState()->measured_speed_mrpm;
        uint32_t current_cycle_count = PulseInput_GetTimerTimestamp();

        if ((stream_active == 0U) &&
            (PulseInput_Absolute(position_error_count) <=
             PULSE_INPUT_LAST_EDGE_MEASUREMENT_TOLERANCE_COUNT) &&
            (PulseInput_Absolute(measured_speed_mrpm) <=
             PULSE_INPUT_LAST_EDGE_STABLE_SPEED_MRPM))
        {
            if (s_pulse_input.last_edge_stable_start_cycle_count == 0U)
            {
                s_pulse_input.last_edge_stable_start_cycle_count =
                    current_cycle_count;
            }
            else if ((uint64_t)(current_cycle_count -
                                s_pulse_input.last_edge_stable_start_cycle_count) *
                     1000000ULL >=
                     (uint64_t)PULSE_INPUT_LAST_EDGE_STABLE_TIME_US *
                     PULSE_INPUT_CAPTURE_TIMER_HZ)
            {
                uint32_t measurement_interrupt_mask = __get_PRIMASK();

                __disable_irq();
                if ((s_pulse_input.state.last_edge_capture_cycle_count ==
                     last_edge_capture_cycle_count) &&
                    (s_pulse_input.state.last_edge_stable == 0U))
                {
                    s_pulse_input.state.last_edge_stable_time_us =
                        PulseInput_ClampInt64ToInt32(
                            ((int64_t)(current_cycle_count -
                                       last_edge_capture_cycle_count) *
                             1000000LL) / PULSE_INPUT_CAPTURE_TIMER_HZ);
                    s_pulse_input.state.last_edge_stable = 1U;
                }
                if (measurement_interrupt_mask == 0U) __enable_irq();
            }
        }
        else
        {
            s_pulse_input.last_edge_stable_start_cycle_count = 0U;
        }
    }

    /*
     * FOLLOW态可使用连续CmdPos/CmdSpeed平滑相邻STEP；但一旦最近一拍
     * 的预计下一拍窗口已经过去，就不能继续让内部轨迹落后于最终真实
     * PulsePos。这里仅在确认STEP流结束后交接到精确终点，绝不因为
     * “检测到减速”而提前交接，避免把匀速脉冲误判为末端。
     *
     * 交接同时清除内部轨迹速度。PositionLoop随后会看到
     * stream=0、reference_speed=0，并以最后PulsePos进入HOLD，负责
     * 清除速度PI旧积分和仅向终点方向的定位制动。
     */
    if ((stream_active == 0U) && (step_cycle_valid != 0U) &&
        (step_interval_cycles != 0U))
    {
        s_pulse_input.command_position_q16 =
            (int64_t)pulse_position_count *
            PULSE_INPUT_COMMAND_POSITION_Q16_SCALE;
        s_pulse_input.command_velocity_cps = 0L;
        command_position_count = pulse_position_count;
        command_speed_mrpm = 0L;
    }
    else
    {
        PulseInput_UpdateCommandTrajectory250us(
            pulse_position_count,
            step_interval_cycles,
            step_cycle_valid,
            &command_position_count,
            &command_speed_mrpm);
    }

    /* 仅供下一个TIM3_CH2边沿锁存诊断，不参与控制决策。 */
    s_pulse_input.command_position_count = command_position_count;

    /* 4kHz提交平滑CmdPos/CmdSpeed；PulsePos本身仍只由真实STEP推进。 */
    (void)PositionLoop_SetExternalReference(
        command_position_count,
        command_speed_mrpm,
        stream_active,
        pulse_decelerating);
}

void PulseInput_Tick1ms(void)
{
    int32_t pulse_delta;
    uint32_t interrupt_mask;
    int64_t speed_mrpm;
    uint32_t step_interval_cycles;
    int8_t step_direction;
    uint8_t step_cycle_valid;

    if (s_pulse_input.state.enabled == 0U)
    {
        return;
    }

    if (PULSE_INPUT_CONTROL_MODE ==
        PULSE_INPUT_CONTROL_MODE_MICROSTEP_CURRENT)
    {
        s_pulse_input.state.input_enabled = PulseInput_IsExternalEnableActive();
        s_pulse_input.state.raw_reference_speed_mrpm = 0L;
        s_pulse_input.state.reference_speed_mrpm = 0L;
        s_pulse_input.state.pulse_stream_active = 0U;
        s_pulse_input.state.pulse_decelerating = 0U;
        return;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    pulse_delta = s_pulse_input.pending_pulse_delta;
    s_pulse_input.pending_pulse_delta = 0L;
    step_interval_cycles = s_pulse_input.last_step_interval_cycles;
    step_direction = s_pulse_input.last_step_direction;
    step_cycle_valid = s_pulse_input.last_step_cycle_valid;
    if (interrupt_mask == 0U) __enable_irq();

    s_pulse_input.state.input_enabled = PulseInput_IsExternalEnableActive();
    if (s_pulse_input.state.input_enabled == 0U)
    {
        pulse_delta = 0L;
    }

    if ((step_cycle_valid != 0U) && (step_interval_cycles != 0U) &&
        (s_pulse_input.state.pulse_stream_active != 0U))
    {
        /*
         * STEP累计数只在真实边沿到来时推进；相邻边沿周期给出速度并在
         * 两个STEP之间保持。这样低速脉冲间隔不会制造假零速，同时绝不
         * 按速度额外增加位置命令。
         */
        speed_mrpm = ((int64_t)PULSE_INPUT_COUNTS_PER_PULSE * 60000LL *
                      (int64_t)PULSE_INPUT_CAPTURE_TIMER_HZ) /
                     ((int64_t)4000L * step_interval_cycles);
        speed_mrpm *= (int64_t)step_direction;
    }
    else
    {
        /* 超时没有新STEP：位置仍保持在最后累计值，速度前馈归零。 */
        speed_mrpm = 0LL;
    }
    s_pulse_input.state.raw_reference_speed_mrpm =
        PulseInput_ClampInt64ToInt32(speed_mrpm);
    /* 硬件捕获周期直接换算的速度作为前馈，不对位置命令做速度外推。 */
    s_pulse_input.state.reference_speed_mrpm =
        s_pulse_input.state.raw_reference_speed_mrpm;

    if ((pulse_delta != 0L) && (speed_mrpm != 0LL))
    {
        int32_t raw_speed_mrpm = s_pulse_input.state.raw_reference_speed_mrpm;
        int32_t absolute_raw_speed_mrpm = PulseInput_Absolute(raw_speed_mrpm);
        int8_t raw_direction = (raw_speed_mrpm > 0L) ? 1 : -1;

        if ((raw_direction == s_pulse_input.last_observed_raw_direction) &&
            (absolute_raw_speed_mrpm >= PULSE_INPUT_DECEL_MINIMUM_SPEED_MRPM) &&
            ((int64_t)absolute_raw_speed_mrpm +
             PULSE_INPUT_DECEL_DROP_MRPM <=
             PulseInput_Absolute(s_pulse_input.last_observed_raw_speed_mrpm)))
        {
            if (s_pulse_input.deceleration_confirmation_count <
                PULSE_INPUT_DECEL_CONFIRMATION_COUNT)
            {
                s_pulse_input.deceleration_confirmation_count++;
            }
        }
        else if ((raw_direction != s_pulse_input.last_observed_raw_direction) ||
                 ((int64_t)absolute_raw_speed_mrpm >=
                  PulseInput_Absolute(s_pulse_input.last_observed_raw_speed_mrpm) +
                  PULSE_INPUT_DECEL_DROP_MRPM))
        {
            s_pulse_input.deceleration_confirmation_count = 0U;
            s_pulse_input.state.pulse_decelerating = 0U;
        }

        s_pulse_input.last_observed_raw_speed_mrpm = raw_speed_mrpm;
        s_pulse_input.last_observed_raw_direction = raw_direction;
        if (s_pulse_input.deceleration_confirmation_count >=
            PULSE_INPUT_DECEL_CONFIRMATION_COUNT)
        {
            s_pulse_input.state.pulse_decelerating = 1U;
        }
    }
    else if (s_pulse_input.state.pulse_stream_active == 0U)
    {
        s_pulse_input.deceleration_confirmation_count = 0U;
        s_pulse_input.state.pulse_decelerating = 0U;
    }

    if ((pulse_delta != 0L) ||
        (s_pulse_input.state.reference_speed_mrpm != 0L))
    {
        int32_t measured_position_count = Encoder_GetPositionCountFast();
        int32_t position_error_count = PulseInput_ClampInt64ToInt32(
            (int64_t)s_pulse_input.state.reference_position_count -
            measured_position_count);
        int32_t speed_error_mrpm = PulseInput_ClampInt64ToInt32(
            (int64_t)s_pulse_input.state.reference_speed_mrpm -
            SpeedLoop_GetState()->measured_speed_mrpm);
        int32_t absolute_position_error =
            PulseInput_Absolute(position_error_count);
        int32_t absolute_speed_error = PulseInput_Absolute(speed_error_mrpm);

        if (absolute_position_error >
            s_pulse_input.state.maximum_abs_position_error_count)
        {
            s_pulse_input.state.maximum_abs_position_error_count =
                absolute_position_error;
            s_pulse_input.state.peak_error_pulse_count =
                s_pulse_input.state.accepted_pulse_count;
            s_pulse_input.state.peak_error_position_count = position_error_count;
            s_pulse_input.state.peak_error_reference_speed_mrpm =
                s_pulse_input.state.reference_speed_mrpm;
            s_pulse_input.state.peak_error_measured_speed_mrpm =
                SpeedLoop_GetState()->measured_speed_mrpm;
        }
        if ((s_pulse_input.state.tracking_error_started == 0U) &&
            (absolute_position_error >=
             PULSE_INPUT_ERROR_START_THRESHOLD_COUNT))
        {
            s_pulse_input.state.tracking_error_started = 1U;
            s_pulse_input.state.first_error_pulse_count =
                s_pulse_input.state.accepted_pulse_count;
            s_pulse_input.state.first_error_position_count = position_error_count;
            s_pulse_input.state.first_error_reference_speed_mrpm =
                s_pulse_input.state.reference_speed_mrpm;
            s_pulse_input.state.first_error_measured_speed_mrpm =
                SpeedLoop_GetState()->measured_speed_mrpm;
        }
        if (absolute_speed_error >
            s_pulse_input.state.maximum_abs_speed_error_mrpm)
        {
            s_pulse_input.state.maximum_abs_speed_error_mrpm =
                absolute_speed_error;
            s_pulse_input.state.peak_speed_error_pulse_count =
                s_pulse_input.state.accepted_pulse_count;
            s_pulse_input.state.peak_speed_error_position_count =
                position_error_count;
            s_pulse_input.state.peak_speed_error_reference_mrpm =
                s_pulse_input.state.reference_speed_mrpm;
            s_pulse_input.state.peak_speed_error_measured_mrpm =
                SpeedLoop_GetState()->measured_speed_mrpm;
        }
        s_pulse_input.state.motion_sample_count++;
    }

    /*
     * 仅在本1ms窗口实际收到STEP时更新快照。脉冲停止后该记录保持，
     * 可直接读出最后一批脉冲对应的跟随误差和制动前速度。
     */
    if (pulse_delta != 0L)
    {
        s_pulse_input.state.last_step_tick_ms = HAL_GetTick();
        s_pulse_input.state.last_step_reference_position_count =
            s_pulse_input.state.reference_position_count;
        s_pulse_input.state.last_step_measured_position_count =
            Encoder_GetPositionCountFast();
        s_pulse_input.state.last_step_error_count =
            PulseInput_ClampInt64ToInt32(
                (int64_t)s_pulse_input.state.last_step_reference_position_count -
                s_pulse_input.state.last_step_measured_position_count);
        s_pulse_input.state.last_step_reference_speed_mrpm =
            s_pulse_input.state.reference_speed_mrpm;
        s_pulse_input.state.last_step_measured_speed_mrpm =
            SpeedLoop_GetState()->measured_speed_mrpm;
    }

}

const volatile PulseInput_State_t *PulseInput_GetState(void)
{
    return &s_pulse_input.state;
}
