#include "position_loop.h"

#include "encoder.h"
#include "stm32f4xx_hal.h"
#include "trajectory_planner.h"

/*
 * Keil 工程当前采用显式文件清单，工程文件编码不支持自动安全更新。
 * 轨迹模块在此处以单一编译单元纳入，接口仍保持在trajectory_planner.h；
 * 后续统一整理工程文件时，应将trajectory_planner.c改为独立编译单元。
 */
#include "trajectory_planner.c"

#include <limits.h>

/*
 * 外部STEP/DIR采用两态闭环：连续STEP期间由周期速度前馈跟随，停止
 * 后立即撤掉前馈并按最后累计位置单向制动。位置累计值始终只由真实
 * STEP推进，绝不按速度外推。
 */

typedef struct
{
    PositionLoop_Config_t config;
    volatile PositionLoop_State_t state;
    uint32_t excessive_error_time_ms;
    int32_t command_initial_error_count;
    uint8_t external_reference_active;
    uint8_t external_tick_divider;
    int32_t external_reference_position_count;
    int32_t external_reference_speed_mrpm;
    uint8_t external_pulse_stream_active;
} PositionLoop_Context_t;

static PositionLoop_Context_t s_position_loop;

static int32_t PositionLoop_Absolute(int32_t value)
{
    return (value == INT32_MIN) ? INT32_MAX : ((value < 0L) ? -value : value);
}

static int32_t PositionLoop_Clamp(int64_t value, int32_t limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return (int32_t)value;
}

static int32_t PositionLoop_SubtractSaturate(int32_t minuend,
                                             int32_t subtrahend)
{
    int64_t difference = (int64_t)minuend - subtrahend;

    if (difference > INT32_MAX)
    {
        return INT32_MAX;
    }
    if (difference < INT32_MIN)
    {
        return INT32_MIN;
    }
    return (int32_t)difference;
}

/*
 * 在尚未接入轨迹规划器的调试阶段，目标位置可以是整圈阶跃；该阶跃本身
 * 不能被误判为跟随误差。仅当误差相对于本条指令的初始值继续扩大超过余量时
 * 才启动位置误差保护，可可靠识别反向、失步等异常。
 */
static uint8_t PositionLoop_IsErrorExcessive(int32_t position_error_count)
{
    int64_t allowed_error =
        (int64_t)s_position_loop.command_initial_error_count +
        s_position_loop.config.maximum_position_error_count;

    return ((int64_t)PositionLoop_Absolute(position_error_count) >
            allowed_error) ? 1U : 0U;
}

void PositionLoop_Init(void)
{
    s_position_loop.state.fault = POSITION_LOOP_FAULT_NONE;
    s_position_loop.external_reference_active = 0U;
    s_position_loop.external_pulse_stream_active = 0U;
    s_position_loop.external_tick_divider = 0U;
}

uint8_t PositionLoop_Start(const PositionLoop_Config_t *config,
                           int32_t target_position_count)
{
    TrajectoryPlanner_Config_t trajectory_config;

    if ((config == 0) || (config->kp_mrpm_per_count <= 0L) ||
        (config->kd_mrpm_per_rpm < 0L) || (config->maximum_speed_mrpm <= 0L) ||
        (config->maximum_tracking_speed_mrpm <
         config->maximum_speed_mrpm) ||
        (config->maximum_tracking_speed_mrpm >
         config->speed_config.maximum_speed_mrpm) ||
        (config->maximum_acceleration_mrpm_per_s <= 0L) ||
        (config->position_tolerance_count < 0L) ||
        (config->settle_speed_mrpm < 0L) || (config->settle_time_ms == 0U) ||
        (config->maximum_position_error_count <= 0L) ||
        (config->position_error_timeout_ms == 0U) ||
        (SpeedLoop_Start(&config->speed_config, 0L) == 0U))
    {
        s_position_loop.state.fault = POSITION_LOOP_FAULT_START_FAILED;
        return 0U;
    }
    s_position_loop.config = *config;
    TrajectoryPlanner_Init();
    s_position_loop.excessive_error_time_ms = 0U;
    s_position_loop.external_tick_divider = 0U;
    s_position_loop.external_reference_active = 0U;
    s_position_loop.external_pulse_stream_active = 0U;
    s_position_loop.state.target_position_count = target_position_count;
    s_position_loop.state.measured_position_count = Encoder_GetPositionCountFast();
    trajectory_config.maximum_speed_mrpm = config->maximum_speed_mrpm;
    trajectory_config.maximum_acceleration_mrpm_per_s =
        config->maximum_acceleration_mrpm_per_s;
    if (TrajectoryPlanner_Start(
            &trajectory_config,
            s_position_loop.state.measured_position_count,
            target_position_count) == 0U)
    {
        SpeedLoop_Stop();
        s_position_loop.state.fault = POSITION_LOOP_FAULT_START_FAILED;
        return 0U;
    }
    s_position_loop.state.reference_position_count =
        s_position_loop.state.measured_position_count;
    s_position_loop.state.position_error_count =
        PositionLoop_SubtractSaturate(
            target_position_count,
            s_position_loop.state.measured_position_count);
    s_position_loop.command_initial_error_count =
        PositionLoop_Absolute(s_position_loop.state.position_error_count);
    s_position_loop.state.speed_target_mrpm = 0L;
    s_position_loop.state.settled_time_ms = 0U;
    s_position_loop.state.reached = 0U;
    s_position_loop.state.running = 1U;
    s_position_loop.state.fault = POSITION_LOOP_FAULT_NONE;
    return 1U;
}

uint8_t PositionLoop_SetTargetPositionCount(int32_t target_position_count)
{
    if (s_position_loop.state.running == 0U) return 0U;
    s_position_loop.external_reference_active = 0U;
    s_position_loop.state.target_position_count = target_position_count;
    (void)TrajectoryPlanner_SetTargetPositionCount(target_position_count);
    s_position_loop.command_initial_error_count = PositionLoop_Absolute(
        PositionLoop_SubtractSaturate(target_position_count,
                                      Encoder_GetPositionCountFast()));
    s_position_loop.excessive_error_time_ms = 0U;
    s_position_loop.state.reached = 0U;
    s_position_loop.state.settled_time_ms = 0U;
    return 1U;
}

uint8_t PositionLoop_EnableExternalReference(int32_t initial_position_count)
{
    uint32_t interrupt_mask;

    if (s_position_loop.state.running == 0U)
    {
        return 0U;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_position_loop.external_reference_position_count = initial_position_count;
    s_position_loop.external_reference_speed_mrpm = 0L;
    s_position_loop.external_pulse_stream_active = 0U;
    s_position_loop.state.target_position_count = initial_position_count;
    s_position_loop.command_initial_error_count = PositionLoop_Absolute(
        PositionLoop_SubtractSaturate(initial_position_count,
                                      Encoder_GetPositionCountFast()));
    s_position_loop.excessive_error_time_ms = 0U;
    s_position_loop.state.reached = 0U;
    s_position_loop.state.settled_time_ms = 0U;
    s_position_loop.external_tick_divider = 0U;
    s_position_loop.external_reference_active = 1U;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

uint8_t PositionLoop_SetExternalReference(int32_t reference_position_count,
                                          int32_t reference_speed_mrpm,
                                          uint8_t pulse_stream_active)
{
    uint32_t interrupt_mask;

    if ((s_position_loop.state.running == 0U) ||
        (s_position_loop.external_reference_active == 0U))
    {
        return 0U;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_position_loop.external_reference_position_count = reference_position_count;
    s_position_loop.external_reference_speed_mrpm = reference_speed_mrpm;
    s_position_loop.external_pulse_stream_active =
        (pulse_stream_active != 0U) ? 1U : 0U;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

uint8_t PositionLoop_SetGains(int32_t kp_mrpm_per_count,
                              int32_t kd_mrpm_per_rpm)
{
    uint32_t interrupt_mask;

    if ((s_position_loop.state.running == 0U) ||
        (kp_mrpm_per_count <= 0L) || (kd_mrpm_per_rpm < 0L))
    {
        return 0U;
    }

    /* 外部STEP模式为4kHz节拍；成对更新增益，避免控制周期读取到半更新配置。 */
    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_position_loop.config.kp_mrpm_per_count = kp_mrpm_per_count;
    s_position_loop.config.kd_mrpm_per_rpm = kd_mrpm_per_rpm;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

uint8_t PositionLoop_SetMaximumSpeedMrpm(int32_t maximum_speed_mrpm)
{
    uint32_t interrupt_mask;

    if ((s_position_loop.state.running == 0U) ||
        (maximum_speed_mrpm <= 0L) ||
        (maximum_speed_mrpm >
         s_position_loop.config.speed_config.maximum_speed_mrpm))
    {
        return 0U;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_position_loop.config.maximum_speed_mrpm = maximum_speed_mrpm;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

void PositionLoop_Stop(void)
{
    SpeedLoop_Stop();
    s_position_loop.external_reference_active = 0U;
    s_position_loop.external_pulse_stream_active = 0U;
    s_position_loop.external_tick_divider = 0U;
    s_position_loop.state.speed_target_mrpm = 0L;
    s_position_loop.state.running = 0U;
}

static uint8_t PositionLoop_UpdateControl(uint8_t update_trajectory)
{
    int32_t measured_speed_mrpm;
    int64_t position_correction_mrpm;
    int64_t speed_damping_mrpm;
    int64_t speed_command_mrpm;
    uint8_t external_position_mode;

    if (s_position_loop.state.running == 0U)
    {
        return 0U;
    }
    if (SpeedLoop_GetState()->running == 0U)
    {
        s_position_loop.state.fault = POSITION_LOOP_FAULT_SPEED_LOOP;
        s_position_loop.state.running = 0U;
        return 0U;
    }
    s_position_loop.state.measured_position_count = Encoder_GetPositionCountFast();
    if (s_position_loop.external_reference_active != 0U)
    {
        s_position_loop.state.target_position_count =
            s_position_loop.external_reference_position_count;
        s_position_loop.state.reference_position_count =
            s_position_loop.external_reference_position_count;
        s_position_loop.state.reference_speed_mrpm =
            s_position_loop.external_reference_speed_mrpm;
    }
    else
    {
        if (update_trajectory == 0U)
        {
            return 0U;
        }
        TrajectoryPlanner_Update1ms();
        s_position_loop.state.reference_position_count =
            TrajectoryPlanner_GetState()->reference_position_count;
        s_position_loop.state.reference_speed_mrpm =
            TrajectoryPlanner_GetState()->reference_speed_mrpm;
    }
    s_position_loop.state.position_error_count =
        PositionLoop_SubtractSaturate(
            s_position_loop.state.reference_position_count,
            s_position_loop.state.measured_position_count);
    external_position_mode =
        (s_position_loop.external_reference_active != 0U) ? 1U : 0U;
    measured_speed_mrpm = SpeedLoop_GetState()->measured_speed_mrpm;
    if (external_position_mode != 0U)
    {
        /*
         * 外部STEP纯位置PD：脉冲数量是唯一命令，频率仅用于诊断。
         * 两个STEP之间不外推位置，也不把上一拍的频率作为速度前馈。
         * 最后STEP到来后，位置目标固定并自然收敛至该位置。
         */
        position_correction_mrpm =
            (int64_t)s_position_loop.config.kp_mrpm_per_count *
            s_position_loop.state.position_error_count;
        speed_damping_mrpm =
            ((int64_t)s_position_loop.config.kd_mrpm_per_rpm *
             measured_speed_mrpm) / 1000LL;
        speed_command_mrpm = position_correction_mrpm - speed_damping_mrpm;
        if ((s_position_loop.state.position_error_count > 0L) &&
            (speed_command_mrpm < 0L))
        {
            speed_command_mrpm = 0L;
        }
        else if ((s_position_loop.state.position_error_count < 0L) &&
                 (speed_command_mrpm > 0L))
        {
            speed_command_mrpm = 0L;
        }
    }
    else
    {
        position_correction_mrpm =
            (int64_t)s_position_loop.config.kp_mrpm_per_count *
            s_position_loop.state.position_error_count;
        speed_damping_mrpm =
            ((int64_t)s_position_loop.config.kd_mrpm_per_rpm *
             measured_speed_mrpm) / 1000LL;
        speed_command_mrpm =
            position_correction_mrpm -
            speed_damping_mrpm +
            s_position_loop.state.reference_speed_mrpm;
    }
    /*
     * 轨迹巡航速度与位置跟随命令上限必须分离：
     * 参考轨迹以maximum_speed_mrpm运行；位置落后时，P项需要少量速度
     * 余量追赶。若两者共用一个限幅，轨迹先到终点而转子尚未到位，
     * 必然在末端产生第二次纠偏动作。
     */
    s_position_loop.state.speed_target_mrpm = PositionLoop_Clamp(
        speed_command_mrpm,
        s_position_loop.config.maximum_tracking_speed_mrpm);
    (void)SpeedLoop_SetTargetSpeedMrpm(s_position_loop.state.speed_target_mrpm);

    return 1U;
}

static void PositionLoop_UpdateMonitoring1ms(void)
{
    int32_t measured_speed_mrpm = SpeedLoop_GetState()->measured_speed_mrpm;
    int32_t final_position_error_count = PositionLoop_SubtractSaturate(
        s_position_loop.state.target_position_count,
        s_position_loop.state.measured_position_count);

    if (PositionLoop_IsErrorExcessive(
            s_position_loop.state.position_error_count) != 0U)
    {
        s_position_loop.excessive_error_time_ms++;
        if (s_position_loop.excessive_error_time_ms >= s_position_loop.config.position_error_timeout_ms)
        {
            s_position_loop.state.fault = POSITION_LOOP_FAULT_POSITION_ERROR;
            PositionLoop_Stop();
        }
    }
    else s_position_loop.excessive_error_time_ms = 0U;
    if (((s_position_loop.external_reference_active != 0U) ||
         (TrajectoryPlanner_GetState()->completed != 0U)) &&
        (PositionLoop_Absolute(final_position_error_count) <=
         s_position_loop.config.position_tolerance_count) &&
        (PositionLoop_Absolute(measured_speed_mrpm) <= s_position_loop.config.settle_speed_mrpm))
    {
        s_position_loop.state.settled_time_ms++;
        s_position_loop.state.reached =
            (s_position_loop.state.settled_time_ms >= s_position_loop.config.settle_time_ms) ? 1U : 0U;
    }
    else { s_position_loop.state.settled_time_ms = 0U; s_position_loop.state.reached = 0U; }
}

void PositionLoop_Tick250us(void)
{
    if (s_position_loop.external_reference_active == 0U)
    {
        return;
    }

    if (PositionLoop_UpdateControl(0U) == 0U)
    {
        return;
    }

    s_position_loop.external_tick_divider++;
    if (s_position_loop.external_tick_divider >= 4U)
    {
        s_position_loop.external_tick_divider = 0U;
        PositionLoop_UpdateMonitoring1ms();
    }
}

void PositionLoop_Tick1ms(void)
{
    /* 外部STEP模式完全由4kHz节拍更新，避免在1ms内额外执行一次控制。 */
    if (s_position_loop.external_reference_active != 0U)
    {
        return;
    }

    if (PositionLoop_UpdateControl(1U) != 0U)
    {
        PositionLoop_UpdateMonitoring1ms();
    }
}

const volatile PositionLoop_State_t *PositionLoop_GetState(void)
{
    return &s_position_loop.state;
}
