#ifndef POSITION_LOOP_H
#define POSITION_LOOP_H

/*
 * 位置外环。
 * 常规模式把位置误差转换为速度命令并驱动速度环；外部 STEP 模式还支持
 * 4 kHz CmdPos 引用和直接位置-转矩试验分支。
 */
#include <stdint.h>

#include "speed_loop.h"

typedef struct
{
    int32_t kp_mrpm_per_count;
    int32_t kd_mrpm_per_rpm;
    /* 外部STEP直接位置-转矩模式的刚度、速度阻尼和安全Iq限幅。 */
    int32_t external_torque_kp_ma_per_count;
    int32_t external_torque_kd_ma_per_rpm;
    int32_t external_torque_limit_ma;
    /* 已确认外部STEP连续减速时使用的预制动速度阻尼。 */
    int32_t external_deceleration_kd_mrpm_per_rpm;
    /* 外部STEP流结束后的专用速度阻尼；运行中的脉冲流仍使用kd。 */
    int32_t external_hold_kd_mrpm_per_rpm;
    /* 轨迹规划的巡航速度上限，即对外声明的运动最高速度。 */
    int32_t maximum_speed_mrpm;
    /* 位置反馈叠加前馈后的速度命令上限，必须不小于巡航速度。 */
    int32_t maximum_tracking_speed_mrpm;
    int32_t maximum_acceleration_mrpm_per_s;
    int32_t position_tolerance_count;
    int32_t settle_speed_mrpm;
    uint32_t settle_time_ms;
    int32_t maximum_position_error_count;
    uint32_t position_error_timeout_ms;
    SpeedLoop_Config_t speed_config;
} PositionLoop_Config_t;

typedef enum
{
    POSITION_LOOP_FAULT_NONE = 0,
    POSITION_LOOP_FAULT_START_FAILED,
    POSITION_LOOP_FAULT_POSITION_ERROR,
    POSITION_LOOP_FAULT_SPEED_LOOP
} PositionLoop_Fault_t;

typedef struct
{
    int32_t target_position_count;
    int32_t reference_position_count;
    int32_t measured_position_count;
    int32_t position_error_count;
    int32_t speed_target_mrpm;
    int32_t reference_speed_mrpm;
    uint32_t settled_time_ms;
    uint8_t reached;
    uint8_t running;
    PositionLoop_Fault_t fault;
} PositionLoop_State_t;

/* 初始化位置环状态，不启动下游速度环。 */
void PositionLoop_Init(void);
/* 启动常规内部轨迹位置模式，并同时启动速度环。 */
uint8_t PositionLoop_Start(const PositionLoop_Config_t *config,
                           int32_t target_position_count);
/* 修改常规内部轨迹的最终目标位置。 */
uint8_t PositionLoop_SetTargetPositionCount(int32_t target_position_count);
/* 由外部STEP/DIR接口直接提供位置参考；不经过内部轨迹规划器。 */
uint8_t PositionLoop_EnableExternalReference(int32_t initial_position_count);
uint8_t PositionLoop_SetExternalReference(int32_t reference_position_count,
                                          int32_t reference_speed_mrpm,
                                          uint8_t pulse_stream_active,
                                          uint8_t pulse_decelerating);
/* 使外部STEP参考直接产生Iq，不再经过位置到速度再到速度PI的链路。 */
uint8_t PositionLoop_SetExternalDirectTorqueMode(uint8_t enabled);
/* 在线更新位置刚度和速度阻尼，仅在位置环运行时使用。 */
uint8_t PositionLoop_SetGains(int32_t kp_mrpm_per_count,
                              int32_t kd_mrpm_per_rpm);
/* 更新常规轨迹规划的最高速度限制。 */
uint8_t PositionLoop_SetMaximumSpeedMrpm(int32_t maximum_speed_mrpm);
/* 停止位置环及其拥有的速度环。 */
void PositionLoop_Stop(void);
/* 外部STEP参考专用的4kHz控制更新；保护与到位计时仍按1ms计。 */
void PositionLoop_Tick250us(void);
/* 1 kHz 更新保护、到位判定和常规内部轨迹。 */
void PositionLoop_Tick1ms(void);
/* 获取位置环的只读状态快照。 */
const volatile PositionLoop_State_t *PositionLoop_GetState(void);

#endif
