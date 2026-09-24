#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

/*
 * 常规位置模式的一维梯形速度轨迹规划器。
 * 仅服务于内部给定位置运动；外部 STEP/DIR 的 PulsePos 不经过此模块。
 */
#include <stdint.h>

/**
 * @brief 一维梯形速度轨迹规划器配置。
 * @note 速度单位为m-rpm，加速度单位为m-rpm/s，位置单位为编码器计数。
 */
typedef struct
{
    int32_t maximum_speed_mrpm;
    int32_t maximum_acceleration_mrpm_per_s;
} TrajectoryPlanner_Config_t;

typedef struct
{
    int32_t target_position_count;
    int32_t reference_position_count;
    int32_t reference_speed_mrpm;
    uint8_t completed;
} TrajectoryPlanner_State_t;

/* 清空轨迹状态。 */
void TrajectoryPlanner_Init(void);
/* 从当前内部位置到目标位置建立一条梯形速度轨迹。 */
uint8_t TrajectoryPlanner_Start(const TrajectoryPlanner_Config_t *config,
                                int32_t initial_position_count,
                                int32_t target_position_count);
/* 运行中更新最终位置，保持当前轨迹速度连续。 */
uint8_t TrajectoryPlanner_SetTargetPositionCount(int32_t target_position_count);
/* 由 1 kHz 位置环推进一次轨迹。 */
void TrajectoryPlanner_Update1ms(void);
/* 获取轨迹位置、速度和完成状态。 */
const TrajectoryPlanner_State_t *TrajectoryPlanner_GetState(void);

#endif
