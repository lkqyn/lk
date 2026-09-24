#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

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

void TrajectoryPlanner_Init(void);
uint8_t TrajectoryPlanner_Start(const TrajectoryPlanner_Config_t *config,
                                int32_t initial_position_count,
                                int32_t target_position_count);
uint8_t TrajectoryPlanner_SetTargetPositionCount(int32_t target_position_count);
void TrajectoryPlanner_Update1ms(void);
const TrajectoryPlanner_State_t *TrajectoryPlanner_GetState(void);

#endif
