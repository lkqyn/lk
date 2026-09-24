/* 内部位置命令使用的一维梯形速度轨迹规划器实现。 */
#include "trajectory_planner.h"

#include "motor_parameters.h"

#include <limits.h>
#include <math.h>

/* 轨迹规划与位置环共用1 kHz节拍，时间单位统一为秒。 */
#define TRAJECTORY_PLANNER_PERIOD_S       (0.001f)
#define TRAJECTORY_PLANNER_POSITION_EPSILON_COUNT (0.0001f)

typedef struct
{
    TrajectoryPlanner_Config_t config;
    TrajectoryPlanner_State_t state;

    /* 以下量只在启动/重新规划时计算，周期内仅按解析式求值。 */
    float initial_position_count;
    float direction;
    float acceleration_count_per_s2;
    float peak_speed_count_per_s;
    float acceleration_time_s;
    float cruise_time_s;
    float total_time_s;
    float acceleration_distance_count;
    float elapsed_time_s;
} TrajectoryPlanner_Context_t;

static TrajectoryPlanner_Context_t s_trajectory_planner;

/* 将轨迹浮点位置按对称四舍五入导出为编码器整数计数。 */
static int32_t TrajectoryPlanner_RoundToInt32(float value)
{
    if (value >= (float)INT32_MAX)
    {
        return INT32_MAX;
    }
    if (value <= (float)INT32_MIN)
    {
        return INT32_MIN;
    }

    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                             (int32_t)(value - 0.5f);
}

/* 将编码器 count/s 的轨迹速度换算为机械 mrpm。 */
static int32_t TrajectoryPlanner_SpeedCountPerSecondToMrpm(float speed_count_per_s)
{
    float speed_mrpm = speed_count_per_s * 60000.0f /
                       (float)MOTOR_ENCODER_COUNTS_PER_REVOLUTION;

    return TrajectoryPlanner_RoundToInt32(speed_mrpm);
}

/*
 * 从静止状态规划一段完整轨迹。三角轨迹和梯形轨迹均由同一套解析表达式
 * 求值，不通过“剩余距离是否跨过终点”的逐周期判断，避免末端量化时出现
 * 参考位置先跳到终点、速度前馈却尚未自然降为零的情况。
 */
/* 为静止起止的点到点运动计算加速、巡航和减速分界。 */
static void TrajectoryPlanner_BuildRestToRestPlan(float initial_position_count,
                                                   int32_t target_position_count)
{
    float travel_distance_count;
    float maximum_speed_count_per_s;
    float acceleration_distance_at_maximum_speed;

    s_trajectory_planner.initial_position_count = initial_position_count;
    s_trajectory_planner.state.target_position_count = target_position_count;
    s_trajectory_planner.elapsed_time_s = 0.0f;
    s_trajectory_planner.cruise_time_s = 0.0f;
    s_trajectory_planner.acceleration_time_s = 0.0f;
    s_trajectory_planner.total_time_s = 0.0f;
    s_trajectory_planner.peak_speed_count_per_s = 0.0f;
    s_trajectory_planner.acceleration_distance_count = 0.0f;
    s_trajectory_planner.state.reference_position_count =
        TrajectoryPlanner_RoundToInt32(initial_position_count);
    s_trajectory_planner.state.reference_speed_mrpm = 0L;

    travel_distance_count = (float)target_position_count - initial_position_count;
    if (fabsf(travel_distance_count) <= TRAJECTORY_PLANNER_POSITION_EPSILON_COUNT)
    {
        s_trajectory_planner.direction = 1.0f;
        s_trajectory_planner.state.reference_position_count = target_position_count;
        s_trajectory_planner.state.completed = 1U;
        return;
    }

    s_trajectory_planner.direction = (travel_distance_count >= 0.0f) ? 1.0f : -1.0f;
    travel_distance_count = fabsf(travel_distance_count);
    maximum_speed_count_per_s =
        (float)s_trajectory_planner.config.maximum_speed_mrpm *
        (float)MOTOR_ENCODER_COUNTS_PER_REVOLUTION / 60000.0f;
    acceleration_distance_at_maximum_speed =
        (maximum_speed_count_per_s * maximum_speed_count_per_s) /
        (2.0f * s_trajectory_planner.acceleration_count_per_s2);

    if (travel_distance_count >= (2.0f * acceleration_distance_at_maximum_speed))
    {
        /* 梯形轨迹：加速、恒速、减速三段齐全。 */
        s_trajectory_planner.peak_speed_count_per_s = maximum_speed_count_per_s;
        s_trajectory_planner.acceleration_time_s =
            maximum_speed_count_per_s / s_trajectory_planner.acceleration_count_per_s2;
        s_trajectory_planner.acceleration_distance_count =
            acceleration_distance_at_maximum_speed;
        s_trajectory_planner.cruise_time_s =
            (travel_distance_count - (2.0f * acceleration_distance_at_maximum_speed)) /
            maximum_speed_count_per_s;
    }
    else
    {
        /* 三角轨迹：行程不足以达到限速，峰值速度由路程决定。 */
        s_trajectory_planner.peak_speed_count_per_s = sqrtf(
            travel_distance_count * s_trajectory_planner.acceleration_count_per_s2);
        s_trajectory_planner.acceleration_time_s =
            s_trajectory_planner.peak_speed_count_per_s /
            s_trajectory_planner.acceleration_count_per_s2;
        s_trajectory_planner.acceleration_distance_count =
            travel_distance_count * 0.5f;
    }

    s_trajectory_planner.total_time_s =
        (2.0f * s_trajectory_planner.acceleration_time_s) +
        s_trajectory_planner.cruise_time_s;
    s_trajectory_planner.state.completed = 0U;
}

/* 清零内部轨迹参数和导出状态。 */
void TrajectoryPlanner_Init(void)
{
    s_trajectory_planner.state.target_position_count = 0L;
    s_trajectory_planner.state.reference_position_count = 0L;
    s_trajectory_planner.state.reference_speed_mrpm = 0L;
    s_trajectory_planner.state.completed = 1U;
    s_trajectory_planner.elapsed_time_s = 0.0f;
}

/* 校验限制并从初始位置建立一条新的梯形速度轨迹。 */
uint8_t TrajectoryPlanner_Start(const TrajectoryPlanner_Config_t *config,
                                int32_t initial_position_count,
                                int32_t target_position_count)
{
    if ((config == 0) || (config->maximum_speed_mrpm <= 0L) ||
        (config->maximum_acceleration_mrpm_per_s <= 0L))
    {
        return 0U;
    }

    s_trajectory_planner.config = *config;
    s_trajectory_planner.acceleration_count_per_s2 =
        (float)config->maximum_acceleration_mrpm_per_s *
        (float)MOTOR_ENCODER_COUNTS_PER_REVOLUTION / 60000.0f;
    TrajectoryPlanner_BuildRestToRestPlan((float)initial_position_count,
                                          target_position_count);
    return 1U;
}

/* 更新目标位置并从当前轨迹位置重新计算剩余运动。 */
uint8_t TrajectoryPlanner_SetTargetPositionCount(int32_t target_position_count)
{
    if (s_trajectory_planner.acceleration_count_per_s2 <= 0.0f)
    {
        return 0U;
    }

    /*
     * 当前模块的指令语义是“从当前参考点启动一条新的静止到静止轨迹”。
     * 连续脉冲的速度连续拼接属于后续独立的队列/轨迹模块，不能在此处用
     * 未验证的近似算法掩盖速度突变。
     */
    TrajectoryPlanner_BuildRestToRestPlan(
        (float)s_trajectory_planner.state.reference_position_count,
        target_position_count);
    return 1U;
}

/* 按 1 ms 步长积分轨迹，输出当前参考位置和速度。 */
void TrajectoryPlanner_Update1ms(void)
{
    float elapsed_time_s;
    float travelled_distance_count;
    float reference_speed_count_per_s;
    float deceleration_time_s;

    if (s_trajectory_planner.state.completed != 0U)
    {
        return;
    }

    s_trajectory_planner.elapsed_time_s += TRAJECTORY_PLANNER_PERIOD_S;
    elapsed_time_s = s_trajectory_planner.elapsed_time_s;
    if (elapsed_time_s >= s_trajectory_planner.total_time_s)
    {
        s_trajectory_planner.state.reference_position_count =
            s_trajectory_planner.state.target_position_count;
        s_trajectory_planner.state.reference_speed_mrpm = 0L;
        s_trajectory_planner.state.completed = 1U;
        return;
    }

    if (elapsed_time_s < s_trajectory_planner.acceleration_time_s)
    {
        travelled_distance_count = 0.5f *
            s_trajectory_planner.acceleration_count_per_s2 *
            elapsed_time_s * elapsed_time_s;
        reference_speed_count_per_s =
            s_trajectory_planner.acceleration_count_per_s2 * elapsed_time_s;
    }
    else if (elapsed_time_s < (s_trajectory_planner.acceleration_time_s +
                               s_trajectory_planner.cruise_time_s))
    {
        travelled_distance_count = s_trajectory_planner.acceleration_distance_count +
            (s_trajectory_planner.peak_speed_count_per_s *
             (elapsed_time_s - s_trajectory_planner.acceleration_time_s));
        reference_speed_count_per_s = s_trajectory_planner.peak_speed_count_per_s;
    }
    else
    {
        deceleration_time_s = elapsed_time_s -
            s_trajectory_planner.acceleration_time_s -
            s_trajectory_planner.cruise_time_s;
        travelled_distance_count = s_trajectory_planner.acceleration_distance_count +
            (s_trajectory_planner.peak_speed_count_per_s *
             s_trajectory_planner.cruise_time_s) +
            (s_trajectory_planner.peak_speed_count_per_s * deceleration_time_s) -
            (0.5f * s_trajectory_planner.acceleration_count_per_s2 *
             deceleration_time_s * deceleration_time_s);
        reference_speed_count_per_s = s_trajectory_planner.peak_speed_count_per_s -
            (s_trajectory_planner.acceleration_count_per_s2 * deceleration_time_s);
    }

    s_trajectory_planner.state.reference_position_count =
        TrajectoryPlanner_RoundToInt32(
            s_trajectory_planner.initial_position_count +
            (s_trajectory_planner.direction * travelled_distance_count));
    s_trajectory_planner.state.reference_speed_mrpm =
        TrajectoryPlanner_SpeedCountPerSecondToMrpm(
            s_trajectory_planner.direction * reference_speed_count_per_s);
}

/* 返回内部轨迹的只读导出状态。 */
const TrajectoryPlanner_State_t *TrajectoryPlanner_GetState(void)
{
    return &s_trajectory_planner.state;
}
