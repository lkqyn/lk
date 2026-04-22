#ifndef __MOTION_TRAP2_H__
#define __MOTION_TRAP2_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "encoder.h"
#include <stdint.h>

#ifndef MOTION_COUNTS_PER_REV
#define MOTION_COUNTS_PER_REV          (4000.0f)
#endif

#ifndef MOTION_CTRL_DT
#define MOTION_CTRL_DT                 (0.0005f)   // 2kHz
#endif

#ifndef MOTION_EPS_TIME
#define MOTION_EPS_TIME                (1e-6f)
#endif

typedef enum
{
    MOTION_MODE_IDLE = 0,
    MOTION_MODE_TRAP_BY_VA,
    MOTION_MODE_TRAP_BY_TIME
} motion_mode_t;

typedef enum
{
    MOTION_RUN_IDLE = 0,
    MOTION_RUN_RUNNING,
    MOTION_RUN_CAPTURE,   // 末端位置精定位
    MOTION_RUN_DONE,
    MOTION_RUN_ABORTED,
    MOTION_RUN_REJECTED
} motion_run_state_t;

typedef enum
{
    MOTION_OK = 0,
    MOTION_ERR_BUSY = -1,
    MOTION_ERR_PARAM = -2,
    MOTION_ERR_INFEASIBLE = -3
} motion_ret_t;

typedef struct
{
    motion_mode_t mode;
    motion_run_state_t state;

    uint8_t active;
    uint8_t reached_time;
    uint8_t capture_enable;

    int8_t dir;

    int64_t start_cnt;            // 本次实际起点
    int64_t target_cnt;           // 绝对目标位置
    int64_t cmd_delta_cnt;        // 本次位移（带符号）
    int64_t total_cnt_abs;        // 本次位移绝对值

    float cmd_turns;
    float cmd_time_s;

    float vmax_rpm_cmd;
    float acc_rpm_s_cmd;

    float vpeak_rpm;
    float acc_rpm_s;

    float t1;
    float t2;
    float t3;
    float elapsed_s;

    float pos_ref_turns;
    float speed_ref_rpm;

    float time_ratio_acc;
    float time_ratio_dec;

    float last_err_turns;
    int64_t last_err_cnt;

    // -------- 精定位参数 --------
    int32_t capture_enter_cnt;    // 小于该误差进入精定位
    int32_t done_window_cnt;      // 小于该误差且低速时完成
    float capture_kp_rpm_per_turn;
    float capture_max_rpm;
    float done_speed_rpm;
} motion_trap2_t;

extern motion_trap2_t g_motion_trap2;

void MotionTrap2_Init(motion_trap2_t *m);
void MotionTrap2_Reset(motion_trap2_t *m);
void MotionTrap2_Abort(motion_trap2_t *m);

/* 相对运动：从当前位置再走 turns 圈 */
int MotionTrap2_StartByVA(motion_trap2_t *m,
                          encoder_t *enc,
                          float turns,
                          float vmax_rpm,
                          float acc_rpm_s);

/* 绝对目标位置：走到 abs_target_cnt */
int MotionTrap2_StartToAbsCountByVA(motion_trap2_t *m,
                                    encoder_t *enc,
                                    int64_t abs_target_cnt,
                                    float vmax_rpm,
                                    float acc_rpm_s);

/* 指定时间 + 圈数 + 速度上限 */
int MotionTrap2_StartByTimeWithVmax(motion_trap2_t *m,
                                    encoder_t *enc,
                                    float turns,
                                    float total_time_s,
                                    float vmax_limit_rpm,
                                    float acc_ratio,
                                    float dec_ratio);

/* 指定时间 + 绝对目标位置 + 速度上限 */
int MotionTrap2_StartToAbsCountByTimeWithVmax(motion_trap2_t *m,
                                              encoder_t *enc,
                                              int64_t abs_target_cnt,
                                              float total_time_s,
                                              float vmax_limit_rpm,
                                              float acc_ratio,
                                              float dec_ratio);

void MotionTrap2_Update(motion_trap2_t *m,
                        encoder_t *enc,
                        float actual_speed_rpm,
                        float dt_s);

uint8_t MotionTrap2_IsBusy(const motion_trap2_t *m);
uint8_t MotionTrap2_IsDone(const motion_trap2_t *m);

float MotionTrap2_GetTargetSpeedRpm(const motion_trap2_t *m);
int64_t MotionTrap2_GetTargetCount(const motion_trap2_t *m);

#ifdef __cplusplus
}
#endif

#endif