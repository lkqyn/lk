#ifndef __MOTION_TRAP2_H__
#define __MOTION_TRAP2_H__

#include <stdint.h>

typedef struct
{
    // 当前输出
    float pos;          // 当前规划位置 rev
    float vel;          // 当前规划速度 rev/s
    uint8_t busy;       // 1=运行中，0=结束

    // 轨迹目标
    float start_pos;    // 起点 rev
    float target_pos;   // 终点 rev
    float dir;          // 方向 +1/-1
    float dist;         // 总距离，恒正 rev

    // 约束
    float v_max;        // 最大速度 rev/s
    float acc;          // 加速度 rev/s^2
    float dec;          // 减速度 rev/s^2

    // 梯形/三角形参数
    float t1;           // 加速段时间
    float t2;           // 匀速段时间
    float t3;           // 减速段时间
    float t_total;      // 总时间

    float s1;           // 加速段位移
    float s2;           // 匀速段位移
    float s3;           // 减速段位移

    float v_peak;       // 实际峰值速度（可能小于v_max，三角形时）

    // 运行时
    float t;            // 已运行时间
} motion_trap2_t;

void MotionTrap2_Init(motion_trap2_t *m);

void MotionTrap2_StartMoveAbs(motion_trap2_t *m,
                              float start_pos_rev,
                              float target_pos_rev,
                              float v_max_rev_s,
                              float acc_rev_s2,
                              float dec_rev_s2);

void MotionTrap2_StartMoveRel(motion_trap2_t *m,
                              float start_pos_rev,
                              float delta_rev,
                              float v_max_rev_s,
                              float acc_rev_s2,
                              float dec_rev_s2);

void MotionTrap2_Update(motion_trap2_t *m, float dt);

void MotionTrap2_Stop(motion_trap2_t *m);

#endif
