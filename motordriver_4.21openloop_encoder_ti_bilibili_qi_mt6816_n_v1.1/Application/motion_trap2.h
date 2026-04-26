#ifndef __MOTION_TRAP2_H__
#define __MOTION_TRAP2_H__

#include <stdint.h>

typedef enum
{
    MOTION_TRAP2_MODE_IDLE = 0,//空闲模式
    MOTION_TRAP2_MODE_MOVE_ABS,//带减速指定圈数停止
    MOTION_TRAP2_MODE_RUN_FOREVER//加速到指定速度一直运行
} motion_trap2_mode_t;

typedef struct
{
    // 当前输出
    float pos;          // 当前规划位置 rev
    float vel;          // 当前规划速度 rev/s
    uint8_t busy;       // 1=运行中，0=结束

    // 模式
    motion_trap2_mode_t mode;

    // 轨迹目标
    float start_pos;    // 起点 rev
    float target_pos;   // 终点 rev（MOVE_ABS模式使用）
    float target_vel;   // 目标速度 rev/s（RUN_FOREVER模式使用）
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

// 新增：加速到指定速度后一直跑
void MotionTrap2_StartRunForever(motion_trap2_t *m,
                                 float start_pos_rev,
                                 float target_vel_rev_s,
                                 float acc_rev_s2);

void MotionTrap2_Update(motion_trap2_t *m, float dt);

void MotionTrap2_Stop(motion_trap2_t *m);

#endif