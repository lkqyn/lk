#include "motion_trap2.h"
#include <math.h>

static float f_abs(float x)
{
    return (x >= 0.0f) ? x : -x;
}

void MotionTrap2_Init(motion_trap2_t *m)
{
    m->mode = MOTION_TRAP2_MODE_IDLE;
    m->target_vel = 0.0f;

    m->pos = 0.0f;
    m->vel = 0.0f;
    m->busy = 0;

    m->start_pos = 0.0f;
    m->target_pos = 0.0f;
    m->dir = 1.0f;
    m->dist = 0.0f;

    m->v_max = 1.0f;
    m->acc = 1.0f;
    m->dec = 1.0f;

    m->t1 = 0.0f;
    m->t2 = 0.0f;
    m->t3 = 0.0f;
    m->t_total = 0.0f;

    m->s1 = 0.0f;
    m->s2 = 0.0f;
    m->s3 = 0.0f;

    m->v_peak = 0.0f;
    m->t = 0.0f;
}

void MotionTrap2_StartMoveAbs(motion_trap2_t *m,
                              float start_pos_rev,
                              float target_pos_rev,
                              float v_max_rev_s,
                              float acc_rev_s2,
                              float dec_rev_s2)
{
    float dist_signed;
    float dist_abs;
    float s_acc_to_vmax;
    float s_dec_from_vmax;
    float v_peak;

    m->mode = MOTION_TRAP2_MODE_MOVE_ABS;
    m->target_vel = 0.0f;

    m->start_pos = start_pos_rev;
    m->target_pos = target_pos_rev;

    m->v_max = (v_max_rev_s > 0.0f) ? v_max_rev_s : 1.0f;
    m->acc   = (acc_rev_s2 > 0.0f) ? acc_rev_s2 : 1.0f;
    m->dec   = (dec_rev_s2 > 0.0f) ? dec_rev_s2 : 1.0f;

    m->t = 0.0f;
    m->pos = start_pos_rev;
    m->vel = 0.0f;

    dist_signed = target_pos_rev - start_pos_rev;
    dist_abs = f_abs(dist_signed);
    m->dist = dist_abs;

    if (dist_abs < 1e-9f)
    {
        m->busy = 0;
        m->mode = MOTION_TRAP2_MODE_IDLE;
        m->dir = 1.0f;
        m->t1 = m->t2 = m->t3 = m->t_total = 0.0f;
        m->s1 = m->s2 = m->s3 = 0.0f;
        m->v_peak = 0.0f;
        return;
    }

    m->dir = (dist_signed >= 0.0f) ? 1.0f : -1.0f;

    // 假设能跑到 vmax，所需加减速距离
    s_acc_to_vmax   = 0.5f * m->v_max * m->v_max / m->acc;
    s_dec_from_vmax = 0.5f * m->v_max * m->v_max / m->dec;

    if ((s_acc_to_vmax + s_dec_from_vmax) <= dist_abs)
    {
        // 梯形轨迹
        m->v_peak = m->v_max;

        m->t1 = m->v_peak / m->acc;
        m->t3 = m->v_peak / m->dec;

        m->s1 = 0.5f * m->acc * m->t1 * m->t1;
        m->s3 = 0.5f * m->dec * m->t3 * m->t3;
        m->s2 = dist_abs - m->s1 - m->s3;

        m->t2 = m->s2 / m->v_peak;
    }
    else
    {
        // 三角形轨迹，达不到 vmax
        v_peak = sqrtf((2.0f * dist_abs * m->acc * m->dec) / (m->acc + m->dec));
        m->v_peak = v_peak;

        m->t1 = m->v_peak / m->acc;
        m->t2 = 0.0f;
        m->t3 = m->v_peak / m->dec;

        m->s1 = 0.5f * m->acc * m->t1 * m->t1;
        m->s2 = 0.0f;
        m->s3 = 0.5f * m->dec * m->t3 * m->t3;
    }

    m->t_total = m->t1 + m->t2 + m->t3;
    m->busy = 1;
}

void MotionTrap2_StartMoveRel(motion_trap2_t *m,
                              float start_pos_rev,
                              float delta_rev,
                              float v_max_rev_s,
                              float acc_rev_s2,
                              float dec_rev_s2)
{
    MotionTrap2_StartMoveAbs(m,
                             start_pos_rev,
                             start_pos_rev + delta_rev,
                             v_max_rev_s,
                             acc_rev_s2,
                             dec_rev_s2);
}

void MotionTrap2_StartRunForever(motion_trap2_t *m,
                                 float start_pos_rev,
                                 float target_vel_rev_s,
                                 float acc_rev_s2)
{
    float v_abs;

    m->mode = MOTION_TRAP2_MODE_RUN_FOREVER;

    m->start_pos = start_pos_rev;
    m->target_pos = start_pos_rev;   // 此模式下不用
    m->target_vel = target_vel_rev_s;

    m->pos = start_pos_rev;
    m->vel = 0.0f;
    m->t = 0.0f;
    m->dist = 0.0f;

    m->acc = (acc_rev_s2 > 0.0f) ? acc_rev_s2 : 1.0f;
    m->dec = m->acc;

    if (target_vel_rev_s >= 0.0f)
    {
        m->dir = 1.0f;
        v_abs = target_vel_rev_s;
    }
    else
    {
        m->dir = -1.0f;
        v_abs = -target_vel_rev_s;
    }

    m->v_max = v_abs;
    m->v_peak = v_abs;

    if (v_abs < 1e-9f)
    {
        m->busy = 0;
        m->mode = MOTION_TRAP2_MODE_IDLE;

        m->t1 = 0.0f;
        m->t2 = 0.0f;
        m->t3 = 0.0f;
        m->t_total = 0.0f;

        m->s1 = 0.0f;
        m->s2 = 0.0f;
        m->s3 = 0.0f;
        return;
    }

    // 只保留加速段，后面一直匀速
    m->t1 = v_abs / m->acc;
    m->t2 = 0.0f;
    m->t3 = 0.0f;
    m->t_total = 0.0f;

    m->s1 = 0.5f * m->acc * m->t1 * m->t1;
    m->s2 = 0.0f;
    m->s3 = 0.0f;

    m->busy = 1;
}

void MotionTrap2_Update(motion_trap2_t *m, float dt)
{
    float t;
    float s;
    float v;

    if (!m->busy)
        return;

    m->t += dt;
    t = m->t;

    // =========================
    // 点到点模式
    // =========================
    if (m->mode == MOTION_TRAP2_MODE_MOVE_ABS)
    {
        if (t >= m->t_total)
        {
            m->t = m->t_total;
            m->pos = m->target_pos;
            m->vel = 0.0f;
            m->busy = 0;
            m->mode = MOTION_TRAP2_MODE_IDLE;
            return;
        }

        if (t < m->t1)
        {
            // 加速段
            v = m->acc * t;
            s = 0.5f * m->acc * t * t;
        }
        else if (t < (m->t1 + m->t2))
        {
            // 匀速段
            float tc = t - m->t1;
            v = m->v_peak;
            s = m->s1 + m->v_peak * tc;
        }
        else
        {
            // 减速段
            float td = t - m->t1 - m->t2;
            v = m->v_peak - m->dec * td;
            if (v < 0.0f) v = 0.0f;

            s = m->s1 + m->s2 + m->v_peak * td - 0.5f * m->dec * td * td;
        }

        m->pos = m->start_pos + m->dir * s;
        m->vel = m->dir * v;
        return;
    }

    // =========================
    // 连续运行模式：加速到目标速度后一直跑
    // =========================
    if (m->mode == MOTION_TRAP2_MODE_RUN_FOREVER)
    {
        if (t < m->t1)
        {
            // 加速段
            v = m->acc * t;
            if (v > m->v_peak) v = m->v_peak;

            s = 0.5f * m->acc * t * t;
        }
        else
        {
            // 匀速一直跑
            float tc = t - m->t1;
            v = m->v_peak;
            s = m->s1 + m->v_peak * tc;
        }

        m->pos = m->start_pos + m->dir * s;
        m->vel = m->dir * v;
        return;
    }

    // 其他未知模式，安全回空闲
    m->busy = 0;
    m->vel = 0.0f;
    m->mode = MOTION_TRAP2_MODE_IDLE;
}

void MotionTrap2_Stop(motion_trap2_t *m)
{
    m->busy = 0;
    m->vel = 0.0f;
    m->mode = MOTION_TRAP2_MODE_IDLE;
}