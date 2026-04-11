#include "motion_trap2.h"
#include "math.h"
#include "connecting.h"
#include "pid.h"

motion_trap2_t g_motion_trap2;

extern connect_crt_TypeDef connect_crt;
extern PID_Controller_t speed_pi;

static float mt_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static int64_t mt_llabs64(int64_t x)
{
    return (x >= 0) ? x : (-x);
}

static float mt_clampf(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

static void mt_set_idle_output(motion_trap2_t *m)
{
    connect_crt.speed = 0.0f;
    speed_pi.target = 0.0f;
    m->speed_ref_rpm = 0.0f;
}

static void mt_finalize_done(motion_trap2_t *m)
{
    m->active = 0;
    m->state = MOTION_RUN_DONE;
    mt_set_idle_output(m);
}

static void mt_finalize_abort(motion_trap2_t *m)
{
    m->active = 0;
    m->state = MOTION_RUN_ABORTED;
    mt_set_idle_output(m);
}

static void mt_finalize_reject(motion_trap2_t *m)
{
    m->active = 0;
    m->state = MOTION_RUN_REJECTED;
    mt_set_idle_output(m);
}

static void mt_prepare_start(motion_trap2_t *m,
                             encoder_t *enc,
                             int64_t abs_target_cnt)
{
    int64_t delta_cnt = abs_target_cnt - enc->pos_cnt_total;

    m->start_cnt = enc->pos_cnt_total;
    m->target_cnt = abs_target_cnt;
    m->cmd_delta_cnt = delta_cnt;
    m->total_cnt_abs = mt_llabs64(delta_cnt);

    m->cmd_turns = (float)delta_cnt / MOTION_COUNTS_PER_REV;
    m->dir = (delta_cnt >= 0) ? 1 : -1;

    m->elapsed_s = 0.0f;
    m->pos_ref_turns = 0.0f;
    m->speed_ref_rpm = 0.0f;
    m->reached_time = 0;
    m->last_err_turns = 0.0f;
    m->last_err_cnt = 0;

    m->active = 1;
    m->state = MOTION_RUN_RUNNING;

    connect_crt.motor_mode = 2;
    connect_crt.speed = 0.0f;
    speed_pi.target = 0.0f;
}

void MotionTrap2_Init(motion_trap2_t *m)
{
    if (m == 0) return;

    m->mode = MOTION_MODE_IDLE;
    m->state = MOTION_RUN_IDLE;

    m->active = 0;
    m->reached_time = 0;
    m->capture_enable = 1;
    m->dir = 1;

    m->start_cnt = 0;
    m->target_cnt = 0;
    m->cmd_delta_cnt = 0;
    m->total_cnt_abs = 0;

    m->cmd_turns = 0.0f;
    m->cmd_time_s = 0.0f;

    m->vmax_rpm_cmd = 0.0f;
    m->acc_rpm_s_cmd = 0.0f;

    m->vpeak_rpm = 0.0f;
    m->acc_rpm_s = 0.0f;

    m->t1 = 0.0f;
    m->t2 = 0.0f;
    m->t3 = 0.0f;
    m->elapsed_s = 0.0f;

    m->pos_ref_turns = 0.0f;
    m->speed_ref_rpm = 0.0f;

    m->time_ratio_acc = 0.25f;
    m->time_ratio_dec = 0.25f;

    m->last_err_turns = 0.0f;
    m->last_err_cnt = 0;

    // 精定位默认参数：你可以后面再微调
    m->capture_enter_cnt = 20;          // 小于20count进精定位
    m->done_window_cnt = 2;             // 小于2count算完成  误差小于0.0005圈
    m->capture_kp_rpm_per_turn = 800.0f; // 1圈误差 -> 800rpm
    m->capture_max_rpm = 40.0f;         // 精定位最大速度
    m->done_speed_rpm = 3.0f;           // 低于3rpm且进窗口算完成
}

void MotionTrap2_Reset(motion_trap2_t *m)
{
    MotionTrap2_Init(m);
}

void MotionTrap2_Abort(motion_trap2_t *m)
{
    if (m == 0) return;
    mt_finalize_abort(m);
}

int MotionTrap2_StartByVA(motion_trap2_t *m,
                          encoder_t *enc,
                          float turns,
                          float vmax_rpm,
                          float acc_rpm_s)
{
    int64_t abs_target_cnt;

    if ((m == 0) || (enc == 0))
        return MOTION_ERR_PARAM;

    abs_target_cnt = enc->pos_cnt_total + (int64_t)(turns * MOTION_COUNTS_PER_REV);

    return MotionTrap2_StartToAbsCountByVA(m, enc, abs_target_cnt, vmax_rpm, acc_rpm_s);
}

int MotionTrap2_StartToAbsCountByVA(motion_trap2_t *m,
                                    encoder_t *enc,
                                    int64_t abs_target_cnt,
                                    float vmax_rpm,
                                    float acc_rpm_s)
{
    float S_rev;
    float V_rps;
    float A_rps2;
    float Ta;
    float Sa;

    if ((m == 0) || (enc == 0))
        return MOTION_ERR_PARAM;

    if (m->active)
        return MOTION_ERR_BUSY;

    if ((vmax_rpm <= 0.0f) || (acc_rpm_s <= 0.0f))
        return MOTION_ERR_PARAM;

    if (abs_target_cnt == enc->pos_cnt_total)
        return MOTION_ERR_PARAM;

    MotionTrap2_Reset(m);

    m->mode = MOTION_MODE_TRAP_BY_VA;
    m->cmd_time_s = 0.0f;
    m->vmax_rpm_cmd = vmax_rpm;
    m->acc_rpm_s_cmd = acc_rpm_s;

    mt_prepare_start(m, enc, abs_target_cnt);

    S_rev = mt_absf(m->cmd_turns);
    V_rps = vmax_rpm / 60.0f;
    A_rps2 = acc_rpm_s / 60.0f;

    Ta = V_rps / A_rps2;
    Sa = 0.5f * A_rps2 * Ta * Ta;

    if ((2.0f * Sa) < S_rev)
    {
        float Sc = S_rev - 2.0f * Sa;
        float Tc = Sc / V_rps;

        m->vpeak_rpm = vmax_rpm;
        m->acc_rpm_s = acc_rpm_s;
        m->t1 = Ta;
        m->t2 = Ta + Tc;
        m->t3 = Ta + Tc + Ta;
    }
    else
    {
        float Vp_rps = sqrtf(S_rev * A_rps2);
        float Tp = Vp_rps / A_rps2;

        m->vpeak_rpm = Vp_rps * 60.0f;
        m->acc_rpm_s = acc_rpm_s;
        m->t1 = Tp;
        m->t2 = Tp;
        m->t3 = 2.0f * Tp;
    }

    return MOTION_OK;
}

int MotionTrap2_StartByTimeWithVmax(motion_trap2_t *m,
                                    encoder_t *enc,
                                    float turns,
                                    float total_time_s,
                                    float vmax_limit_rpm,
                                    float acc_ratio,
                                    float dec_ratio)
{
    int64_t abs_target_cnt;

    if ((m == 0) || (enc == 0))
        return MOTION_ERR_PARAM;

    abs_target_cnt = enc->pos_cnt_total + (int64_t)(turns * MOTION_COUNTS_PER_REV);

    return MotionTrap2_StartToAbsCountByTimeWithVmax(m,
                                                     enc,
                                                     abs_target_cnt,
                                                     total_time_s,
                                                     vmax_limit_rpm,
                                                     acc_ratio,
                                                     dec_ratio);
}

int MotionTrap2_StartToAbsCountByTimeWithVmax(motion_trap2_t *m,
                                              encoder_t *enc,
                                              int64_t abs_target_cnt,
                                              float total_time_s,
                                              float vmax_limit_rpm,
                                              float acc_ratio,
                                              float dec_ratio)
{
    float S_rev;
    float T;
    float Ta;
    float Td;
    float Tc;
    float A_rps2;
    float V_rps;
    float denom;

    if ((m == 0) || (enc == 0))
        return MOTION_ERR_PARAM;

    if (m->active)
        return MOTION_ERR_BUSY;

    if ((total_time_s <= 0.0f) || (vmax_limit_rpm <= 0.0f))
        return MOTION_ERR_PARAM;

    if ((acc_ratio <= 0.0f) || (dec_ratio <= 0.0f) || ((acc_ratio + dec_ratio) >= 1.0f))
        return MOTION_ERR_PARAM;

    if (abs_target_cnt == enc->pos_cnt_total)
        return MOTION_ERR_PARAM;

    MotionTrap2_Reset(m);

    m->mode = MOTION_MODE_TRAP_BY_TIME;
    m->cmd_time_s = total_time_s;
    m->vmax_rpm_cmd = vmax_limit_rpm;
    m->time_ratio_acc = acc_ratio;
    m->time_ratio_dec = dec_ratio;

    mt_prepare_start(m, enc, abs_target_cnt);

    S_rev = mt_absf(m->cmd_turns);
    T = total_time_s;

    Ta = acc_ratio * T;
    Td = dec_ratio * T;
    Tc = T - Ta - Td;

    if ((Ta <= MOTION_EPS_TIME) || (Td <= MOTION_EPS_TIME) || (Tc < 0.0f))
    {
        mt_finalize_reject(m);
        return MOTION_ERR_PARAM;
    }

    denom = Ta * (0.5f * Ta + Tc + 0.5f * Td);
    if (denom <= MOTION_EPS_TIME)
    {
        mt_finalize_reject(m);
        return MOTION_ERR_PARAM;
    }

    A_rps2 = S_rev / denom;
    V_rps = A_rps2 * Ta;

    m->acc_rpm_s = A_rps2 * 60.0f;
    m->vpeak_rpm = V_rps * 60.0f;

    m->t1 = Ta;
    m->t2 = Ta + Tc;
    m->t3 = T;

    if (m->vpeak_rpm > vmax_limit_rpm)
    {
        mt_finalize_reject(m);
        return MOTION_ERR_INFEASIBLE;
    }

    return MOTION_OK;
}

static float mt_eval_speed_rpm(const motion_trap2_t *m, float t)
{
    float v;

    if (t <= 0.0f)
        return 0.0f;

    if (t < m->t1)
    {
        v = m->acc_rpm_s * t;
    }
    else if (t < m->t2)
    {
        v = m->vpeak_rpm;
    }
    else if (t < m->t3)
    {
        v = m->vpeak_rpm - m->acc_rpm_s * (t - m->t2);
        if (v < 0.0f) v = 0.0f;
    }
    else
    {
        v = 0.0f;
    }

    return (float)m->dir * v;
}

static float mt_eval_pos_turns(const motion_trap2_t *m, float t)
{
    float pos_rev = 0.0f;
    float Ta = m->t1;
    float Tc = m->t2 - m->t1;
    float Td = m->t3 - m->t2;
    float A = m->acc_rpm_s / 60.0f;
    float V = m->vpeak_rpm / 60.0f;

    if (t <= 0.0f)
    {
        pos_rev = 0.0f;
    }
    else if (t < Ta)
    {
        pos_rev = 0.5f * A * t * t;
    }
    else if (t < (Ta + Tc))
    {
        float dt = t - Ta;
        float Sa = 0.5f * A * Ta * Ta;
        pos_rev = Sa + V * dt;
    }
    else if (t < (Ta + Tc + Td))
    {
        float dt = t - (Ta + Tc);
        float Sa = 0.5f * A * Ta * Ta;
        float Sc = V * Tc;
        pos_rev = Sa + Sc + V * dt - 0.5f * A * dt * dt;
    }
    else
    {
        pos_rev = mt_absf(m->cmd_turns);
    }

    return (float)m->dir * pos_rev;
}

void MotionTrap2_Update(motion_trap2_t *m,
                        encoder_t *enc,
                        float actual_speed_rpm,
                        float dt_s)
{
    int64_t pos_err_cnt;
    float speed_ref_rpm;
    float pos_ref_turns;

    if ((m == 0) || (enc == 0))
        return;

    if (!m->active)
        return;

    if (dt_s <= 0.0f)
        dt_s = MOTION_CTRL_DT;

    pos_err_cnt = m->target_cnt - enc->pos_cnt_total;
    m->last_err_cnt = pos_err_cnt;
    m->last_err_turns = (float)pos_err_cnt / MOTION_COUNTS_PER_REV;

    // ---------- 进入末端精定位 ----------
    if (m->capture_enable &&
        (m->state == MOTION_RUN_RUNNING) &&
        (mt_llabs64(pos_err_cnt) <= (int64_t)m->capture_enter_cnt))
    {
        m->state = MOTION_RUN_CAPTURE;
    }

    // ---------- 精定位阶段 ----------
    if (m->state == MOTION_RUN_CAPTURE)
    {
        float speed_cmd;

        // 位置误差(圈) -> 小速度命令(rpm)
        speed_cmd = m->capture_kp_rpm_per_turn * m->last_err_turns;
        speed_cmd = mt_clampf(speed_cmd, -m->capture_max_rpm, m->capture_max_rpm);

        m->speed_ref_rpm = speed_cmd;
        connect_crt.speed = speed_cmd;

        // 完成条件：进窗口 + 速度足够小
        if ((mt_llabs64(pos_err_cnt) <= (int64_t)m->done_window_cnt) &&
            (mt_absf(actual_speed_rpm) <= m->done_speed_rpm))
        {
            mt_finalize_done(m);
            return;
        }

        return;
    }

    // ---------- 正常轨迹阶段 ----------
    m->elapsed_s += dt_s;

    if (m->elapsed_s >= m->t3)
    {
        m->elapsed_s = m->t3;
        m->pos_ref_turns = m->cmd_turns;
        m->speed_ref_rpm = 0.0f;
        m->reached_time = 1;

        // 时间到了，如果还没进精定位，则强制进入
        if (m->capture_enable)
        {
            m->state = MOTION_RUN_CAPTURE;
            return;
        }
        else
        {
            connect_crt.speed = 0.0f;
            speed_pi.target = 0.0f;
            mt_finalize_done(m);
            return;
        }
    }

    speed_ref_rpm = mt_eval_speed_rpm(m, m->elapsed_s);
    pos_ref_turns = mt_eval_pos_turns(m, m->elapsed_s);

    m->speed_ref_rpm = speed_ref_rpm;
    m->pos_ref_turns = pos_ref_turns;

    connect_crt.speed = speed_ref_rpm;
}

uint8_t MotionTrap2_IsBusy(const motion_trap2_t *m)
{
    if (m == 0) return 0;
    return m->active;
}

uint8_t MotionTrap2_IsDone(const motion_trap2_t *m)
{
    if (m == 0) return 0;
    return (m->state == MOTION_RUN_DONE) ? 1 : 0;
}

float MotionTrap2_GetTargetSpeedRpm(const motion_trap2_t *m)
{
    if (m == 0) return 0.0f;
    return m->speed_ref_rpm;
}

int64_t MotionTrap2_GetTargetCount(const motion_trap2_t *m)
{
    if (m == 0) return 0;
    return m->target_cnt;
}