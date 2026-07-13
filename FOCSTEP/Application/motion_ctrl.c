#include "motion_ctrl.h"

#define MOTION_CTRL_PI                 (3.14159265358979323846f)
#define MOTION_CTRL_TWO_PI             (2.0f * MOTION_CTRL_PI)
#define MOTION_CTRL_DEFAULT_UQ         (0.0f)
#define MOTION_CTRL_DEFAULT_FREQ_HZ    (0.0f)
#define MOTION_CTRL_DEFAULT_ACCEL_RPM_S (1000.0f)

static MotionCtrl_Mode_t s_motion_mode;
static MotionCtrl_OpenLoopConfig_t s_motion_open_loop;
static MotionCtrl_PositionPState_t s_motion_position_p;
static MotionCtrl_SpeedPIState_t s_motion_speed_pi;
static MotionCtrl_Output_t s_motion_output;

static float MotionCtrl_NormalizeAngle(float angle_rad)
{
    while (angle_rad >= MOTION_CTRL_TWO_PI)
    {
        angle_rad -= MOTION_CTRL_TWO_PI;
    }

    while (angle_rad < 0.0f)
    {
        angle_rad += MOTION_CTRL_TWO_PI;
    }

    return angle_rad;
}

static float MotionCtrl_LimitUq(float uq)
{
    if (uq > 1.0f)
    {
        return 1.0f;
    }

    if (uq < -1.0f)
    {
        return -1.0f;
    }

    return uq;
}

static float MotionCtrl_LimitAbs(float value, float limit)
{
    float abs_limit = limit;

    if (abs_limit < 0.0f)
    {
        abs_limit = -abs_limit;
    }

    if (value > abs_limit)
    {
        return abs_limit;
    }

    if (value < -abs_limit)
    {
        return -abs_limit;
    }

    return value;
}

static float MotionCtrl_RampToward(float current, float target, float step)
{
    float delta = target - current;
    float abs_step = step;

    if (abs_step < 0.0f)
    {
        abs_step = -abs_step;
    }

    if (delta > abs_step)
    {
        return current + abs_step;
    }

    if (delta < -abs_step)
    {
        return current - abs_step;
    }

    return target;
}

static float MotionCtrl_GetEncoderFocAngle(const Encoder_State_t *encoder)
{
    return encoder->electrical_angle_rad;
}

void MotionCtrl_Init(void)
{
    s_motion_mode = MOTION_CTRL_MODE_OPEN_LOOP;
    PID_Init(&s_motion_speed_pi.pid);
    MotionCtrl_SetOpenLoop(MOTION_CTRL_DEFAULT_UQ, MOTION_CTRL_DEFAULT_FREQ_HZ);
    MotionCtrl_SetPositionP(0.0f, 0.0f, 0.0f);
    MotionCtrl_SetSpeedPI(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    s_motion_output.uq = 0.0f;
    s_motion_output.angle_rad = 0.0f;
}

void MotionCtrl_Update(float dt_s, float *open_loop_angle, const Encoder_State_t *encoder)
{
    if ((open_loop_angle == 0) || (encoder == 0))
    {
        return;
    }

    if (s_motion_mode == MOTION_CTRL_MODE_OPEN_LOOP)
    {
        *open_loop_angle = MotionCtrl_NormalizeAngle(
            *open_loop_angle + (MOTION_CTRL_TWO_PI * s_motion_open_loop.elec_freq_hz * dt_s));

        s_motion_output.uq = s_motion_open_loop.uq;
        s_motion_output.angle_rad = *open_loop_angle;
    }
    else if (s_motion_mode == MOTION_CTRL_MODE_ENCODER_ANGLE)
    {
        s_motion_output.uq = s_motion_open_loop.uq;
        s_motion_output.angle_rad = MotionCtrl_GetEncoderFocAngle(encoder);
        *open_loop_angle = s_motion_output.angle_rad;
    }
    else if (s_motion_mode == MOTION_CTRL_MODE_POSITION_P)
    {
        s_motion_position_p.target_count += s_motion_position_p.speed_cps * dt_s;
        s_motion_position_p.error_count = s_motion_position_p.target_count - (float)encoder->relative_count;
        s_motion_position_p.output_uq = MotionCtrl_LimitAbs(
            -s_motion_position_p.kp * s_motion_position_p.error_count,
            s_motion_position_p.max_uq);

        s_motion_output.uq = s_motion_position_p.output_uq;
        s_motion_output.angle_rad = MotionCtrl_GetEncoderFocAngle(encoder);
        *open_loop_angle = s_motion_output.angle_rad;
    }
    else
    {
        s_motion_speed_pi.target_rpm =
            MotionCtrl_RampToward(s_motion_speed_pi.target_rpm,
                                  s_motion_speed_pi.command_rpm,
                                  s_motion_speed_pi.accel_rpm_s * dt_s);
        s_motion_speed_pi.last_count = encoder->relative_count;
        s_motion_speed_pi.actual_rpm = encoder->speed_lpf_rpm;
        s_motion_speed_pi.error_rpm = s_motion_speed_pi.target_rpm - s_motion_speed_pi.actual_rpm;
        s_motion_speed_pi.output_iq = PID_Update(&s_motion_speed_pi.pid,
                                                 s_motion_speed_pi.target_rpm,
                                                 s_motion_speed_pi.actual_rpm,
                                                 dt_s);

        s_motion_output.uq = s_motion_speed_pi.output_iq;
        s_motion_output.angle_rad = MotionCtrl_GetEncoderFocAngle(encoder);
        *open_loop_angle = s_motion_output.angle_rad;
    }
}

void MotionCtrl_SetOpenLoop(float uq, float elec_freq_hz)
{
    s_motion_open_loop.uq = MotionCtrl_LimitUq(uq);
    s_motion_open_loop.elec_freq_hz = elec_freq_hz;
}

void MotionCtrl_SetPositionP(float speed_cps, float kp, float max_uq)
{
    const Encoder_State_t *encoder = Encoder_GetState();

    s_motion_position_p.target_count = (float)encoder->relative_count;
    s_motion_position_p.error_count = 0.0f;
    s_motion_position_p.output_uq = 0.0f;
    s_motion_position_p.speed_cps = speed_cps;
    s_motion_position_p.kp = kp;
    s_motion_position_p.max_uq = MotionCtrl_LimitUq(max_uq);
}

void MotionCtrl_SetSpeedPI(float target_rpm, float kp, float ki, float max_iq, float max_integral_iq)
{
    const Encoder_State_t *encoder = Encoder_GetState();
    float integral_limit;

    s_motion_speed_pi.command_rpm = target_rpm;
    s_motion_speed_pi.target_rpm = encoder->speed_lpf_rpm;
    s_motion_speed_pi.actual_rpm = 0.0f;
    s_motion_speed_pi.error_rpm = 0.0f;
    s_motion_speed_pi.accel_rpm_s = MOTION_CTRL_DEFAULT_ACCEL_RPM_S;
    s_motion_speed_pi.max_iq = MotionCtrl_LimitUq(max_iq);
    integral_limit = MotionCtrl_LimitAbs(max_integral_iq, s_motion_speed_pi.max_iq);
    s_motion_speed_pi.max_integral_iq = integral_limit;
    s_motion_speed_pi.output_iq = 0.0f;
    s_motion_speed_pi.last_count = encoder->relative_count;
    PID_Reset(&s_motion_speed_pi.pid);
    PID_SetGains(&s_motion_speed_pi.pid, kp, ki);
    PID_SetOutputLimit(&s_motion_speed_pi.pid,
                       -s_motion_speed_pi.max_iq,
                       s_motion_speed_pi.max_iq);
    PID_SetIntegralLimit(&s_motion_speed_pi.pid,
                         -s_motion_speed_pi.max_integral_iq,
                         s_motion_speed_pi.max_integral_iq);
}

void MotionCtrl_SetMode(MotionCtrl_Mode_t mode)
{
    if ((mode != MOTION_CTRL_MODE_OPEN_LOOP) &&
        (mode != MOTION_CTRL_MODE_ENCODER_ANGLE) &&
        (mode != MOTION_CTRL_MODE_POSITION_P) &&
        (mode != MOTION_CTRL_MODE_SPEED_PI))
    {
        return;
    }

    s_motion_mode = mode;
}

MotionCtrl_Mode_t MotionCtrl_GetMode(void)
{
    return s_motion_mode;
}

const MotionCtrl_OpenLoopConfig_t *MotionCtrl_GetOpenLoopConfig(void)
{
    return &s_motion_open_loop;
}

const MotionCtrl_PositionPState_t *MotionCtrl_GetPositionPState(void)
{
    return &s_motion_position_p;
}

const MotionCtrl_SpeedPIState_t *MotionCtrl_GetSpeedPIState(void)
{
    return &s_motion_speed_pi;
}

const MotionCtrl_Output_t *MotionCtrl_GetOutput(void)
{
    return &s_motion_output;
}
