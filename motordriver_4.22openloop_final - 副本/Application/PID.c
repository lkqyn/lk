#include "pid.h"

PID_Controller_t speed_pi;

// PID控制
float PID_Controller_Update(PID_Controller_t *pi, float error)
{
    pi->integral += error * pi->ki;

    if (pi->integral > pi->output_max)
        pi->integral = pi->output_max;
    else if (pi->integral < pi->output_min)
        pi->integral = pi->output_min;

    float output = pi->kp * error
                 + pi->integral
                 + pi->kd * (error - pi->last_err);

    if (output > pi->output_max)
        output = pi->output_max;
    else if (output < pi->output_min)
        output = pi->output_min;

    pi->last_err = error;
    return output;
}

// PID参数初始化
void PID_Controller_Init(PID_Controller_t *pi,
                         float kp,
                         float ki,
                         float kd,
                         float target,
                         float out_min,
                         float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->kd = kd;

    pi->target = target;
    pi->actual_value = 0.0f;

    pi->integral = 0.0f;
    pi->last_err = 0.0f;

    pi->output_min = out_min;
    pi->output_max = out_max;
}

// 清零PID状态
void PID_Controller_Reset(PID_Controller_t *pi)
{
    pi->integral = 0.0f;
    pi->last_err = 0.0f;
}