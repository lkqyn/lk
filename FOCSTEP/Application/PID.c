#include "PID.h"

static float PID_Limit(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

void PID_Init(PID_Controller_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->ref = 0.0f;
    pid->fbk = 0.0f;
    pid->error = 0.0f;
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;
    pid->out_min = -1.0f;
    pid->out_max = 1.0f;
    pid->integral_min = -1.0f;
    pid->integral_max = 1.0f;
}

void PID_Reset(PID_Controller_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->ref = 0.0f;
    pid->fbk = 0.0f;
    pid->error = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;
}

void PID_SetGains(PID_Controller_t *pid, float kp, float ki)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
}

void PID_SetOutputLimit(PID_Controller_t *pid, float out_min, float out_max)
{
    float temp;

    if (pid == 0)
    {
        return;
    }

    if (out_min > out_max)
    {
        temp = out_min;
        out_min = out_max;
        out_max = temp;
    }

    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->out = PID_Limit(pid->out, out_min, out_max);
}

void PID_SetIntegralLimit(PID_Controller_t *pid, float integral_min, float integral_max)
{
    float temp;

    if (pid == 0)
    {
        return;
    }

    if (integral_min > integral_max)
    {
        temp = integral_min;
        integral_min = integral_max;
        integral_max = temp;
    }

    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->integral = PID_Limit(pid->integral, integral_min, integral_max);
}

float PID_Update(PID_Controller_t *pid, float ref, float fbk, float dt_s)
{
    float proportional;
    float unsat_out;

    if (pid == 0)
    {
        return 0.0f;
    }

    if (dt_s < 0.0f)
    {
        dt_s = 0.0f;
    }

    pid->ref = ref;
    pid->fbk = fbk;
    pid->error = ref - fbk;

    pid->integral += pid->error * pid->ki * dt_s;
    pid->integral = PID_Limit(pid->integral, pid->integral_min, pid->integral_max);

    proportional = pid->kp * pid->error;
    unsat_out = proportional + pid->integral;
    pid->out = PID_Limit(unsat_out, pid->out_min, pid->out_max);

    if (unsat_out != pid->out)
    {
        pid->integral = pid->out - proportional;
        pid->integral = PID_Limit(pid->integral, pid->integral_min, pid->integral_max);
    }

    return pid->out;
}
