#include "pid.h"
#include "mt6816.h"
#include "math.h"
#include "connecting.h"
#include "FOC.h"

PID_Controller_t speed_pi;
AngleControl_TypeDef AngleControl;
EncoderAccumulator_TypeDef encoder_acc;

uint16_t mt6816_count = 0;
uint16_t encoder_last_count = 0;
int32_t encoder_count_accum = 0;

#define MT6816_PPR 16384

float PID_Controller_Update(PID_Controller_t *pi, float error)
{
    float output;

    pi->integral += error * pi->ki;
    if(pi->integral > pi->output_max) pi->integral = pi->output_max;
    else if(pi->integral < pi->output_min) pi->integral = pi->output_min;

    output = pi->kp * error + pi->integral + pi->kd * (error - pi->last_err);
    if(output > pi->output_max) output = pi->output_max;
    else if(output < pi->output_min) output = pi->output_min;

    pi->last_err = error;
    return output;
}

void PID_Controller_Init(PID_Controller_t *pi, float kp, float ki, float kd, float target, float out_min, float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->kd = kd;
    pi->target = target;
    pi->actual_value = 0.0f;
    pi->integral = 0.0f;
    pi->last_err = 0.0f;
    pi->erro = 0.0f;

    if(out_max > System_MAX_I) out_max = System_MAX_I;
    if(out_min < -System_MAX_I) out_min = -System_MAX_I;

    pi->output_min = out_min;
    pi->output_max = out_max;
}

static int16_t Encoder_GetDiff(uint16_t current_count, uint16_t last_count)
{
    int16_t diff = (int16_t)current_count - (int16_t)last_count;

    if(diff > (MT6816_PPR / 2)) diff -= MT6816_PPR;
    else if(diff < -(MT6816_PPR / 2)) diff += MT6816_PPR;

    return (int16_t)(diff * DIR);
}

void Encoder_Update(void)
{
    int16_t diff = Encoder_GetDiff(mt6816_count, encoder_last_count);
    encoder_count_accum += diff;
    encoder_last_count = mt6816_count;
}

void init_encoder_update(void)
{
    encoder_last_count = mt6816_count;
    encoder_count_accum = 0;
}

float Calculate_Speed(float speed_loop_period)
{
    float revolutions = (float)encoder_count_accum / MT6816_PPR;
    encoder_count_accum = 0;
    return (revolutions / speed_loop_period) * 60.0f;
}

static float Angle_Normalize(float angle)
{
    while(angle < 0.0f) angle += 360.0f;
    while(angle >= 360.0f) angle -= 360.0f;
    return angle;
}

static float Angle_Error(float target, float current)
{
    float diff = target - current;
    if(diff > 180.0f) diff -= 360.0f;
    else if(diff < -180.0f) diff += 360.0f;
    return diff;
}

void AngleControl_SetTargetAngle(AngleControl_TypeDef* ctrl, float target_deg)
{
    ctrl->pid_angle.target = Angle_Normalize(target_deg);
}

void AngleControl_SetZero(AngleControl_TypeDef* ctrl)
{
    ctrl->zero_position = DIR * mt6816_count * 0.02197265625f;
    ctrl->pid_angle.target = 0.0f;
}

void AngleControl_Update(AngleControl_TypeDef* ctrl)
{
    float current_angle = DIR * mt6816_count * 0.02197265625f;
    float relative_angle = Angle_Normalize(current_angle - ctrl->zero_position);
    float error = Angle_Error(ctrl->pid_angle.target, relative_angle);

    if(error > 60.0f) error = 60.0f;
    else if(error < -60.0f) error = -60.0f;

    ctrl->Iq_ref = PID_Controller_Update(&ctrl->pid_angle, error);
}
