#include "pid.h"
#include "mt6816.h"
#include "math.h"

// ================= 全局变量 =================
PID_Controller_t speed_pi;   // 速度环pid

uint16_t mt6816_count = 0;       // 编码器计数值
uint16_t encoder_last_count = 0;
int32_t encoder_count_accum = 0;

AngleControl_TypeDef AngleControl;

extern int DIR;

// ================= PID更新 =================
float PID_Controller_Update(PID_Controller_t *pi, float error)
{
    pi->integral += error * pi->ki;

    // 反积分限幅
    if (pi->integral > pi->output_max)
        pi->integral = pi->output_max;
    else if (pi->integral < pi->output_min)
        pi->integral = pi->output_min;

    float output = pi->kp * error
                 + pi->integral
                 + pi->kd * (error - pi->last_err);

    // 输出限幅
    if (output > pi->output_max)
        output = pi->output_max;
    else if (output < pi->output_min)
        output = pi->output_min;

    pi->last_err = error;
    return output;
}

// ================= PID初始化 =================
void PID_Controller_Init(PID_Controller_t *pi, float kp, float ki, float kd,
                         float target, float out_min, float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->kd = kd;

    pi->target = target;
    pi->integral = 0.0f;
    pi->last_err = 0.0f;

    if (out_max > System_MAX_I)
        out_max = System_MAX_I;
    if (out_min < -System_MAX_I)
        out_min = -System_MAX_I;

    pi->output_min = out_min;
    pi->output_max = out_max;
}

// ================= 编码器差值 =================
// 计算编码器差值，考虑0~16383环绕
int16_t Encoder_GetDiff(uint16_t current_count, uint16_t last_count)
{
    int16_t diff = (int16_t)current_count - (int16_t)last_count;

    if (diff > (MT6816_PPR / 2))
        diff -= MT6816_PPR;
    else if (diff < -(MT6816_PPR / 2))
        diff += MT6816_PPR;

    return (int16_t)(diff * DIR);
}

// ================= 编码器累计更新 =================
void Encoder_Update(void)
{
    int16_t diff = Encoder_GetDiff(mt6816_count, encoder_last_count);
    encoder_count_accum += diff;
    encoder_last_count = mt6816_count;
}

// ================= 计算速度 =================
// speed_loop_period: 速度环执行周期，单位秒
// 返回值：rpm
float Calculate_Speed(float speed_loop_period)
{
    float revolutions = (float)encoder_count_accum / MT6816_PPR;
    encoder_count_accum = 0;  // 清零累计计数

    float speed_rpm = (revolutions / speed_loop_period) * 60.0f;
    return speed_rpm;
}

// ================= 角度归一化到[0,360) =================
float Angle_Normalize(float angle)
{
    while (angle < 0.0f)   angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

// ================= 计算角度误差，范围[-180,180] =================
float Angle_Error(float target, float current)
{
    float diff = target - current;

    if (diff > 180.0f)
        diff -= 360.0f;
    else if (diff < -180.0f)
        diff += 360.0f;

    return diff;
}

// ================= 设置目标绝对角度 =================
void AngleControl_SetTargetAngle(AngleControl_TypeDef* ctrl, float target_deg)
{
    ctrl->pid_angle.target = Angle_Normalize(target_deg);
}

// ================= 设置零点 =================
void AngleControl_SetZero(AngleControl_TypeDef* ctrl)
{
    // 16384 count -> 360 deg
    ctrl->zero_position = DIR * mt6816_count * (360.0f / 16384.0f);
    ctrl->pid_angle.target = 0.0f;
}

// ================= 角度环更新 =================
void AngleControl_Update(AngleControl_TypeDef* ctrl)
{
    float current_angle = DIR * mt6816_count * (360.0f / 16384.0f);

    // 当前相对零点角度
    float relative_angle = Angle_Normalize(current_angle - ctrl->zero_position);

    // 计算误差
    float error = Angle_Error(ctrl->pid_angle.target, relative_angle);

    // PID控制输出
    ctrl->Iq_ref = PID_Controller_Update(&ctrl->pid_angle, error);
}