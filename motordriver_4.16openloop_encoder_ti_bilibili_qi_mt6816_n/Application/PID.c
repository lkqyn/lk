#include "pid.h"
#include "mt6816.h"
#include "math.h"
#include "usbd_cdc_if.h"
//#include "can.h"
#include "connecting.h"
#include "FOC.h"

PID_Controller_t speed_pi;                 // 速度环pid
AngleControl_TypeDef AngleControl;         // 角度控制结构体
EncoderAccumulator_TypeDef encoder_acc;    // 累计角度结构体

// 编码器变量
uint16_t mt6816_count = 0;     // 编码器计数值
uint16_t encoder_last_count = 0;
int32_t encoder_count_accum = 0;

extern uint8_t pos_finish_flag;
extern uint8_t connect_type;

// 编码器PPR
#define MT6816_PPR 16384

/// PI控制
float PID_Controller_Update(PID_Controller_t *pi, float error)
{
    pi->integral += error * pi->ki;

    // 反积分限幅
    if(pi->integral > pi->output_max)
        pi->integral = pi->output_max;
    else if(pi->integral < pi->output_min)
        pi->integral = pi->output_min;

    float output = pi->kp * error + pi->integral + pi->kd * (error - pi->last_err);

    // 输出限幅
    if(output > pi->output_max)
        output = pi->output_max;
    else if(output < pi->output_min)
        output = pi->output_min;

    pi->last_err = error;
    return output;
}

/// PI参数初始化
void PID_Controller_Init(PID_Controller_t *pi, float kp, float ki, float kd, float target, float out_min, float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->kd = kd;
    pi->target = target;
    pi->actual_value = 0.0f;
    pi->integral = 0.0f;
    pi->last_err = 0.0f;

    if(out_max > System_MAX_I)
        out_max = System_MAX_I;
    if(out_min < -System_MAX_I)
        out_min = -System_MAX_I;

    pi->output_min = out_min;
    pi->output_max = out_max;
}

// 计算编码器差值，考虑编码器计数范围为0~MT6816_PPR-1环绕
int16_t Encoder_GetDiff(uint16_t current_count, uint16_t last_count)
{
    int16_t diff = ((int16_t)current_count - (int16_t)last_count);

    // 如果差值大于半个计数周期，说明发生了环绕，需校正
    if(diff > (MT6816_PPR / 2))
        diff -= MT6816_PPR;
    else if(diff < -(MT6816_PPR / 2))
        diff += MT6816_PPR;

    return (int16_t)(diff * DIR);
}

// 读取编码器计数，计算累计计数（处理溢出）
void Encoder_Update(void)
{
    int16_t diff = Encoder_GetDiff(mt6816_count, encoder_last_count);
    encoder_count_accum += diff;
    encoder_last_count = mt6816_count;
}

// 计算速度（单位rpm），周期为 speed_loop_period 秒
float Calculate_Speed(float speed_loop_period)
{
    float revolutions = (float)encoder_count_accum / MT6816_PPR;
    encoder_count_accum = 0;  // 清零累计计数

    float speed_rpm = (revolutions / speed_loop_period) * 60.0f;
    return speed_rpm;
}

// 角度归一化到[0, 360)
float Angle_Normalize(float angle)
{
    while(angle < 0.0f)
        angle += 360.0f;
    while(angle >= 360.0f)
        angle -= 360.0f;
    return angle;
}

// 计算角度误差，范围在[-180, 180]
float Angle_Error(float target, float current)
{
    float diff = target - current;
    if(diff > 180.0f)
        diff -= 360.0f;
    else if(diff < -180.0f)
        diff += 360.0f;
    return diff;
}

// 设置目标绝对角度，单位度，自动转换并归一化到[0,360)
void AngleControl_SetTargetAngle(AngleControl_TypeDef* ctrl, float target_deg)
{
    ctrl->pid_angle.target = Angle_Normalize(target_deg);
}

// 设置零点（记录当前编码器角度作为零点）
void AngleControl_SetZero(AngleControl_TypeDef* ctrl)
{
    ctrl->zero_position = DIR * mt6816_count * 0.02197265625f;   // 360 / 16384
    ctrl->pid_angle.target = 0.0f;
}

// 角度控制环
void AngleControl_Update(AngleControl_TypeDef* ctrl)
{
    float current_angle = DIR * mt6816_count * 0.02197265625f;

    // 当前相对零点角度
    float relative_angle = Angle_Normalize(current_angle - ctrl->zero_position);

    // 计算误差
    float error = Angle_Error(ctrl->pid_angle.target, relative_angle);

    // PID控制计算Iq参考电流
    ctrl->Iq_ref = PID_Controller_Update(&ctrl->pid_angle, error);
}