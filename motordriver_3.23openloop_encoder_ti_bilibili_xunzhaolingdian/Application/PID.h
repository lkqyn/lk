#ifndef __PID_H
#define __PID_H

#include "main.h"
#include <stdint.h>

// ================= PID控制器 =================
typedef struct
{
    float kp;
    float ki;
    float kd;

    float target;//目标值
    float actual_value;//测量值

    float integral;
    float last_err;

    float output_min;
    float output_max;
} PID_Controller_t;

// ================= 角度控制结构体 =================
typedef struct
{
    PID_Controller_t pid_angle;
    float zero_position;   // 零点位置，单位：度
    float Iq_ref;          // 角度环输出（后续可作为速度/电流参考）
} AngleControl_TypeDef;

// ================= 外部变量 =================
extern PID_Controller_t speed_pi;   // 速度环PID

extern uint16_t mt6816_count;       // 编码器当前计数值
extern uint16_t encoder_last_count;
extern int32_t encoder_count_accum;

extern AngleControl_TypeDef AngleControl;

// MT6816单圈分辨率
#define MT6816_PPR 16384

// 如果工程里没有这个宏，你可以保留这个默认限幅
#ifndef System_MAX_I
#define System_MAX_I 20.0f
#endif

// ================= PID接口 =================
float PID_Controller_Update(PID_Controller_t *pi, float error);
void PID_Controller_Init(PID_Controller_t *pi, float kp, float ki, float kd,
                         float target, float out_min, float out_max);

// ================= 编码器速度接口 =================
int16_t Encoder_GetDiff(uint16_t current_count, uint16_t last_count);
// 读取编码器计数，计算累计计数（处理溢出）
void Encoder_Update(void);
// 计算速度（单位rpm），周期为 speed_loop_period 秒
float Calculate_Speed(float speed_loop_period);

// ================= 角度环接口 =================
float Angle_Normalize(float angle);
float Angle_Error(float target, float current);
void AngleControl_SetTargetAngle(AngleControl_TypeDef* ctrl, float target_deg);
void AngleControl_SetZero(AngleControl_TypeDef* ctrl);
void AngleControl_Update(AngleControl_TypeDef* ctrl);

#endif