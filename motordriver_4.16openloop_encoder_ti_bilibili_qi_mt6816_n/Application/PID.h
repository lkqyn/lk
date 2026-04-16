#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FOC.h"

/* ====================== 速度环PID结构体 ====================== */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float actual_value;   // 测量值
    float target;         // 目标值
    float output_max;
    float output_min;
    float last_err;
} PID_Controller_t;

/* ====================== 绝对角度闭环控制结构体 ====================== */
typedef struct AngleControl_TypeDef {
    float zero_position;      // 零点编码器角度，单位度
    PID_Controller_t pid_angle;
    float Iq_ref;             // 输出Iq参考电流
} AngleControl_TypeDef;

/* ====================== 编码器累计角度处理结构体（单位：度） ====================== */
typedef struct {
    float angle_prev;         // 上次角度 0~360
    int32_t round_count;      // 圈数计数
    float angle_accum;        // 累计角度 = angle_now + 360*round_count
    uint8_t initialized;
    float start_angle;        // 每次位置环开始时的位置
    float erro;               // 累计误差补偿
    float target_angle_acc;   // 目标累计角度
} EncoderAccumulator_TypeDef;

/* ====================== 全局变量 ====================== */
extern AngleControl_TypeDef AngleControl;      // 角度控制结构体
extern PID_Controller_t speed_pi;              // 速度环pid
extern EncoderAccumulator_TypeDef encoder_acc; // 累计角度结构体

/* ====================== PID接口 ====================== */
float PID_Controller_Update(PID_Controller_t *pi, float error);
void PID_Controller_Init(PID_Controller_t *pi, float kp, float ki, float kd, float target, float out_min, float out_max);

/* ====================== 速度环 / 编码器接口 ====================== */
extern uint16_t mt6816_count;
extern uint16_t encoder_last_count;

// 读取编码器计数，计算累计计数（处理溢出）
void Encoder_Update(void);

// 计算速度（单位rpm），周期为 speed_loop_period 秒
float Calculate_Speed(float speed_loop_period);

/* ====================== 角度环接口 ====================== */
// 设置目标绝对角度，单位度，归一化到[0,360)
void AngleControl_SetTargetAngle(AngleControl_TypeDef* ctrl, float target_deg);

// 设置零点（记录当前编码器角度作为零点）
void AngleControl_SetZero(AngleControl_TypeDef* ctrl);

// 角度控制环
void AngleControl_Update(AngleControl_TypeDef* ctrl);

/* ====================== 可继续补的位置环接口（可选） ====================== */
// 后面如果你要把参考代码的位置累计逻辑补上，可以继续在这里加声明
// 例如：
// void EncoderAccumulator_Init(EncoderAccumulator_TypeDef *acc);
// void EncoderAccumulator_Update(EncoderAccumulator_TypeDef *acc, float current_angle);

#ifdef __cplusplus
}
#endif

#endif