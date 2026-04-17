#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FOC.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float actual_value;
    float target;
    float output_max;
    float output_min;
    float last_err;
    float erro;
} PID_Controller_t;

typedef struct AngleControl_TypeDef {
    float zero_position;
    PID_Controller_t pid_angle;
    float Iq_ref;
} AngleControl_TypeDef;

typedef struct {
    float angle_prev;
    int32_t round_count;
    float angle_accum;
    uint8_t initialized;
    float start_angle;
    float erro;
    float target_angle_acc;
} EncoderAccumulator_TypeDef;

extern AngleControl_TypeDef AngleControl;
extern PID_Controller_t speed_pi;
extern EncoderAccumulator_TypeDef encoder_acc;

extern uint16_t mt6816_count;
extern uint16_t encoder_last_count;

float PID_Controller_Update(PID_Controller_t *pi, float error);
void PID_Controller_Init(PID_Controller_t *pi, float kp, float ki, float kd, float target, float out_min, float out_max);
void Encoder_Update(void);
void init_encoder_update(void);
float Calculate_Speed(float speed_loop_period);
void AngleControl_SetTargetAngle(AngleControl_TypeDef* ctrl, float target_deg);
void AngleControl_SetZero(AngleControl_TypeDef* ctrl);
void AngleControl_Update(AngleControl_TypeDef* ctrl);

#ifdef __cplusplus
}
#endif

#endif
