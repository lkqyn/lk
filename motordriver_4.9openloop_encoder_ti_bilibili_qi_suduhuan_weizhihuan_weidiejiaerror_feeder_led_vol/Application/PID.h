#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;

    float target;//目标值
    float actual_value;//真实值

    float integral;
    float last_err;

    float output_min;
    float output_max;
} PID_Controller_t;

extern PID_Controller_t speed_pi;

float PID_Controller_Update(PID_Controller_t *pi, float error);

void PID_Controller_Init(PID_Controller_t *pi,
                         float kp,
                         float ki,
                         float kd,
                         float target,
                         float out_min,
                         float out_max);

void PID_Controller_Reset(PID_Controller_t *pi);

#ifdef __cplusplus
}
#endif

#endif