#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float ref;
    float fbk;
    float error;
    float kp;
    float ki;
    float integral;
    float out;
    float out_min;
    float out_max;
    float integral_min;
    float integral_max;
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid);
void PID_Reset(PID_Controller_t *pid);
void PID_SetGains(PID_Controller_t *pid, float kp, float ki);
void PID_SetOutputLimit(PID_Controller_t *pid, float out_min, float out_max);
void PID_SetIntegralLimit(PID_Controller_t *pid, float integral_min, float integral_max);
float PID_Update(PID_Controller_t *pid, float ref, float fbk, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
