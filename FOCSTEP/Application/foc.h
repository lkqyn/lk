#ifndef __FOC_H__
#define __FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float ia;
    float ib;
    float electrical_angle;
    float id;
    float iq;
    float id_ref;
    float iq_ref;
    float id_error;
    float iq_error;
    float id_integral;
    float iq_integral;
    float kp;
    float ki;
    float voltage_limit;
    float feedforward_speed_rpm;
    float feedforward_scale;
    float psi_wb;
    float ld_h;
    float lq_h;
    float ud_ff;
    float uq_ff;
    float ud;
    float uq;
} FOC_State_t;

void FOC_Init(void);
void FOC_UpdateCurrent(float ia, float ib, float electrical_angle);
void FOC_SetCurrentLoopGains(float kp, float ki);
void FOC_SetCurrentLoopBandwidth(float bandwidth_hz);
void FOC_SetCurrentLoopLimit(float voltage_limit);
void FOC_SetMotorModel(float psi_wb, float ld_h, float lq_h);
void FOC_SetFeedforwardSpeed(float speed_rpm);
void FOC_SetFeedforwardScale(float scale);
void FOC_SetCurrentTarget(float id_ref, float iq_ref);
void FOC_ResetCurrentLoop(void);
void FOC_RunCurrentLoop(float dt_s);
const FOC_State_t *FOC_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
