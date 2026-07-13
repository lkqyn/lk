#ifndef __FOC_V_H__
#define __FOC_V_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float ud;
    float uq;
    float electrical_angle;
    float ua;
    float ub;
} FocV_State_t;

void FocV_Init(void);
void FocV_Stop(void);
void FocV_SetVoltage(float ud, float uq);
void FocV_SetElectricalAngle(float angle_rad);
void FocV_Run(void);
void FocV_RunUq(float uq, float angle_rad);
const FocV_State_t *FocV_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
