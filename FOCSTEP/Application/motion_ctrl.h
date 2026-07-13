#ifndef __MOTION_CTRL_H__
#define __MOTION_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "encoder.h"
#include "PID.h"

typedef enum
{
    MOTION_CTRL_MODE_OPEN_LOOP = 0,
    MOTION_CTRL_MODE_ENCODER_ANGLE,
    MOTION_CTRL_MODE_POSITION_P,
    MOTION_CTRL_MODE_SPEED_PI
} MotionCtrl_Mode_t;

typedef struct
{
    float uq;
    float elec_freq_hz;
} MotionCtrl_OpenLoopConfig_t;

typedef struct
{
    float target_count;
    float error_count;
    float output_uq;
    float speed_cps;
    float kp;
    float max_uq;
} MotionCtrl_PositionPState_t;

typedef struct
{
    float command_rpm;
    float target_rpm;
    float actual_rpm;
    float error_rpm;
    float accel_rpm_s;
    float max_iq;
    float max_integral_iq;
    float output_iq;
    int32_t last_count;
    PID_Controller_t pid;
} MotionCtrl_SpeedPIState_t;

typedef struct
{
    float uq;
    float angle_rad;
} MotionCtrl_Output_t;

void MotionCtrl_Init(void);
void MotionCtrl_Update(float dt_s, float *open_loop_angle, const Encoder_State_t *encoder);
void MotionCtrl_SetOpenLoop(float uq, float elec_freq_hz);
void MotionCtrl_SetPositionP(float speed_cps, float kp, float max_uq);
void MotionCtrl_SetSpeedPI(float target_rpm, float kp, float ki, float max_iq, float max_integral_iq);
void MotionCtrl_SetMode(MotionCtrl_Mode_t mode);
MotionCtrl_Mode_t MotionCtrl_GetMode(void);
const MotionCtrl_OpenLoopConfig_t *MotionCtrl_GetOpenLoopConfig(void);
const MotionCtrl_PositionPState_t *MotionCtrl_GetPositionPState(void);
const MotionCtrl_SpeedPIState_t *MotionCtrl_GetSpeedPIState(void);
const MotionCtrl_Output_t *MotionCtrl_GetOutput(void);

#ifdef __cplusplus
}
#endif

#endif
