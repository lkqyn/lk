#ifndef __APP_H__
#define __APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motion_ctrl.h"

typedef MotionCtrl_OpenLoopConfig_t App_OpenLoopConfig_t;
typedef MotionCtrl_Mode_t App_RunMode_t;
typedef MotionCtrl_PositionPState_t App_PositionPState_t;
typedef MotionCtrl_SpeedPIState_t App_SpeedPIState_t;

void App_Init(void);
void App_Loop(void);
void App_FastLoop(void);
void App_ClearFault(void);
uint8_t App_IsFaulted(void);
void App_SetOpenLoop(float uq, float elec_freq_hz);
void App_SetEncoderVoltage(float uq_volt, float lead_deg);
void App_ClearEncoderVoltage(void);
void App_SetPositionP(float speed_cps, float kp, float max_uq);
void App_SetSpeedPI(float target_rpm, float kp, float ki, float max_iq, float max_integral_iq);
void App_SetSpeedIdRef(float id_ref);
void App_SetAngleOffsetDeg(float offset_deg);
float App_GetAngleOffsetDeg(void);
void App_SetVoltageLeadDeg(float lead_deg);
float App_GetVoltageLeadDeg(void);
void App_SetCurrentFeedforward(uint8_t enabled);
uint8_t App_GetCurrentFeedforward(void);
void App_SetRunMode(App_RunMode_t mode);
void App_SetVoltageLock(float angle_rad, float ud, float uq);
void App_ClearVoltageLock(void);
void App_SetCurrentLoop(float id_ref, float iq_ref);
void App_SetCurrentLoopGains(float kp, float ki, float voltage_limit);
void App_SetCurrentLoopBandwidth(float bandwidth_hz, float voltage_limit);
void App_ClearCurrentLoop(void);
const App_OpenLoopConfig_t *App_GetOpenLoopConfig(void);
const App_PositionPState_t *App_GetPositionPState(void);
const App_SpeedPIState_t *App_GetSpeedPIState(void);
App_RunMode_t App_GetRunMode(void);

#define APP_RUN_MODE_OPEN_LOOP         MOTION_CTRL_MODE_OPEN_LOOP
#define APP_RUN_MODE_ENCODER_ANGLE     MOTION_CTRL_MODE_ENCODER_ANGLE
#define APP_RUN_MODE_POSITION_P        MOTION_CTRL_MODE_POSITION_P
#define APP_RUN_MODE_SPEED_PI          MOTION_CTRL_MODE_SPEED_PI

#ifdef __cplusplus
}
#endif

#endif
