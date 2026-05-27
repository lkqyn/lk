#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include <stdint.h>

void AppMotor_Init(void);
void AppMotor_StartRewind(void);
void AppMotor_StopRewind(void);
void AppMotor_RewindTask(void);
uint8_t AppMotor_IsRewindAlarm(void);
uint8_t AppMotor_IsRewindRunning(void);
uint8_t AppMotor_IsDacReady(void);
uint8_t AppMotor_GetDacInitOk(void);
uint32_t AppMotor_GetStepCount(void);
void AppMotor_ForceStepTest(void);

#endif
