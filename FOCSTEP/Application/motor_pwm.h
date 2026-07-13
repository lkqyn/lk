#ifndef __MOTOR_PWM_H__
#define __MOTOR_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define MOTOR_PWM_TIMER_ARR        (4199U)
#define MOTOR_PWM_MAX_DUTY         (MOTOR_PWM_TIMER_ARR)

typedef struct
{
    uint16_t ap;
    uint16_t an;
    uint16_t bp;
    uint16_t bn;
} MotorPWM_Duty_t;

void MotorPWM_Init(void);
void MotorPWM_Enable(void);
void MotorPWM_Disable(void);
void MotorPWM_SetRaw(uint16_t ap, uint16_t an, uint16_t bp, uint16_t bn);
void MotorPWM_SetPhaseVoltage(float ua, float ub);
MotorPWM_Duty_t MotorPWM_GetDuty(void);

#ifdef __cplusplus
}
#endif

#endif
