#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#include <stdint.h>

typedef enum {
  MOTOR_MODE_STEPPER_GPIO = 0,
} motor_mode_t;

typedef enum { HB_AP=0, HB_AN=1, HB_BP=2, HB_BN=3 } hb_id_t;
typedef enum { HB_OFF=0, HB_LOW=1, HB_HIGH=2 } hb_state_t;

void Motor_InitStepperGPIO(void);
void Motor_SetMode(motor_mode_t mode);

void HB_Set(hb_id_t id, hb_state_t s);
void Motor_Test_AP_Waveform(uint8_t pwm_percent);
#endif
