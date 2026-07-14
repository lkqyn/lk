#include "stepper_driver.h"
#include "motor_driver.h"
#include "main.h"

// 半步八拍：A/B 相方向（+1/-1/0）
typedef struct 
{ 
  int8_t A; 
  int8_t B; 
} StepAB;

static const StepAB seq8[8] = {
  {+1,  0},
  {+1, +1},
  { 0, +1},
  {-1, +1},
  {-1,  0},
  {-1, -1},
  { 0, -1},
  {+1, -1},
};

static uint8_t  idx = 0;
static uint8_t  dir = 0;
static uint16_t step_ms = 80;  //  5A 电机+小限流，建议先慢

static void apply_A(int8_t a)
{
  if (a > 0) {      // A+ : AP=HIGH, AN=LOW
    HB_Set(HB_AP, HB_HIGH);
    HB_Set(HB_AN, HB_LOW);
  } else if (a < 0) { // A- : AP=LOW, AN=HIGH
    HB_Set(HB_AP, HB_LOW);
    HB_Set(HB_AN, HB_HIGH);
  } else {          // A0 : 两端OFF
    HB_Set(HB_AP, HB_OFF);
    HB_Set(HB_AN, HB_OFF);
  }
}

static void apply_B(int8_t b)
{
  if (b > 0) {      // B+ : BP=HIGH, BN=LOW
    HB_Set(HB_BP, HB_HIGH);
    HB_Set(HB_BN, HB_LOW);
  } else if (b < 0) { // B- : BP=LOW, BN=HIGH
    HB_Set(HB_BP, HB_LOW);
    HB_Set(HB_BN, HB_HIGH);
  } else {          // B0 : 两端OFF
    HB_Set(HB_BP, HB_OFF);
    HB_Set(HB_BN, HB_OFF);
  }
}

void Stepper_Init(void)
{
  idx = 0;
  dir = 0;
  step_ms = 80;
}

void Stepper_SetDir(uint8_t d) { dir = d ? 1 : 0; }
void Stepper_SetStepDelayMs(uint16_t ms)
{
  if (ms < 10) ms = 10;
  if (ms > 500) ms = 500;
  step_ms = ms;
}

void Stepper_StepOnce_Blocking(void)
{
  apply_A(seq8[idx].A);
  apply_B(seq8[idx].B);

  idx = dir ? ((idx + 7) & 7) : ((idx + 1) & 7);
  HAL_Delay(step_ms);
}

void Stepper_Stop(void)
{
  HB_Set(HB_AP, HB_OFF);
  HB_Set(HB_AN, HB_OFF);
  HB_Set(HB_BP, HB_OFF);
  HB_Set(HB_BN, HB_OFF);
}
