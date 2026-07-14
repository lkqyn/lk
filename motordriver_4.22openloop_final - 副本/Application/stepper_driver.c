#include "stepper_driver.h"
#include "motor_driver.h"
#include "driver_eg2131_gpio.h"
#include "main.h"
#include <stdint.h>

typedef struct { int8_t A; int8_t B; } StepAB;

static const StepAB seq8[8] = {
  {+1,  0},   // A正向，B断
  {+1, +1},   // A正向，B正向
  { 0, +1},   // A断， B正向
  {-1, +1},   // A反向，B正向
  {-1,  0},   // A反向，B断
  {-1, -1},   // A反向，B反向
  { 0, -1},   // A断， B反向
  {+1, -1},   // A正向，B反向
};

/*
static const StepAB seq4_single[4] = {
  {+1,  0},
  { 0, +1},
  {-1,  0},
  { 0, -1},
};
*/
static uint8_t  idx = 0;
static uint8_t  dir = 0;
static uint16_t step_ms = 800;     // 先慢
static uint8_t  pwm_percent = 5;  // 先大点

static void apply_A(int8_t a)
{
  if (a > 0) {        // A+ : AP PWM, AN LOW
    HB_Set(HB_AP, HB_HIGH);
    HB_Set(HB_AN, HB_LOW);
  } else if (a < 0) { // A- : AN PWM, AP LOW
    HB_Set(HB_AP, HB_LOW);
    HB_Set(HB_AN, HB_HIGH);
  } else {            // A0 : 刹车
    HB_Set(HB_AP, HB_LOW);
    HB_Set(HB_AN, HB_LOW);
  }
}

static void apply_B(int8_t b)
{
  if (b > 0) {        // B+ : BP PWM, BN LOW
    HB_Set(HB_BP, HB_HIGH);
    HB_Set(HB_BN, HB_LOW);
  } else if (b < 0) { // B- : BN PWM, BP LOW
    HB_Set(HB_BP, HB_LOW);
    HB_Set(HB_BN, HB_HIGH);
  } else {            // B0 : 刹车
    HB_Set(HB_BP, HB_LOW);
    HB_Set(HB_BN, HB_LOW);
  }
}

void Stepper_Init(void)
{
  driver_eg2131_gpio_init();
  driver_eg2131_gpio_set_pwm_percent(pwm_percent);

// 上电先停一下，避免突然拉电流（可选）
  Stepper_Stop();
  idx = 0;
  dir = 0;
}

void Stepper_SetDir(uint8_t d) { dir = d ? 1 : 0; }

void Stepper_SetDelayMs(uint16_t ms)
{
  if (ms < 1) ms = 1;
  if (ms > 5000) ms = 5000;
  step_ms = ms;
}

void Stepper_SetPWMPercent(uint8_t percent)
{
  if (percent > 98) percent = 98;
  pwm_percent = percent;
  driver_eg2131_gpio_set_pwm_percent(pwm_percent);
}

// 在 stepper_driver.c 顶部加两个参数
static uint8_t pwm_dual   = 15; // 双相步
static uint8_t pwm_single = 21; // 单相步（≈15*1.414）
void Stepper_StepOnce_Blocking(void)
{
  uint8_t idx_now = idx;
  int8_t a = seq8[idx_now].A;
  int8_t b = seq8[idx_now].B;

  // 1) 按当前拍决定PWM（单相步提高一点）
  if ((a != 0) && (b != 0)) {
    Stepper_SetPWMPercent(pwm_dual);    // 双相
  } else {
    Stepper_SetPWMPercent(pwm_single);  // 单相
  }

  // 2) 输出这一拍
  apply_A(a);
  apply_B(b);

  // 3) 更新拍号
  idx = dir ? ((idx_now + 7) & 7) : ((idx_now + 1) & 7);

  HAL_Delay(step_ms);
}

void Stepper_Stop(void)
{
  HB_Set(HB_AP, HB_LOW);
  HB_Set(HB_AN, HB_LOW);
  HB_Set(HB_BP, HB_LOW);
  HB_Set(HB_BN, HB_LOW);
}

void Stepper_RunSteps_Accel(uint32_t steps,
                            uint16_t start_ms,
                            uint16_t target_ms,
                            uint16_t accel_every_n_steps)
{
  if (start_ms < 50) start_ms = 50;
  if (target_ms < 50) target_ms = 50;
  if (start_ms < target_ms) start_ms = target_ms;

  uint16_t cur = start_ms;
  uint32_t i;

  Stepper_SetDelayMs(cur);

  for (i = 0; i < steps; i++) {
    Stepper_StepOnce_Blocking();

    // 每 accel_every_n_steps 步，把延时减 1ms（可按需改成 2ms/5ms）
    if (cur > target_ms && (i % accel_every_n_steps) == 0) {
      cur--;
      Stepper_SetDelayMs(cur);
    }
  }
}
