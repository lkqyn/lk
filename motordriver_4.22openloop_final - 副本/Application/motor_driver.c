#include "motor_driver.h"
#include "driver_eg2131_gpio.h"

static motor_mode_t g_mode = MOTOR_MODE_STEPPER_GPIO;

void Motor_InitStepperGPIO(void)
{
  
  driver_eg2131_gpio_init();              // 启动PWM并全OFF
  driver_eg2131_gpio_set_pwm_percent(50); // 建议默认 40~60 之间
  g_mode = MOTOR_MODE_STEPPER_GPIO;
}

void Motor_SetMode(motor_mode_t mode)
{
  if (mode == g_mode) return;

  // 1) 全关断（安全态）
  HB_Set(HB_AP, HB_OFF);
  HB_Set(HB_AN, HB_OFF);
  HB_Set(HB_BP, HB_OFF);
  HB_Set(HB_BN, HB_OFF);

  // 2) 留一点时间放电（可选）
  for (volatile int i=0; i<100000; i++) { /* wait */ }

  // 3) 模式切换（目前只实现步进模式：HIN PWM + LIN GPIO）
  if (mode == MOTOR_MODE_STEPPER_GPIO) {
    driver_eg2131_gpio_init();
    driver_eg2131_gpio_set_pwm_percent(50);
    g_mode = mode;
    return;
  }

  // PWM/FOC 预留：未来在这里做
}

void HB_Set(hb_id_t id, hb_state_t s)
{
  // 现在的 driver_eg2131_gpio_set() 内部已经是：HIN PWM + LIN GPIO
  driver_eg2131_gpio_set(id, s);
}

void HB_SetPWMDuty_HighSide(hb_id_t id, uint16_t duty, uint16_t period)
{
  (void)id; (void)duty; (void)period;
  // 预留：未来实现“自定义 duty”版本
}




// 仅用于示波器测量：让 A+(AP) 固定输出 PWM，其他全关断
void Motor_Test_AP_Waveform(uint8_t pwm_percent)
{
  if (pwm_percent > 98) pwm_percent = 98;

  // 1) 初始化PWM（启用CH/CHN）
  driver_eg2131_gpio_init();
  driver_eg2131_gpio_set_pwm_percent(pwm_percent);

  // 2) 其它三相全关断（防止干扰）
  HB_Set(HB_AN, HB_LOW);
  HB_Set(HB_BP, HB_LOW);
  HB_Set(HB_BN, HB_LOW);

  // 3) 只让 A+ 这相输出 PWM
  // 你现在的同相PWM方案里：HB_HIGH -> CH输出占空比 = pwm_percent（CHN由硬件极性配合）
  HB_Set(HB_AP, HB_HIGH);

  // 4) 保持输出（不要再调用 Stepper_StepOnce_Blocking）
  // 让它一直跑，方便你用示波器稳定触发
}