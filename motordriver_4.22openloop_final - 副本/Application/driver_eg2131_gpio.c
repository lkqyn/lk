#include "driver_eg2131_gpio.h"
#include "main.h"
#include "tim.h"
#include <stdint.h>

/*
同相PWM方案（CH + CHN）：
- CH 与 CHN 输出同相PWM（靠 CubeMX：OCPolarity=HIGH, OCNPolarity=LOW）
- EG2131 输入得到：PWM=1 -> 11（上管），PWM=0 -> 00（下管）
- 所以：
  HB_LOW  = 占空比 0%   （恒 00，输出拉地）
  HB_HIGH = 占空比 x%   （在00/11之间PWM）
  HB_OFF  = 也用 0%（刹车态，先保证能转起来）
*/

static uint8_t  g_pwm_percent = 50;  // 0..95/98，建议先 50~80
static uint16_t g_per1 = 0, g_per8 = 0;

static inline uint16_t period(TIM_HandleTypeDef* htim)
{
  return (uint16_t)(__HAL_TIM_GET_AUTORELOAD(htim) + 1);
}
static inline uint16_t duty_from_percent(uint16_t per, uint8_t percent)
{
  if (percent > 98) percent = 98;        // 不要100%
  return (uint16_t)((per * (uint32_t)percent) / 100);
}
static inline void set_pwm(TIM_HandleTypeDef* htim, uint32_t ch, uint16_t duty)
{
  uint16_t per = period(htim);
  if (duty > per) duty = per;
  __HAL_TIM_SET_COMPARE(htim, ch, duty);
}

// ===== 通用：每相三态控制 =====
static inline void hb_off(TIM_HandleTypeDef* htim, uint32_t ch)
{
  // 进入 Idle State：CH=1, CHN=0 -> EG2131 真OFF
  HAL_TIM_PWM_Stop(htim, ch);
  HAL_TIMEx_PWMN_Stop(htim, ch);
}

static inline void hb_low(TIM_HandleTypeDef* htim, uint32_t ch)
{
  // 00：下管导通（刹车）
  HAL_TIM_PWM_Start(htim, ch);
  HAL_TIMEx_PWMN_Start(htim, ch);
  set_pwm(htim, ch, 0);
}

static inline void hb_pwm(TIM_HandleTypeDef* htim, uint32_t ch, uint16_t duty)
{
  // 00/11：同步PWM
  HAL_TIM_PWM_Start(htim, ch);
  HAL_TIMEx_PWMN_Start(htim, ch);
  set_pwm(htim, ch, duty);
}

static inline void bridge_AP(hb_state_t s)
{
  if (s == HB_OFF) { hb_off(&htim1, TIM_CHANNEL_1); return; }
  if (s == HB_LOW) { hb_low(&htim1, TIM_CHANNEL_1); return; }
  hb_pwm(&htim1, TIM_CHANNEL_1, duty_from_percent(g_per1, g_pwm_percent));
}

static inline void bridge_AN(hb_state_t s)
{
  if (s == HB_OFF) { hb_off(&htim1, TIM_CHANNEL_2); return; }
  if (s == HB_LOW) { hb_low(&htim1, TIM_CHANNEL_2); return; }
  hb_pwm(&htim1, TIM_CHANNEL_2, duty_from_percent(g_per1, g_pwm_percent));
}

static inline void bridge_BP(hb_state_t s)
{
  if (s == HB_OFF) { hb_off(&htim1, TIM_CHANNEL_3); return; }
  if (s == HB_LOW) { hb_low(&htim1, TIM_CHANNEL_3); return; }
  hb_pwm(&htim1, TIM_CHANNEL_3, duty_from_percent(g_per1, g_pwm_percent));
}

static inline void bridge_BN(hb_state_t s)
{
  if (s == HB_OFF) { hb_off(&htim8, TIM_CHANNEL_1); return; }
  if (s == HB_LOW) { hb_low(&htim8, TIM_CHANNEL_1); return; }
  hb_pwm(&htim8, TIM_CHANNEL_1, duty_from_percent(g_per8, g_pwm_percent));
}

void driver_eg2131_gpio_init(void)
{
  // 启动 CH 和 CHN（必须）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);

  g_per1 = period(&htim1);
  g_per8 = period(&htim8);

  // 全部先关闭
  bridge_AP(HB_OFF);
  bridge_AN(HB_OFF);
  bridge_BP(HB_OFF);
  bridge_BN(HB_OFF);
}

void driver_eg2131_gpio_set(hb_id_t id, hb_state_t s)
{
  switch (id) {
    case HB_AP: bridge_AP(s); break;
    case HB_AN: bridge_AN(s); break;
    case HB_BP: bridge_BP(s); break;
    case HB_BN: bridge_BN(s); break;
    default: break;
  }
}

void driver_eg2131_gpio_set_pwm_percent(uint8_t percent)
{
  if (percent > 98) percent = 98;
  g_pwm_percent = percent;
}
