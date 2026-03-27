#include "driver_eg2131_gpio.h"
#include "main.h"

// EG2131 真值表：
// LOW : H=0 L=0  -> LO=1（下管导通）
// HIGH: H=1 L=1  -> HO=1（上管导通）
// OFF : H!=L     -> HO=0 LO=0（上下关），推荐 H=0 L=1
static inline void set_pair(GPIO_TypeDef* Hport, uint16_t Hpin,
                            GPIO_TypeDef* Lport, uint16_t Lpin,
                            hb_state_t s)
{
  // 安全过渡：先 OFF（H=0, L=1）
  HAL_GPIO_WritePin(Hport, Hpin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Lport, Lpin, GPIO_PIN_SET);
  for (volatile int i=0; i<200; i++) __NOP();

  if (s == HB_LOW) {
    HAL_GPIO_WritePin(Hport, Hpin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Lport, Lpin, GPIO_PIN_RESET);
  } else if (s == HB_HIGH) {
    HAL_GPIO_WritePin(Hport, Hpin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Lport, Lpin, GPIO_PIN_SET);
  } else { // HB_OFF
    HAL_GPIO_WritePin(Hport, Hpin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Lport, Lpin, GPIO_PIN_SET);
  }
}

void driver_eg2131_gpio_init(void)
{
  // 上电统一关断
  set_pair(PWM_AP_H_GPIO_Port, PWM_AP_H_Pin, PWM_AP_L_GPIO_Port, PWM_AP_L_Pin, HB_OFF);
  set_pair(PWM_AN_H_GPIO_Port, PWM_AN_H_Pin, PWM_AN_L_GPIO_Port, PWM_AN_L_Pin, HB_OFF);
  set_pair(PWM_BP_H_GPIO_Port, PWM_BP_H_Pin, PWM_BP_L_GPIO_Port, PWM_BP_L_Pin, HB_OFF);
  set_pair(PWM_BN_H_GPIO_Port, PWM_BN_H_Pin, PWM_BN_L_GPIO_Port, PWM_BN_L_Pin, HB_OFF);
}

void driver_eg2131_gpio_set(hb_id_t id, hb_state_t s)
{
  switch (id) {
    case HB_AP:
      set_pair(PWM_AP_H_GPIO_Port, PWM_AP_H_Pin, PWM_AP_L_GPIO_Port, PWM_AP_L_Pin, s);
      break;
    case HB_AN:
      set_pair(PWM_AN_H_GPIO_Port, PWM_AN_H_Pin, PWM_AN_L_GPIO_Port, PWM_AN_L_Pin, s);
      break;
    case HB_BP:
      set_pair(PWM_BP_H_GPIO_Port, PWM_BP_H_Pin, PWM_BP_L_GPIO_Port, PWM_BP_L_Pin, s);
      break;
    case HB_BN:
      set_pair(PWM_BN_H_GPIO_Port, PWM_BN_H_Pin, PWM_BN_L_GPIO_Port, PWM_BN_L_Pin, s);
      break;
    default:
      break;
  }
}
