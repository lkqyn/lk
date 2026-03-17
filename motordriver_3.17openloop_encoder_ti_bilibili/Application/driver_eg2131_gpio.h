#ifndef __DRIVER_EG2131_GPIO_H
#define __DRIVER_EG2131_GPIO_H

#include <stdint.h>
#include "motor_driver.h"

void driver_eg2131_gpio_init(void);
void driver_eg2131_gpio_set(hb_id_t id, hb_state_t s);

// PWM Á¦¶È£¨0~98£©
void driver_eg2131_gpio_set_pwm_percent(uint8_t percent);

#endif

