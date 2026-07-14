#ifndef __DRIVER_EG2131_GPIO_H
#define __DRIVER_EG2131_GPIO_H

#include "motor_driver.h"
#include <stdint.h>
// 初始化：把所有半桥置 OFF（H=0, L=1）
void driver_eg2131_gpio_init(void);

// 设置单个半桥三态：OFF/LOW/HIGH
void driver_eg2131_gpio_set(hb_id_t id, hb_state_t s);

#endif

