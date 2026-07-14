#ifndef __CONNECTING_H__
#define __CONNECTING_H__

#include "main.h"

typedef struct
{
    uint8_t motor_set;      // 0:步进电机
    uint8_t motor_mode;     // 0:空闲  2:速度环

    float drive_current;    // 先借用作速度环输出限幅参考
    float speed;            // 目标速度 rpm
    float s_acc;            // 加速度 rpm/s

} connect_crt_TypeDef;

extern connect_crt_TypeDef connect_crt;

void init_connect_crt(connect_crt_TypeDef* connect);
void stop_motor(connect_crt_TypeDef* connect);

#endif