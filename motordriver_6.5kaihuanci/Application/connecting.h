#ifndef __CONNECTING_H__
#define __CONNECTING_H__

#include "main.h"

typedef struct
{
    uint8_t motor_set;
    uint8_t motor_mode;
    float drive_current;
    float max_current;
    float speed;
    float s_acc;
    float distance;
    float max_speed;
    float accel;
    float decel;
} connect_crt_TypeDef;

extern connect_crt_TypeDef connect_crt;

void init_connect_crt(connect_crt_TypeDef* connect);
void stop_motor(connect_crt_TypeDef* connect);
void check_limit(void);
void get_zero_by_current(void);

#endif
