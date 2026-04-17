#include "connecting.h"
#include "pid.h"
#include "FOC.h"
#include "math.h"

float speed_send = 0.0f;
connect_crt_TypeDef connect_crt;

extern foc_TypeDef m1_foc;
extern PID_Controller_t speed_pi;

void stop_motor(connect_crt_TypeDef* connect)
{
    if(connect->motor_mode == 1U)
    {
        connect->motor_mode = 0;
        set_foc_Iqcurrent(&m1_foc, 0.0f);
        set_uduq(&m1_foc, 0.0f, 0.0f);
        foc_open(&m1_foc, 0);
    }
    else if(connect->motor_mode == 2U)
    {
        connect->speed = 0.0f;
        speed_pi.target = 0.0f;
        set_foc_Iqcurrent(&m1_foc, 0.0f);
    }
    else
    {
        connect->motor_mode = 0;
    }
}

void check_limit(void)
{
}

void get_zero_by_current(void)
{
}

void init_connect_crt(connect_crt_TypeDef* connect)
{
    connect->motor_set = 0;
    connect->motor_mode = 0;
    connect->drive_current = 1.0f;
    connect->max_current = 2.0f;
    connect->speed = 0.0f;
    connect->distance = 0.0f;
    connect->max_speed = 120.0f;
    connect->accel = 400.0f;
    connect->decel = 400.0f;
    connect->s_acc = 1000.0f;

    PID_Controller_Init(&speed_pi,
                        0.01f,
                        0.00010f,
                        0.0f,
                        connect->speed,
                        -connect->max_current,
                        connect->max_current);

    PID_Controller_Init(&AngleControl.pid_angle,
                        0.05f,
                        0.0001f,
                        0.8f,
                        0.0f,
                        -connect->max_current,
                        connect->max_current);
}
