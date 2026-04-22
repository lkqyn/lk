#include "connecting.h"
#include "pid.h"
#include "FOC.h"

// 外部变量
connect_crt_TypeDef connect_crt;
extern foc_TypeDef m1_foc;
extern PID_Controller_t speed_pi;
extern volatile float Uq_ref;
extern volatile float Ud_ref;

// 初始化控制参数
void init_connect_crt(connect_crt_TypeDef* connect)
{
    connect->motor_set = 0;      // 步进电机
    connect->motor_mode = 0;     // 默认空闲
    connect->drive_current = 12.0f; // 先作为Uq输出限幅参考
    connect->speed = 0.0f;       // 目标速度 rpm
    connect->s_acc = 300.0f;     // 加速度 rpm/s

    // 初始化速度环PID
    PID_Controller_Init(&speed_pi,
                        0.004f,                   // kp
                        0.0008f,                  // ki1
                        0.0f,                    // kd
                        connect->speed,          // target
                        -fabsf(connect->drive_current), // output min
                        fabsf(connect->drive_current)); // output max

    PID_Controller_Reset(&speed_pi);
}

// 停止电机
void stop_motor(connect_crt_TypeDef* connect)
{
    if (connect->motor_mode == 2)
    {
        connect->speed = 0.0f;
        speed_pi.target = 0.0f;

        Uq_ref = 0.0f;
        Ud_ref = 0.0f;

        m1_foc.Uq = 0.0f;
        m1_foc.Ud = 0.0f;
    }

    connect->motor_mode = 0;
}