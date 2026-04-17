#ifndef FOC_TYPEDEF_H
#define FOC_TYPEDEF_H

#include "main.h"

//定义系统允许的最大电流
#define System_MAX_I 2.5f

typedef struct
{
    uint8_t angle_sector;   // 电角度扇区
    int scope;
    int16_t angle;          // 电角度
    int lead_angle;

    float Ud;               // 目标电压
    float Uq;

    float Ualpha;           // A相电压反Park后
    float Ubeta;            // B相电压反Park后
    float Udc;              // 电源电压

    uint16_t Ta;            // A相时间
    uint16_t Tb;            // B相时间
    uint16_t Ts;            // 总时间
    uint8_t sector;         // 换向扇区
    float deadband;         // 小占空直接关断阈值
    float sintheta;         // sin值
    float costheta;         // cos值
    float outmax;           // 最大限幅

    // PID
    float Ialpha;
    float Ibeta;

    float Iq;
    float Id;
    float tar_Iq;
    float tar_Id;
    float iq_err;
    float id_err;
    float last_Iq_err;
    float last_Id_err;
    float kp;
    float ki;
    float kd;
    float iq_inter;
    float id_inter;

    // 前馈控制
    float Ls;               // 电机电感
    float FLux;             // 磁链

} foc_TypeDef;

extern foc_TypeDef m1_foc;

void setfoc_angle(foc_TypeDef *mfoc, uint16_t angle);      // 设置电角度
void repark_transfer(foc_TypeDef *mfoc);                   // repark变换
void svpwmctr(foc_TypeDef *mfoc);                          // pwm调制
void set_uduq(foc_TypeDef *mfoc, float ud, float uq);
void foc_open(foc_TypeDef *mfoc, uint16_t angle);          // 开环控制
void can_foc_init(void);                                   // 初始化参数
void set_foc_Iqcurrent(foc_TypeDef *mfoc, float current);  // 设置focIq闭环电流
void get_AB_current(foc_TypeDef *mfoc);                    // 获取AB相电流
void current_ctr(foc_TypeDef *mfoc, uint16_t angle);       // 电流闭环控制
void adjust_I(void);

void FOC_Init(void);               // 初始化，寻找0点
void Sector_tracker(void);         // 寻找扇区，找到电角度
void b_foc_init(void);             // 初始化参数与0点
void get_scope(void);              // 从储存获取每个扇区长度
void park_transform(foc_TypeDef *mfoc);
void set_duty_by_sector(foc_TypeDef *mfoc);
void foc_current_pid(foc_TypeDef *mfoc);
void FOC_AllOff(void);

#endif // FOC_TYPEDEF_H
