#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include <stdint.h>
#include  "encoder.h"
typedef struct
{
    float Ts;          // PWM周期计数(ARR+1)
    float Udc;         // 母线电压
    float outmax;      // Ud/Uq最大输出

    float Ud;
    float Uq;
    float Ualpha;
    float Ubeta;
uint16_t angle_sector;   // 当前电角大扇区
uint16_t angle;          // 当前90°电角扇区内细分角度 0~255
int16_t  scope;          // 当前扇区宽度（这里可固定）
uint16_t lead_angle;     // 超前角 0~1023
    
    float sintheta;
    float costheta;

    float fe;          // 开环电频率 Hz

    float duty_max;    // 最大占空比比例 0~1
    float deadband;    // 小占空比关闭阈值

    uint8_t  sector;   // 1~4 扇区
    uint16_t Ta;       // A相占空
    uint16_t Tb;       // B相占空

    // 预留给后续闭环
    float Ialpha;
    float Ibeta;
    float Id;
    float Iq;

    float tar_Id;
    float tar_Iq;
    float id_err;
    float iq_err;
    float last_Id_err;
    float last_Iq_err;
    float id_inter;
    float iq_inter;

    float kp;
    float ki;
    float kd;

    float Ls;
    float FLux;

} foc_TypeDef;

extern foc_TypeDef m1_foc;

void FOC_Init(void);
void FOC_AllOff(void);

void set_uduq(foc_TypeDef *mfoc, float ud, float uq);
void setfoc_angle(foc_TypeDef *mfoc, uint16_t angle);
void repark_transfer(foc_TypeDef *mfoc);
void svpwmctr(foc_TypeDef *mfoc);
void set_duty_by_sector(foc_TypeDef *mfoc);
void foc_open(foc_TypeDef *mfoc, float angle);

void FOC_OpenLoopInit(foc_TypeDef *mfoc, float vbus, float uq, float fe_hz);

//void FOC_OpenLoopUpdate(foc_TypeDef *mfoc, float dt);
//void FOC_OutputByTheta(foc_TypeDef *mfoc, float angle, float uq);
//void FOC_OpenLoopSetFe(foc_TypeDef *mfoc, float fe_hz);
//void FOC_OpenLoopSetUqRatio(foc_TypeDef *mfoc, float uq_ratio);
//void FOC_OpenLoopRampUqRatio(foc_TypeDef *mfoc, float target_ratio, float step_per_call);
void Sector_tracker_inc_encoder(foc_TypeDef *mfoc, encoder_t *enc);
void Lock_A_Phase_Positive(uint16_t duty);
void Lock_A_Phase_Negative(uint16_t duty);
#endif