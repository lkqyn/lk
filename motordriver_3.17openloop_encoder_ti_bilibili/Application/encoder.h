#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"
#include "tim.h"
#include <stdint.h>

typedef struct
{
    int32_t cnt_now;
    int32_t cnt_last;
    int32_t delta;           // 本周期增量（count）

    int64_t pos_cnt_total;   // 累计位置（count）
    float   pos_rev;         // 机械位置（rev）
    float   vel_rps;         // 机械速度（rev/s）
    float   vel_rpm;         // 机械速度（rpm）
    float vel_rpm_f;
} encoder_t;

/**
 * @brief 初始化结构体变量（不启动定时器）
 */
void Encoder_Init(encoder_t *enc);

/**
 * @brief 启动编码器定时器（TIM2 encoder mode）
 */
void Encoder_Start(encoder_t *enc, TIM_HandleTypeDef *htim_enc);

/**
 * @brief 周期更新（建议在 TIM6 10kHz 中断里调用）
 */
void Encoder_Update(encoder_t *enc, TIM_HandleTypeDef *htim_enc, float dt);

/**
 * @brief 可选：清零位置（用于Z回零后对齐）
 */
void Encoder_Zero(encoder_t *enc, TIM_HandleTypeDef *htim_enc);
int32_t Encoder_GetCountInOneTurn(encoder_t *enc);
#endif
