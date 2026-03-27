#include "encoder.h"

// 你的编码器：1000 CPR/相，TIM2 Encoder x4 => 4000 count/rev
#define ENC_COUNTS_PER_REV   4000.0f

// 如果你发现方向反了，把这个宏改成 1
#define ENC_INVERT_DIR       1

void Encoder_Init(encoder_t *enc)
{
    enc->cnt_now = 0;
    enc->cnt_last = 0;
    enc->delta = 0;
    enc->pos_cnt_total = 0;
    enc->pos_rev = 0.0f;
    enc->vel_rps = 0.0f;
    enc->vel_rpm = 0.0f;
    enc->vel_rpm_f = 0.0f;
    //HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_1);
    //HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_2);
}

void Encoder_Start(encoder_t *enc, TIM_HandleTypeDef *htim_enc)
{
    // 启动 TIM2 Encoder
    HAL_TIM_Encoder_Start(htim_enc, TIM_CHANNEL_ALL);

    // 清零硬件计数器
    __HAL_TIM_SET_COUNTER(htim_enc, 0);

    // 同步软件状态
    enc->cnt_last = 0;
    enc->pos_cnt_total = 0;
}

void Encoder_Zero(encoder_t *enc, TIM_HandleTypeDef *htim_enc)
{
    __HAL_TIM_SET_COUNTER(htim_enc, 0);
    enc->cnt_last = 0;
    enc->cnt_now = 0;
    enc->delta = 0;
    enc->pos_cnt_total = 0;
    enc->pos_rev = 0.0f;
    enc->vel_rps = 0.0f;
    enc->vel_rpm = 0.0f;
}


//void Encoder_Update(encoder_t *enc, TIM_HandleTypeDef *htim_enc, float dt)
//{
//    int32_t now = (int32_t)__HAL_TIM_GET_COUNTER(htim_enc);

//    // 32-bit计数差分（TIM2很大，常规情况下不会频繁溢出）
//    int32_t delta = now - enc->cnt_last;
//    enc->cnt_last = now;
//    enc->cnt_now = now;

//#if ENC_INVERT_DIR
//    delta = -delta;
//#endif

//    enc->delta = delta;
//    enc->pos_cnt_total += (int64_t)delta;
//    enc->pos_rev = (float)enc->pos_cnt_total / ENC_COUNTS_PER_REV;
//    enc->vel_rps = ((float)delta / ENC_COUNTS_PER_REV) / dt;
//    enc->vel_rpm = enc->vel_rps * 60.0f;

//    const float alpha = 0.05f;
//    enc->vel_rpm_f += alpha * (enc->vel_rpm - enc->vel_rpm_f);
//}

int32_t Encoder_GetCountInOneTurn(encoder_t *enc)
{
    int32_t cnt = (int32_t)(enc->pos_cnt_total % 4000);
    if (cnt < 0) cnt += 4000;
    return cnt;
}