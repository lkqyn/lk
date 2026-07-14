#ifndef __FEEDER_H__
#define __FEEDER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "motion_trap2.h"
#include "encoder.h"
#include <stdint.h>

typedef struct
{
    int64_t base_cnt;          // 送料基准位置
    uint32_t feed_index;       // 第几次送料
    float turns_per_feed;      // 每次送料圈数
    float time_per_feed_s;     // 每次送料时间
    float vmax_limit_rpm;      // 速度上限
    float acc_ratio;           // 加速时间占比
    float dec_ratio;           // 减速时间占比
    uint8_t initialized;       // 是否已初始化基准
} feeder_ctrl_t;

extern feeder_ctrl_t g_feeder;

void Feeder_Init(feeder_ctrl_t *f,
                 encoder_t *enc,
                 float turns_per_feed,
                 float time_per_feed_s,
                 float vmax_limit_rpm,
                 float acc_ratio,
                 float dec_ratio);

void Feeder_SetBaseHere(feeder_ctrl_t *f, encoder_t *enc);

int Feeder_Once(feeder_ctrl_t *f,
                motion_trap2_t *m,
                encoder_t *enc);

int Feeder_GotoIndex(feeder_ctrl_t *f,
                     motion_trap2_t *m,
                     encoder_t *enc,
                     uint32_t index);

int64_t Feeder_GetIdealTargetCnt(const feeder_ctrl_t *f);

#ifdef __cplusplus
}
#endif

#endif