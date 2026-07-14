#include "feeder.h"

#ifndef FEEDER_COUNTS_PER_REV
#define FEEDER_COUNTS_PER_REV 4000LL
#endif

feeder_ctrl_t g_feeder;

static int64_t feeder_index_to_target_cnt(const feeder_ctrl_t *f, uint32_t index)
{
    int64_t delta_cnt = (int64_t)(f->turns_per_feed * (float)FEEDER_COUNTS_PER_REV * (float)index);
    return f->base_cnt + delta_cnt;
}

void Feeder_Init(feeder_ctrl_t *f,
                 encoder_t *enc,
                 float turns_per_feed,
                 float time_per_feed_s,
                 float vmax_limit_rpm,
                 float acc_ratio,
                 float dec_ratio)
{
    if ((f == 0) || (enc == 0))
        return;

    f->base_cnt = enc->pos_cnt_total;
    f->feed_index = 0;
    f->turns_per_feed = turns_per_feed;
    f->time_per_feed_s = time_per_feed_s;
    f->vmax_limit_rpm = vmax_limit_rpm;
    f->acc_ratio = acc_ratio;
    f->dec_ratio = dec_ratio;
    f->initialized = 1;
}

void Feeder_SetBaseHere(feeder_ctrl_t *f, encoder_t *enc)
{
    if ((f == 0) || (enc == 0))
        return;

    f->base_cnt = enc->pos_cnt_total;
    f->feed_index = 0;
    f->initialized = 1;
}

int Feeder_Once(feeder_ctrl_t *f,
                motion_trap2_t *m,
                encoder_t *enc)
{
    int ret;
    int64_t target_cnt;

    if ((f == 0) || (m == 0) || (enc == 0))
        return MOTION_ERR_PARAM;

    if (!f->initialized)
        return MOTION_ERR_PARAM;

    target_cnt = feeder_index_to_target_cnt(f, f->feed_index + 1u);

    ret = MotionTrap2_StartToAbsCountByTimeWithVmax(m,
                                                    enc,
                                                    target_cnt,
                                                    f->time_per_feed_s,
                                                    f->vmax_limit_rpm,
                                                    f->acc_ratio,
                                                    f->dec_ratio);
    if (ret == MOTION_OK)
    {
        f->feed_index++;
    }

    return ret;
}

int Feeder_GotoIndex(feeder_ctrl_t *f,
                     motion_trap2_t *m,
                     encoder_t *enc,
                     uint32_t index)
{
    int ret;
    int64_t target_cnt;

    if ((f == 0) || (m == 0) || (enc == 0))
        return MOTION_ERR_PARAM;

    if (!f->initialized)
        return MOTION_ERR_PARAM;

    target_cnt = feeder_index_to_target_cnt(f, index);

    ret = MotionTrap2_StartToAbsCountByTimeWithVmax(m,
                                                    enc,
                                                    target_cnt,
                                                    f->time_per_feed_s,
                                                    f->vmax_limit_rpm,
                                                    f->acc_ratio,
                                                    f->dec_ratio);
    if (ret == MOTION_OK)
    {
        f->feed_index = index;
    }

    return ret;
}

int64_t Feeder_GetIdealTargetCnt(const feeder_ctrl_t *f)
{
    if ((f == 0) || (!f->initialized))
        return 0;

    return feeder_index_to_target_cnt(f, f->feed_index);
}