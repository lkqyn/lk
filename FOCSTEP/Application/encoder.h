#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ENCODER_CPR_PER_PHASE          (1000L)
#define ENCODER_COUNT_PER_REV          (ENCODER_CPR_PER_PHASE * 4L)
#define ENCODER_ELECTRICAL_CYCLES      (50.0f)
#define ENCODER_DIRECTION              (-1.0f)

typedef struct
{
    int32_t count;
    int32_t zero_count;
    int32_t relative_count;
    int32_t count_in_rev;
    int32_t delta_count;
    float mechanical_angle_rad;
    float mechanical_angle_deg;
    float electrical_angle_rad;
    float speed_raw_rpm;
    float speed_window_rpm;
    float speed_lpf_rpm;
} Encoder_State_t;

void Encoder_Init(void);
void Encoder_Update(void);
void Encoder_UpdateSpeed(float dt_s);
void Encoder_SetZeroCurrentPosition(void);
int32_t Encoder_GetCount(void);
const Encoder_State_t *Encoder_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
