#ifndef __CURRENT_H__
#define __CURRENT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    uint16_t raw_a;
    uint16_t raw_b;
    uint16_t raw_bus;
    uint16_t raw_ntc;
    float offset_a;
    float offset_b;
    float ia;
    float ib;
    float bus_v;
    float ntc_v;
    uint32_t sample_count;
    uint32_t debug_adc_start;
} Current_State_t;

void Current_Init(void);
void Current_CalibrateOffset(uint16_t sample_count);
void Current_StartSampling(void);
void Current_Update(void);
const Current_State_t *Current_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
