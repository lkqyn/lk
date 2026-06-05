#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VOFA_CHANNEL_COUNT   6u
#define VOFA_FRAME_SIZE      ((VOFA_CHANNEL_COUNT + 1u) * 4u)

typedef struct
{
    float frame[VOFA_CHANNEL_COUNT + 1u];
} vofa_handle_t;

extern vofa_handle_t g_vofa;
extern volatile uint32_t vofa_cdc_ok_count;
extern volatile uint32_t vofa_cdc_busy_count;

void VOFA_Init(void);
uint8_t VOFA_SendFrame6(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6);
void VOFA_TxCpltCallback(void);

#ifdef __cplusplus
}
#endif

#endif
