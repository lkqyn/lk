#include "vofa.h"
#include "usbd_cdc_if.h"
#include <string.h>

vofa_handle_t g_vofa;

static uint16_t vofa_next_index(uint16_t idx)
{
    idx++;
    if (idx >= VOFA_FRAME_CAPACITY) {
        idx = 0;
    }
    return idx;
}

void VOFA_Init(void)
{
    memset(&g_vofa, 0, sizeof(g_vofa));
}

uint16_t VOFA_GetPendingFrames(void)
{
    uint16_t head = g_vofa.head;
    uint16_t tail = g_vofa.tail;

    if (head >= tail) {
        return (uint16_t)(head - tail);
    } else {
        return (uint16_t)(VOFA_FRAME_CAPACITY - tail + head);
    }
}

uint32_t VOFA_GetOverflowCount(void)
{
    return g_vofa.overflow_cnt;
}

uint8_t VOFA_SendFrame6(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6)
{
    const uint8_t tail[4] = {0x00u, 0x00u, 0x80u, 0x7Fu};
    float channels[VOFA_CHANNEL_COUNT];

    channels[0] = ch1;
    channels[1] = ch2;
    channels[2] = ch3;
    channels[3] = ch4;
    channels[4] = ch5;
    channels[5] = ch6;

    memcpy(&g_vofa.direct_frame[0], channels, VOFA_CHANNEL_COUNT * sizeof(float));
    memcpy(&g_vofa.direct_frame[VOFA_CHANNEL_COUNT * sizeof(float)], tail, sizeof(tail));

    g_vofa.frame_ready = 1u;
    return 1u;
}

void VOFA_Process(void)
{
    if (!g_vofa.frame_ready || g_vofa.tx_busy) {
        return;
    }

    g_vofa.tx_busy = 1u;
    if (CDC_Transmit_FS((uint8_t *)g_vofa.direct_frame, VOFA_FRAME_SIZE) != USBD_OK) {
        g_vofa.tx_busy = 0u;
        return;
    }

    g_vofa.frame_ready = 0u;
}

void VOFA_TxCpltCallback(void)
{
    __disable_irq();

    if (g_vofa.tail != g_vofa.head) {
        g_vofa.tail = vofa_next_index(g_vofa.tail);
    }

    g_vofa.tx_busy = 0u;

    __enable_irq();
}
