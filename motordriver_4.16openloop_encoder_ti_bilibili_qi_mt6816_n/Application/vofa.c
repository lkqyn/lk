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
    uint16_t head, next;
    float frame[VOFA_CHANNEL_COUNT + 1u];

    frame[0] = ch1;
    frame[1] = ch2;
    frame[2] = ch3;
    frame[3] = ch4;
    frame[4] = ch5;
    frame[5] = ch6;

    // VOFA+ JustFloat ≥£”√÷°Œ≤£∫+Inf = 0x7F800000
    
      uint32_t inf_bits = 0x7F800000u;
      memcpy(&frame[6], &inf_bits, sizeof(float));
    

    __disable_irq();

    head = g_vofa.head;
    next = vofa_next_index(head);

    if (next == g_vofa.tail) {
        g_vofa.overflow_cnt++;
        __enable_irq();
        return 0u;
    }

    memcpy(g_vofa.tx_buf[head], frame, VOFA_FRAME_SIZE);
    g_vofa.head = next;

    __enable_irq();
    return 1u;
}

void VOFA_Process(void)
{
    if (g_vofa.tx_busy) {
        return;
    }

    __disable_irq();

    if (g_vofa.head == g_vofa.tail) {
        __enable_irq();
        return;
    }

    uint8_t *p = g_vofa.tx_buf[g_vofa.tail];

    __enable_irq();

    if (CDC_Transmit_FS(p, VOFA_FRAME_SIZE) == USBD_OK) {
        __disable_irq();
        g_vofa.tx_busy = 1u;
        __enable_irq();
    }
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
