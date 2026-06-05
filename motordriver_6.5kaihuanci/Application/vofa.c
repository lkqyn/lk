#include "vofa.h"
#include "usbd_cdc_if.h"
#include <string.h>

vofa_handle_t g_vofa;
volatile uint32_t vofa_cdc_ok_count = 0u;
volatile uint32_t vofa_cdc_busy_count = 0u;

void VOFA_Init(void)
{
    memset(&g_vofa, 0, sizeof(g_vofa));
}

uint8_t VOFA_SendFrame6(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6)
{
    uint32_t tail = 0x7F800000u;

    if(CDC_IsTxBusy_FS())
    {
        vofa_cdc_busy_count++;
        return 0u;
    }

    g_vofa.frame[0] = ch1;
    g_vofa.frame[1] = ch2;
    g_vofa.frame[2] = ch3;
    g_vofa.frame[3] = ch4;
    g_vofa.frame[4] = ch5;
    g_vofa.frame[5] = ch6;
    memcpy(&g_vofa.frame[6], &tail, sizeof(tail));

    if(CDC_Transmit_FS((uint8_t *)g_vofa.frame, VOFA_FRAME_SIZE) == USBD_OK)
    {
        vofa_cdc_ok_count++;
        return 1u;
    }

    vofa_cdc_busy_count++;
    return 0u;
}

void VOFA_TxCpltCallback(void)
{
}
