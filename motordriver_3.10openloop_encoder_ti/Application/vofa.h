#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VOFA_CHANNEL_COUNT   6u
#define VOFA_FRAME_SIZE      ((VOFA_CHANNEL_COUNT + 1u) * 4u)   // 6个float + 1个帧尾float
#define VOFA_FRAME_CAPACITY  64u   // 最多缓存64帧

typedef struct
{
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t overflow_cnt;
    uint8_t tx_busy;
    uint8_t tx_buf[VOFA_FRAME_CAPACITY][VOFA_FRAME_SIZE];
} vofa_handle_t;

extern vofa_handle_t g_vofa;

void VOFA_Init(void);

// 发送一帧 6 通道 JustFloat 数据
// 返回1成功，0表示缓冲区满
uint8_t VOFA_SendFrame6(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6);

// 主循环里调用，推动 USB CDC 发送
void VOFA_Process(void);

// 在 CDC_TransmitCplt_FS 里调用
void VOFA_TxCpltCallback(void);

uint32_t VOFA_GetOverflowCount(void);
uint16_t VOFA_GetPendingFrames(void);

#ifdef __cplusplus
}
#endif

#endif

