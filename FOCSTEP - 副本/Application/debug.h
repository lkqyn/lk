#ifndef DEBUG_H
#define DEBUG_H

/*
 * USB CDC 调试与遥测模块。
 * 解析文本命令、组织状态文本，并向 VOFA/串口输出非阻塞诊断数据。
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 初始化命令缓冲区、遥测状态和 CDC 输出状态。 */
void Debug_Init(void);
/* 主循环调用：解析已经接收完整的文本命令。 */
void Debug_Loop(void);
/* 主循环调用：推进 VOFA 帧、文本输出和长诊断转储。 */
void Debug_TelemetryLoop(void);
/* 脉冲接口就绪后自动开启JustFloat末端验证遥测；不改变脉冲控制。 */
uint8_t Debug_StartPulseVofa(void);
/* USB CDC 接收回调：只缓存原始字节，不在中断中执行命令。 */
void Debug_CdcRxCallback(const uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif
