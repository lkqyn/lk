#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void Debug_Init(void);
void Debug_Loop(void);
void Debug_TelemetryLoop(void);
void Debug_CdcRxCallback(const uint8_t *data, uint32_t len);
void Debug_Print(const char *text);
void Debug_SpeedTraceReset(float command_rpm);
void Debug_SpeedTraceSample(void);

#ifdef __cplusplus
}
#endif

#endif
