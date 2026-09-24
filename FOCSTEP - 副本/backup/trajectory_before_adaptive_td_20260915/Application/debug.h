#ifndef DEBUG_H
#define DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void Debug_Init(void);
void Debug_Loop(void);
void Debug_TelemetryLoop(void);
void Debug_CdcRxCallback(const uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif
