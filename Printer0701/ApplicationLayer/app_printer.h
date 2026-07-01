#ifndef APP_PRINTER_H
#define APP_PRINTER_H

#include <stdint.h>

void AppPrinter_TestPrint(void);
void AppPrinter_InitBaud115200(void);
void AppPrinter_SendZpl(const char *zpl);
void AppPrinter_SendBytes(const uint8_t *data, uint32_t size);
uint16_t AppPrinter_QueryHostStatus(uint8_t *buffer, uint16_t bufferSize, uint32_t timeoutMs);
uint8_t AppPrinter_IsHostStatusDone(const uint8_t *buffer, uint16_t size);
uint8_t AppPrinter_WaitDone(uint32_t timeoutMs, uint32_t intervalMs);

#endif
