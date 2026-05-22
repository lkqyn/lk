#ifndef APP_PRINTER_H
#define APP_PRINTER_H

#include <stdint.h>

void AppPrinter_TestPrint(void);
void AppPrinter_SendZpl(const char *zpl);
void AppPrinter_SendBytes(const uint8_t *data, uint32_t size);

#endif
