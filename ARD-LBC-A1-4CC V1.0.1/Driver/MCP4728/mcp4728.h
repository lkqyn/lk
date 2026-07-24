#ifndef _MCP4728_H_
#define _MCP4728_H_

#include "stm32f1xx_hal.h"

void MCP4728_setOutput(uint8_t channel,uint16_t output);
void MCP4728_config(void);
void MCP4728_writeOutputSeq(uint16_t out1,uint16_t out2,uint16_t out3,uint16_t out4);

#define MCP4728_LDAC_GPIOx          GPIOC
#define MCP4728_LDAC_PIN            GPIO_PIN_1
#define MCP4728_RDYBSY_GPIOx        GPIOC
#define MCP4728_RDYBSY_PIN          GPIO_PIN_0

#endif
