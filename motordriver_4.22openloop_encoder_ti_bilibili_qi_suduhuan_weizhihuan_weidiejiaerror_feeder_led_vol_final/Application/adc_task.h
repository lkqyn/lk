#ifndef __ADC_TASK_H__
#define __ADC_TASK_H__

#include "main.h"

extern float offet_I[2];

uint16_t get_adc_xchannel(uint8_t CHANNEL);
float get_temp(void);
float get_vol(void);

#endif
