#ifndef __ADC_TASK_H
#define __ADC_TASK_H

#include "main.h"

#define ADC_REF_VOLTAGE         3.3f
#define ADC_FULL_SCALE          4095.0f
#define CURRENT_SENSE_R         0.02f
#define CURRENT_AMP_GAIN        10.0f
#define CURRENT_VREF_VOLTAGE    1.75f

// (Vadc - Vref) / (Rs * Gain)
#define ADCtoI                  (ADC_REF_VOLTAGE / ADC_FULL_SCALE / (CURRENT_SENSE_R * CURRENT_AMP_GAIN))
#define CURRENT_VREF_ADC        (CURRENT_VREF_VOLTAGE / ADC_REF_VOLTAGE * ADC_FULL_SCALE)

extern float offet_I[2];

float get_temp(void);
float get_vol(void);
uint16_t get_adc_xchannel(uint8_t channel);
void calibrate_current_offset(uint16_t sample_times);
float get_phase_current(uint32_t rank, uint8_t index);

#endif
