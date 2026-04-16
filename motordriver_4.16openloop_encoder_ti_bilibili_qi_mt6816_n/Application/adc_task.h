#include "main.h"

///定义ADC值到电流转换系数
#define ADCtoI 0.005371f

extern float offet_I[2];

float get_temp(void);
float get_vol(void);
uint16_t get_adc_xchannel(uint8_t CHANNEL);//获取某一路ADC值