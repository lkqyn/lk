#include "adc_task.h"
#include "adc.h"
#include "math.h"

float offet_I[2] = {0.0f, 0.0f};

uint16_t get_adc_xchannel(uint8_t CHANNEL)//获取某一路ADC值
{
    uint16_t adcvalue = 0;
    ADC_ChannelConfTypeDef sConfig = {0};

    switch(CHANNEL)
    {
        case 8:
            sConfig.Channel = ADC_CHANNEL_8;   // PB0 -> 电压检测
            break;

        case 9:
            sConfig.Channel = ADC_CHANNEL_9;   // PB1 -> NTC温度检测
            break;

        default:
            return 0;
    }

    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 0xFFFF);//等待ADC转换完成
    adcvalue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return adcvalue;
}

float get_temp(void)
{
    uint16_t adcvalue = get_adc_xchannel(9);

    if(adcvalue == 0) adcvalue = 1;
    if(adcvalue >= 4095) adcvalue = 4094;

    // ADC电压
    float vadc = adcvalue * 3.3f / 4095.0f;

    // NTC分压：3.3V -- 10k --+-- ADC -- NTC -- GND
    // Rntc = 10k * Vadc / (3.3 - Vadc)
    float Rntc = 10000.0f * vadc / (3.3f - vadc);

    // 按常见 10k / B3950 NTC 计算
    const float T0 = 298.15f;     // 25°C
    const float R0 = 10000.0f;    // 10k
    const float B  = 3950.0f;

    float temp = 1.0f / (1.0f / T0 + logf(Rntc / R0) / B) - 273.15f;

    return temp;
}

float get_vol(void)
{
    uint16_t adcvalue = get_adc_xchannel(8);

    // 原理图分压：
    // VM -- 20k --+-- ADC
    //             |
    //            1k
    //             |
    //            GND
    //
    // Vadc = VM / 21
    // VM = Vadc * 21
    float vol = adcvalue * 3.3f / 4095.0f * 21.0f;

    return vol;
}