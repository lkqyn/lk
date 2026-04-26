#include "adc_task.h"
#include "adc.h"
#include "math.h"

float offet_I[2] = {0.0f, 0.0f};

uint16_t get_adc_xchannel(uint8_t channel)
{
    uint16_t adcvalue = 0;
    ADC_ChannelConfTypeDef sConfig = {0};

    switch(channel)
    {
        case 8:
            sConfig.Channel = ADC_CHANNEL_8;
            break;

        case 9:
            sConfig.Channel = ADC_CHANNEL_9;
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
    HAL_ADC_PollForConversion(&hadc1, 0xFFFF);
    adcvalue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return adcvalue;
}

float get_phase_current(uint32_t rank, uint8_t index)
{
    float raw = (float)HAL_ADCEx_InjectedGetValue(&hadc1, rank);
    float current = (raw - CURRENT_VREF_ADC) * ADCtoI;

    return current - offet_I[index];
}

void calibrate_current_offset(uint16_t sample_times)
{
    float sum_a = 0.0f;
    float sum_b = 0.0f;

    if(sample_times == 0)
        sample_times = 1;

    for(uint16_t i = 0; i < sample_times; i++)
    {
        HAL_ADCEx_InjectedStart(&hadc1);
        HAL_ADCEx_InjectedPollForConversion(&hadc1, 10);
        HAL_ADCEx_InjectedPollForConversion(&hadc1, 10);
        sum_a += ((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) - CURRENT_VREF_ADC) * ADCtoI;
        sum_b += ((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - CURRENT_VREF_ADC) * ADCtoI;
        HAL_ADCEx_InjectedStop(&hadc1);
        HAL_Delay(1);
    }

    offet_I[0] = sum_a / sample_times;
    offet_I[1] = sum_b / sample_times;
}

float get_temp(void)
{
    uint16_t adcvalue = get_adc_xchannel(9);

    if(adcvalue == 0) adcvalue = 1;
    if(adcvalue >= 4095) adcvalue = 4094;

    float vadc = adcvalue * ADC_REF_VOLTAGE / ADC_FULL_SCALE;
    float Rntc = 10000.0f * vadc / (ADC_REF_VOLTAGE - vadc);

    const float T0 = 298.15f;
    const float R0 = 10000.0f;
    const float B  = 3950.0f;

    return 1.0f / (1.0f / T0 + logf(Rntc / R0) / B) - 273.15f;
}

float get_vol(void)
{
    uint16_t adcvalue = get_adc_xchannel(8);

    return adcvalue * ADC_REF_VOLTAGE / ADC_FULL_SCALE * 21.0f;
}
