#include "adc_task.h"
#include "adc.h"
#include "math.h"

float offet_I[2] = {0.0f, 0.0f};

uint16_t get_adc_xchannel(uint8_t CHANNEL)   // 获取某一路ADC值
{
    uint16_t adcvalue = 0;
    ADC_ChannelConfTypeDef sConfig = {0};

    switch (CHANNEL)
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

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 0xFFFF);   // 等待ADC转换完成
    adcvalue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return adcvalue;
}

float get_temp(void)
{
    uint16_t adcvalue = get_adc_xchannel(9);   // PB1 -> ADC1_IN9

    if (adcvalue == 0 || adcvalue >= 4095)
        return -1000.0f;

    /* 你的原理图：
     * 3.3V --- R32(10k) --- ADC点 --- NTC --- GND
     *
     * Rntc = 10000 * adc / (4095 - adc)
     */

    const float R_FIXED = 10000.0f;   // R32 = 10k
    const float R0      = 10000.0f;   // NTC 25℃ 时 10k
    const float T0      = 298.15f;    // 25℃ = 298.15K
    const float BETA    = 3950.0f;    // 常见10K NTC，若NTC参数不同就改这里

    float Rntc = R_FIXED * (float)adcvalue / (4095.0f - (float)adcvalue);

    float tempK = 1.0f / (1.0f / T0 + logf(Rntc / R0) / BETA);
    float tempC = tempK - 273.15f;

    return tempC;
}

float get_vol(void)
{
    uint16_t adcvalue = get_adc_xchannel(8);   // PB0 -> ADC1_IN8
    float vol = 0.0f;

    /* 你的原理图：
     * VM --- R22(20k) --- ADC点 --- R25(1k) --- GND
     *
     * Vadc = VM / 21
     * VM   = adc * 3.3 / 4095 * 21
     */
    vol = ((float)adcvalue * 3.3f / 4095.0f) * 21.0f;

    return vol;
}