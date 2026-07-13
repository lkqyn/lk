#include "current.h"

#include "adc.h"

#define CURRENT_ADC_MAX             (4095.0f)
#define CURRENT_ADC_VREF            (3.3f)
#define CURRENT_SHUNT_OHM           (0.010f)
#define CURRENT_INA_GAIN            (20.0f)
#define CURRENT_LPF_ALPHA           (0.12f)
#define CURRENT_BUS_VOLTAGE_V       (24.0f)
#define CURRENT_DMA_CHANNELS        (2U)
#define CURRENT_DMA_SAMPLES         (32U)
#define CURRENT_DMA_LENGTH          (CURRENT_DMA_CHANNELS * CURRENT_DMA_SAMPLES)

static Current_State_t s_current;
static uint8_t s_current_filter_ready;
static uint8_t s_current_sampling_started;
static uint16_t s_current_dma_buffer[CURRENT_DMA_LENGTH];

static void Current_ApplyCurrentRaw(uint16_t raw_a, uint16_t raw_b)
{
    float delta_a;
    float delta_b;
    float ia_sample;
    float ib_sample;

    s_current.raw_a = raw_a;
    s_current.raw_b = raw_b;
    s_current.sample_count++;

    delta_a = (float)s_current.raw_a - s_current.offset_a;
    delta_b = (float)s_current.raw_b - s_current.offset_b;

    ia_sample =
        (-delta_a * CURRENT_ADC_VREF) /
        (CURRENT_ADC_MAX * CURRENT_SHUNT_OHM * CURRENT_INA_GAIN);
    ib_sample =
        (delta_b * CURRENT_ADC_VREF) /
        (CURRENT_ADC_MAX * CURRENT_SHUNT_OHM * CURRENT_INA_GAIN);

    if (s_current_filter_ready == 0U)
    {
        s_current.ia = ia_sample;
        s_current.ib = ib_sample;
        s_current_filter_ready = 1U;
    }
    else
    {
        s_current.ia += CURRENT_LPF_ALPHA * (ia_sample - s_current.ia);
        s_current.ib += CURRENT_LPF_ALPHA * (ib_sample - s_current.ib);
    }
}

static uint16_t Current_ReadRegularChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t raw = 0U;

    (void)HAL_ADC_Stop_DMA(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);

    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        return 0U;
    }

    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 2U) == HAL_OK)
    {
        raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }

    (void)HAL_ADC_Stop(&hadc1);
    return raw;
}

void Current_Init(void)
{
    s_current.raw_a = 0U;
    s_current.raw_b = 0U;
    s_current.raw_bus = 0U;
    s_current.raw_ntc = 0U;
    s_current.offset_a = CURRENT_ADC_MAX * 0.5f;
    s_current.offset_b = CURRENT_ADC_MAX * 0.5f;
    s_current.ia = 0.0f;
    s_current.ib = 0.0f;
    s_current.bus_v = CURRENT_BUS_VOLTAGE_V;
    s_current.ntc_v = 0.0f;
    s_current.sample_count = 0U;
    s_current.debug_adc_start = 0U;
    s_current_filter_ready = 0U;
    s_current_sampling_started = 0U;
}

void Current_CalibrateOffset(uint16_t sample_count)
{
    uint32_t sum_a = 0U;
    uint32_t sum_b = 0U;
    uint16_t i;

    if (sample_count == 0U)
    {
        return;
    }

    for (i = 0U; i < sample_count; i++)
    {
        s_current.raw_a = Current_ReadRegularChannel(ADC_CHANNEL_14);
        s_current.raw_b = Current_ReadRegularChannel(ADC_CHANNEL_15);
        sum_a += s_current.raw_a;
        sum_b += s_current.raw_b;
    }

    s_current.offset_a = (float)sum_a / (float)sample_count;
    s_current.offset_b = (float)sum_b / (float)sample_count;
    s_current_filter_ready = 0U;
    s_current.ia = 0.0f;
    s_current.ib = 0.0f;
}

void Current_StartSampling(void)
{
    uint16_t i;

    (void)HAL_ADC_Stop_DMA(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
    MX_ADC1_Init();
    for (i = 0U; i < CURRENT_DMA_LENGTH; i++)
    {
        if ((i & 1U) == 0U)
        {
            s_current_dma_buffer[i] = (uint16_t)s_current.offset_a;
        }
        else
        {
            s_current_dma_buffer[i] = (uint16_t)s_current.offset_b;
        }
    }

    s_current.debug_adc_start =
        (uint32_t)HAL_ADC_Start_DMA(&hadc1,
                                    (uint32_t *)s_current_dma_buffer,
                                    CURRENT_DMA_LENGTH);
    s_current_sampling_started = (s_current.debug_adc_start == HAL_OK) ? 1U : 0U;
}

void Current_Update(void)
{
    uint32_t sum_a = 0U;
    uint32_t sum_b = 0U;
    uint16_t raw_a;
    uint16_t raw_b;
    uint16_t i;

    if (s_current_sampling_started == 0U)
    {
        return;
    }

    for (i = 0U; i < CURRENT_DMA_SAMPLES; i++)
    {
        sum_a += s_current_dma_buffer[(i * CURRENT_DMA_CHANNELS) + 0U];
        sum_b += s_current_dma_buffer[(i * CURRENT_DMA_CHANNELS) + 1U];
    }

    raw_a = (uint16_t)(sum_a / CURRENT_DMA_SAMPLES);
    raw_b = (uint16_t)(sum_b / CURRENT_DMA_SAMPLES);
    Current_ApplyCurrentRaw(raw_a, raw_b);
}

const Current_State_t *Current_GetState(void)
{
    return &s_current;
}
