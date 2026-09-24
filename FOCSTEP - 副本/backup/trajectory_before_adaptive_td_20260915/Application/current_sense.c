#include "current_sense.h"

#include "adc.h"
#include "tim.h"

#define CURRENT_SENSE_INJECTED_CHANNEL_COUNT  (2U)
#define CURRENT_SENSE_CALIBRATION_SAMPLES      (1024U)
#define CURRENT_SENSE_CALIBRATION_PERIOD_MS    (1U)

/*
 * INA240A1增益为20，分流电阻为10mΩ，电流通道灵敏度为0.2V/A。
 * 12位ADC、3.3V参考下，每个ADC计数约对应4.029mA。
 */
#define CURRENT_SENSE_MA_NUMERATOR            (16500L)
#define CURRENT_SENSE_MA_DENOMINATOR          (4095L)

/* REF1=3.3V、REF2=GND，零电流理论值为ADC中点。 */
#define CURRENT_SENSE_OFFSET_MIN_RAW           (1500U)
#define CURRENT_SENSE_OFFSET_MAX_RAW           (2600U)

typedef struct
{
    CurrentSense_State_t state;
    CurrentSense_SampleCallback_t sample_callback;
    void *sample_callback_context;
    uint8_t dma_started;
} CurrentSense_Context_t;

static CurrentSense_Context_t s_current_sense;

static int32_t CurrentSense_RawDeltaToMilliamp(int32_t raw_delta)
{
    return (raw_delta * CURRENT_SENSE_MA_NUMERATOR) /
           CURRENT_SENSE_MA_DENOMINATOR;
}

static void CurrentSense_UpdateFromRaw(uint16_t raw_a, uint16_t raw_b)
{
    s_current_sense.state.raw_a = raw_a;
    s_current_sense.state.raw_b = raw_b;

    if (s_current_sense.state.calibrated == 0U)
    {
        s_current_sense.state.current_a_ma = 0L;
        s_current_sense.state.current_b_ma = 0L;
        return;
    }

    /*
     * 低电流极性实测确认：A相INA240采样方向与软件正方向相反，
     * B相采样方向与软件正方向一致。
     */
    s_current_sense.state.current_a_ma =
        CurrentSense_RawDeltaToMilliamp((int32_t)s_current_sense.state.offset_a -
                                        (int32_t)raw_a);
    s_current_sense.state.current_b_ma =
        CurrentSense_RawDeltaToMilliamp((int32_t)raw_b -
                                        (int32_t)s_current_sense.state.offset_b);
}

static uint8_t CurrentSense_ConfigureInjectedChannels(void)
{
    ADC_InjectionConfTypeDef config = {0};

    config.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
    config.InjectedOffset = 0U;
    config.InjectedNbrOfConversion = CURRENT_SENSE_INJECTED_CHANNEL_COUNT;
    config.InjectedDiscontinuousConvMode = DISABLE;
    config.AutoInjectedConv = DISABLE;
    config.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T4_CC2;
    config.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;

    config.InjectedChannel = ADC_CHANNEL_14;
    config.InjectedRank = ADC_INJECTED_RANK_1;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &config) != HAL_OK)
    {
        return 0U;
    }

    config.InjectedChannel = ADC_CHANNEL_15;
    config.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &config) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

void CurrentSense_Init(void)
{
    s_current_sense.sample_callback = 0;
    s_current_sense.sample_callback_context = 0;
    s_current_sense.dma_started = 0U;
    s_current_sense.state.calibrated = 0U;
    s_current_sense.state.offset_valid = 0U;
    s_current_sense.state.synchronized = 0U;
    s_current_sense.state.synchronized_sample_count = 0U;

    if (CurrentSense_ConfigureInjectedChannels() == 0U)
    {
        return;
    }

    if (ADC_RegularStart() == 0U)
    {
        return;
    }

    s_current_sense.dma_started = 1U;
    HAL_Delay(10U);
    (void)CurrentSense_CalibrateOffsets();
}

void CurrentSense_SetSampleCallback(CurrentSense_SampleCallback_t callback,
                                    void *context)
{
    if (s_current_sense.state.synchronized != 0U)
    {
        return;
    }

    s_current_sense.sample_callback = callback;
    s_current_sense.sample_callback_context = context;
}

void CurrentSense_Update(void)
{
    uint16_t raw_a = ADC_RegularGetRaw(ADC_REGULAR_CURRENT_A);
    uint16_t raw_b = ADC_RegularGetRaw(ADC_REGULAR_CURRENT_B);

    if (s_current_sense.state.synchronized != 0U)
    {
        return;
    }

    CurrentSense_UpdateFromRaw(raw_a, raw_b);
}

uint8_t CurrentSense_CalibrateOffsets(void)
{
    uint32_t sum_a = 0U;
    uint32_t sum_b = 0U;
    uint16_t min_a = UINT16_MAX;
    uint16_t min_b = UINT16_MAX;
    uint16_t max_a = 0U;
    uint16_t max_b = 0U;
    uint32_t sample;

    if (s_current_sense.dma_started == 0U)
    {
        return 0U;
    }

    for (sample = 0U; sample < CURRENT_SENSE_CALIBRATION_SAMPLES; sample++)
    {
        uint16_t raw_a = ADC_RegularGetRaw(ADC_REGULAR_CURRENT_A);
        uint16_t raw_b = ADC_RegularGetRaw(ADC_REGULAR_CURRENT_B);

        sum_a += raw_a;
        sum_b += raw_b;

        if (raw_a < min_a)
        {
            min_a = raw_a;
        }
        if (raw_a > max_a)
        {
            max_a = raw_a;
        }
        if (raw_b < min_b)
        {
            min_b = raw_b;
        }
        if (raw_b > max_b)
        {
            max_b = raw_b;
        }

        HAL_Delay(CURRENT_SENSE_CALIBRATION_PERIOD_MS);
    }

    s_current_sense.state.offset_a =
        (uint16_t)((sum_a + (CURRENT_SENSE_CALIBRATION_SAMPLES / 2U)) /
                   CURRENT_SENSE_CALIBRATION_SAMPLES);
    s_current_sense.state.offset_b =
        (uint16_t)((sum_b + (CURRENT_SENSE_CALIBRATION_SAMPLES / 2U)) /
                   CURRENT_SENSE_CALIBRATION_SAMPLES);
    s_current_sense.state.calibration_span_a = (uint16_t)(max_a - min_a);
    s_current_sense.state.calibration_span_b = (uint16_t)(max_b - min_b);
    s_current_sense.state.calibrated = 1U;
    s_current_sense.state.offset_valid =
        ((s_current_sense.state.offset_a >= CURRENT_SENSE_OFFSET_MIN_RAW) &&
         (s_current_sense.state.offset_a <= CURRENT_SENSE_OFFSET_MAX_RAW) &&
         (s_current_sense.state.offset_b >= CURRENT_SENSE_OFFSET_MIN_RAW) &&
         (s_current_sense.state.offset_b <= CURRENT_SENSE_OFFSET_MAX_RAW)) ? 1U : 0U;

    CurrentSense_Update();
    return 1U;
}

uint8_t CurrentSense_StartSynchronizedSampling(void)
{
    if ((s_current_sense.dma_started == 0U) ||
        (s_current_sense.state.offset_valid == 0U))
    {
        return 0U;
    }

    if (s_current_sense.state.synchronized != 0U)
    {
        return 1U;
    }

    s_current_sense.state.synchronized_sample_count = 0U;

    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
        return 0U;
    }

    s_current_sense.state.synchronized = 1U;
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
    {
        s_current_sense.state.synchronized = 0U;
        (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
        return 0U;
    }

    return 1U;
}

void CurrentSense_StopSynchronizedSampling(void)
{
    if (s_current_sense.state.synchronized == 0U)
    {
        return;
    }

    (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
    s_current_sense.state.synchronized = 0U;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint16_t raw_a;
    uint16_t raw_b;

    if ((hadc == 0) || (hadc->Instance != ADC1) ||
        (s_current_sense.state.synchronized == 0U))
    {
        return;
    }

    raw_a = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    raw_b = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    CurrentSense_UpdateFromRaw(raw_a, raw_b);
    s_current_sense.state.synchronized_sample_count++;

    if (s_current_sense.sample_callback != 0)
    {
        s_current_sense.sample_callback(s_current_sense.sample_callback_context);
    }
}

const CurrentSense_State_t *CurrentSense_GetState(void)
{
    return &s_current_sense.state;
}
