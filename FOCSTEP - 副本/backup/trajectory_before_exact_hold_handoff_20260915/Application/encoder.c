#include "encoder.h"

#include "motor_parameters.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

#define ENCODER_DEGREES_PER_COUNT        (360.0f / (float)MOTOR_ENCODER_COUNTS_PER_REVOLUTION)

/*
 * 速度估算采用自适应测量窗口：
 * 1. 中高速累计到指定计数后立即更新，减小测速延迟；
 * 2. 低速最长等待20ms，避免1ms差分造成15rpm/计数的量化跳变；
 * 3. 100ms没有任何计数变化后确认静止。
 */
#define ENCODER_SPEED_MIN_DELTA_COUNT        (4L)
#define ENCODER_SPEED_MAX_WINDOW_MS          (20U)
#define ENCODER_SPEED_ZERO_TIMEOUT_MS        (100U)
#define ENCODER_SPEED_FILTER_TIME_CONSTANT_MS (4U)
#define ENCODER_MRPM_NUMERATOR               (60000L * 1000L)
#define ENCODER_FIXED_5MS_WINDOW_SIZE         (5U)
#define ENCODER_FIXED_10MS_WINDOW_SIZE        (10U)

typedef struct
{
    Encoder_State_t state;
    volatile int32_t zero_count;
    volatile int32_t last_sample_position;
    volatile int32_t accumulated_delta;
    volatile int32_t measured_delta;
    volatile int32_t raw_speed_mrpm;
    volatile int32_t filtered_speed_mrpm;
    volatile int32_t sample_delta_count;
    volatile int32_t fixed_5ms_speed_mrpm;
    volatile int32_t fixed_10ms_speed_mrpm;
    int32_t delta_5ms_buffer[ENCODER_FIXED_5MS_WINDOW_SIZE];
    int32_t delta_10ms_buffer[ENCODER_FIXED_10MS_WINDOW_SIZE];
    int32_t delta_5ms_sum;
    int32_t delta_10ms_sum;
    uint8_t delta_5ms_index;
    uint8_t delta_10ms_index;
    uint8_t delta_5ms_count;
    uint8_t delta_10ms_count;
    volatile uint16_t window_time_ms;
    volatile uint16_t measurement_period_ms;
    volatile uint16_t no_motion_time_ms;
    volatile uint8_t speed_valid;
    volatile uint8_t initialized;
} Encoder_Context_t;

static Encoder_Context_t s_encoder;

static int32_t Encoder_DeltaToSpeedMrpm(int32_t delta_count,
                                        uint32_t period_ms)
{
    int64_t speed_numerator;

    if (period_ms == 0U)
    {
        return 0L;
    }

    speed_numerator = (int64_t)delta_count * ENCODER_MRPM_NUMERATOR;
    return (int32_t)(speed_numerator /
        ((int64_t)MOTOR_ENCODER_COUNTS_PER_REVOLUTION *
         (int64_t)period_ms));
}

static void Encoder_ResetFixedWindowDiagnostics(void)
{
    uint32_t index;

    s_encoder.sample_delta_count = 0L;
    s_encoder.fixed_5ms_speed_mrpm = 0L;
    s_encoder.fixed_10ms_speed_mrpm = 0L;
    s_encoder.delta_5ms_sum = 0L;
    s_encoder.delta_10ms_sum = 0L;
    s_encoder.delta_5ms_index = 0U;
    s_encoder.delta_10ms_index = 0U;
    s_encoder.delta_5ms_count = 0U;
    s_encoder.delta_10ms_count = 0U;

    for (index = 0U; index < ENCODER_FIXED_5MS_WINDOW_SIZE; index++)
    {
        s_encoder.delta_5ms_buffer[index] = 0L;
    }
    for (index = 0U; index < ENCODER_FIXED_10MS_WINDOW_SIZE; index++)
    {
        s_encoder.delta_10ms_buffer[index] = 0L;
    }
}

static void Encoder_UpdateFixedWindowDiagnostics(int32_t sample_delta)
{
    s_encoder.sample_delta_count = sample_delta;

    s_encoder.delta_5ms_sum -=
        s_encoder.delta_5ms_buffer[s_encoder.delta_5ms_index];
    s_encoder.delta_5ms_buffer[s_encoder.delta_5ms_index] = sample_delta;
    s_encoder.delta_5ms_sum += sample_delta;
    s_encoder.delta_5ms_index++;
    if (s_encoder.delta_5ms_index >= ENCODER_FIXED_5MS_WINDOW_SIZE)
    {
        s_encoder.delta_5ms_index = 0U;
    }
    if (s_encoder.delta_5ms_count < ENCODER_FIXED_5MS_WINDOW_SIZE)
    {
        s_encoder.delta_5ms_count++;
    }
    s_encoder.fixed_5ms_speed_mrpm = Encoder_DeltaToSpeedMrpm(
        s_encoder.delta_5ms_sum, s_encoder.delta_5ms_count);

    s_encoder.delta_10ms_sum -=
        s_encoder.delta_10ms_buffer[s_encoder.delta_10ms_index];
    s_encoder.delta_10ms_buffer[s_encoder.delta_10ms_index] = sample_delta;
    s_encoder.delta_10ms_sum += sample_delta;
    s_encoder.delta_10ms_index++;
    if (s_encoder.delta_10ms_index >= ENCODER_FIXED_10MS_WINDOW_SIZE)
    {
        s_encoder.delta_10ms_index = 0U;
    }
    if (s_encoder.delta_10ms_count < ENCODER_FIXED_10MS_WINDOW_SIZE)
    {
        s_encoder.delta_10ms_count++;
    }
    s_encoder.fixed_10ms_speed_mrpm = Encoder_DeltaToSpeedMrpm(
        s_encoder.delta_10ms_sum, s_encoder.delta_10ms_count);
}

static int32_t Encoder_RawToLogicalPosition(int32_t raw_count)
{
    int32_t hardware_delta = (int32_t)(
        (uint32_t)raw_count - (uint32_t)s_encoder.zero_count);

    if (MOTOR_ENCODER_OUTPUT_CCW_DIRECTION_SIGN < 0L)
    {
        /* 使用无符号补码取负，避免INT32_MIN有符号溢出。 */
        return (int32_t)(0UL - (uint32_t)hardware_delta);
    }
    return hardware_delta;
}

static int32_t Encoder_WrapCount(int32_t count)
{
    int32_t wrapped = count % MOTOR_ENCODER_COUNTS_PER_REVOLUTION;

    if (wrapped < 0)
    {
        wrapped += MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
    }

    return wrapped;
}

void Encoder_Init(void)
{
    uint32_t interrupt_state;
    int32_t raw_count;

    (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    raw_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    s_encoder.initialized = 0U;
    s_encoder.zero_count = raw_count;
    s_encoder.last_sample_position = 0;
    s_encoder.accumulated_delta = 0;
    s_encoder.measured_delta = 0;
    s_encoder.raw_speed_mrpm = 0;
    s_encoder.filtered_speed_mrpm = 0;
    Encoder_ResetFixedWindowDiagnostics();
    s_encoder.window_time_ms = 0U;
    s_encoder.measurement_period_ms = 0U;
    s_encoder.no_motion_time_ms = 0U;
    s_encoder.speed_valid = 0U;
    s_encoder.initialized = 1U;
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }

    s_encoder.state.raw_count = raw_count;
    s_encoder.state.zero_count = raw_count;
    s_encoder.state.position_count = 0;
    s_encoder.state.count_in_revolution = 0;
    s_encoder.state.delta_count = 0;
    s_encoder.state.mechanical_angle_deg = 0.0f;
    s_encoder.state.raw_speed_mrpm = 0;
    s_encoder.state.filtered_speed_mrpm = 0;
    s_encoder.state.speed_rpm = 0.0f;
    s_encoder.state.speed_measurement_period_ms = 0U;
    s_encoder.state.speed_valid = 0U;
}

void Encoder_Update(void)
{
    uint32_t interrupt_state;
    int32_t current_position;

    s_encoder.state.raw_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    current_position = Encoder_RawToLogicalPosition(
        s_encoder.state.raw_count);
    s_encoder.state.position_count = current_position;
    s_encoder.state.count_in_revolution = Encoder_WrapCount(current_position);
    s_encoder.state.mechanical_angle_deg =
        (float)s_encoder.state.count_in_revolution * ENCODER_DEGREES_PER_COUNT;

    /* 多字段快照需保持一致，临界区只包含32位数据复制。 */
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    s_encoder.state.zero_count = s_encoder.zero_count;
    s_encoder.state.delta_count = s_encoder.measured_delta;
    s_encoder.state.raw_speed_mrpm = s_encoder.raw_speed_mrpm;
    s_encoder.state.filtered_speed_mrpm = s_encoder.filtered_speed_mrpm;
    s_encoder.state.speed_measurement_period_ms =
        s_encoder.measurement_period_ms;
    s_encoder.state.speed_valid = s_encoder.speed_valid;
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }

    s_encoder.state.speed_rpm =
        (float)s_encoder.state.filtered_speed_mrpm / 1000.0f;
}

void Encoder_Tick1ms(void)
{
    int32_t current_position;
    int32_t sample_delta;
    int32_t absolute_accumulated_delta;
    int32_t filter_error;
    int64_t speed_numerator;

    if (s_encoder.initialized == 0U)
    {
        return;
    }

    current_position = Encoder_RawToLogicalPosition(
        (int32_t)__HAL_TIM_GET_COUNTER(&htim2));
    sample_delta = current_position - s_encoder.last_sample_position;
    s_encoder.last_sample_position = current_position;
    Encoder_UpdateFixedWindowDiagnostics(sample_delta);
    s_encoder.accumulated_delta += sample_delta;

    if (s_encoder.window_time_ms < UINT16_MAX)
    {
        s_encoder.window_time_ms++;
    }

    if (sample_delta != 0)
    {
        s_encoder.no_motion_time_ms = 0U;
    }
    else if (s_encoder.no_motion_time_ms < UINT16_MAX)
    {
        s_encoder.no_motion_time_ms++;
    }

    if (s_encoder.no_motion_time_ms >= ENCODER_SPEED_ZERO_TIMEOUT_MS)
    {
        s_encoder.accumulated_delta = 0;
        s_encoder.measured_delta = 0;
        s_encoder.raw_speed_mrpm = 0;
        s_encoder.filtered_speed_mrpm = 0;
        s_encoder.window_time_ms = 0U;
        s_encoder.measurement_period_ms = ENCODER_SPEED_ZERO_TIMEOUT_MS;
        s_encoder.speed_valid = 1U;
        return;
    }

    absolute_accumulated_delta = s_encoder.accumulated_delta;
    if (absolute_accumulated_delta < 0)
    {
        absolute_accumulated_delta = -absolute_accumulated_delta;
    }

    if ((absolute_accumulated_delta < ENCODER_SPEED_MIN_DELTA_COUNT) &&
        (s_encoder.window_time_ms < ENCODER_SPEED_MAX_WINDOW_MS))
    {
        return;
    }

    s_encoder.measured_delta = s_encoder.accumulated_delta;
    s_encoder.measurement_period_ms = s_encoder.window_time_ms;
    speed_numerator = (int64_t)s_encoder.accumulated_delta *
                      ENCODER_MRPM_NUMERATOR;
    s_encoder.raw_speed_mrpm = (int32_t)(speed_numerator /
        ((int64_t)MOTOR_ENCODER_COUNTS_PER_REVOLUTION *
         (int64_t)s_encoder.window_time_ms));

    /* 一阶低通的系数随测量窗口变化，使滤波时间常数基本保持一致。 */
    filter_error = s_encoder.raw_speed_mrpm - s_encoder.filtered_speed_mrpm;
    s_encoder.filtered_speed_mrpm += (int32_t)(
        ((int64_t)filter_error * (int64_t)s_encoder.window_time_ms) /
        ((int64_t)ENCODER_SPEED_FILTER_TIME_CONSTANT_MS +
         (int64_t)s_encoder.window_time_ms));

    s_encoder.accumulated_delta = 0;
    s_encoder.window_time_ms = 0U;
    s_encoder.speed_valid = 1U;
}

void Encoder_SetZero(void)
{
    uint32_t interrupt_state = __get_PRIMASK();
    int32_t raw_count;

    __disable_irq();
    raw_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    s_encoder.zero_count = raw_count;
    s_encoder.last_sample_position = 0;
    s_encoder.accumulated_delta = 0;
    s_encoder.measured_delta = 0;
    s_encoder.raw_speed_mrpm = 0;
    s_encoder.filtered_speed_mrpm = 0;
    Encoder_ResetFixedWindowDiagnostics();
    s_encoder.window_time_ms = 0U;
    s_encoder.measurement_period_ms = 0U;
    s_encoder.no_motion_time_ms = 0U;
    s_encoder.speed_valid = 0U;
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }

    s_encoder.state.raw_count = raw_count;
    s_encoder.state.zero_count = raw_count;
    s_encoder.state.position_count = 0;
    s_encoder.state.count_in_revolution = 0;
    s_encoder.state.delta_count = 0;
    s_encoder.state.mechanical_angle_deg = 0.0f;
    s_encoder.state.raw_speed_mrpm = 0;
    s_encoder.state.filtered_speed_mrpm = 0;
    s_encoder.state.speed_rpm = 0.0f;
    s_encoder.state.speed_measurement_period_ms = 0U;
    s_encoder.state.speed_valid = 0U;
}

int32_t Encoder_GetPositionCountFast(void)
{
    return Encoder_RawToLogicalPosition(
        (int32_t)__HAL_TIM_GET_COUNTER(&htim2));
}

int32_t Encoder_GetControlSpeedMrpmFast(void)
{
    return s_encoder.fixed_5ms_speed_mrpm;
}

void Encoder_GetSpeedDiagnosticFast(Encoder_SpeedDiagnostic_t *diagnostic)
{
    if (diagnostic == 0)
    {
        return;
    }

    diagnostic->sample_delta_count = s_encoder.sample_delta_count;
    diagnostic->adaptive_speed_mrpm = s_encoder.filtered_speed_mrpm;
    diagnostic->fixed_5ms_speed_mrpm = s_encoder.fixed_5ms_speed_mrpm;
    diagnostic->fixed_10ms_speed_mrpm = s_encoder.fixed_10ms_speed_mrpm;
}

const Encoder_State_t *Encoder_GetState(void)
{
    return &s_encoder.state;
}
