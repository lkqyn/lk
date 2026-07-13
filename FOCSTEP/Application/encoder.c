#include "encoder.h"

#include "tim.h"

#define ENCODER_TWO_PI                 (6.28318530717958647692f)
#define ENCODER_RAD_TO_DEG             (57.2957795130823208768f)
#define ENCODER_RPM_PER_COUNT_PER_SEC  (60.0f / (float)ENCODER_COUNT_PER_REV)
#define ENCODER_SPEED_WINDOW_SIZE      (10U)
#define ENCODER_SPEED_LPF_ALPHA        (0.15f)

static Encoder_State_t s_encoder;
static int32_t s_encoder_last_relative_count;
static int32_t s_encoder_delta_window[ENCODER_SPEED_WINDOW_SIZE];
static float s_encoder_dt_window[ENCODER_SPEED_WINDOW_SIZE];
static int32_t s_encoder_delta_sum;
static float s_encoder_dt_sum;
static uint8_t s_encoder_speed_index;
static uint8_t s_encoder_speed_count;
static uint8_t s_encoder_speed_valid;

static int32_t Encoder_WrapCountInRev(int32_t count)
{
    int32_t wrapped = count % ENCODER_COUNT_PER_REV;

    if (wrapped < 0)
    {
        wrapped += ENCODER_COUNT_PER_REV;
    }

    return wrapped;
}

static float Encoder_NormalizeAngle(float angle_rad)
{
    while (angle_rad >= ENCODER_TWO_PI)
    {
        angle_rad -= ENCODER_TWO_PI;
    }

    while (angle_rad < 0.0f)
    {
        angle_rad += ENCODER_TWO_PI;
    }

    return angle_rad;
}

void Encoder_Init(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    Encoder_Update();
    s_encoder_last_relative_count = s_encoder.relative_count;
    s_encoder_speed_valid = 0U;
}

void Encoder_Update(void)
{
    int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t relative_count = count - s_encoder.zero_count;
    int32_t count_in_rev = Encoder_WrapCountInRev(relative_count);

    s_encoder.count = count;
    s_encoder.relative_count = relative_count;
    s_encoder.count_in_rev = count_in_rev;
    s_encoder.mechanical_angle_rad = ((float)count_in_rev * ENCODER_TWO_PI) / (float)ENCODER_COUNT_PER_REV;
    s_encoder.mechanical_angle_deg = s_encoder.mechanical_angle_rad * ENCODER_RAD_TO_DEG;
    s_encoder.electrical_angle_rad = Encoder_NormalizeAngle(
        ENCODER_DIRECTION * ENCODER_ELECTRICAL_CYCLES * s_encoder.mechanical_angle_rad);
}

void Encoder_UpdateSpeed(float dt_s)
{
    int32_t delta_count;

    if (dt_s <= 0.0f)
    {
        return;
    }

    if (s_encoder_speed_valid == 0U)
    {
        s_encoder_last_relative_count = s_encoder.relative_count;
        s_encoder_speed_valid = 1U;
        return;
    }

    delta_count = s_encoder.relative_count - s_encoder_last_relative_count;
    s_encoder_last_relative_count = s_encoder.relative_count;

    if (s_encoder_speed_count >= ENCODER_SPEED_WINDOW_SIZE)
    {
        s_encoder_delta_sum -= s_encoder_delta_window[s_encoder_speed_index];
        s_encoder_dt_sum -= s_encoder_dt_window[s_encoder_speed_index];
    }
    else
    {
        s_encoder_speed_count++;
    }

    s_encoder_delta_window[s_encoder_speed_index] = delta_count;
    s_encoder_dt_window[s_encoder_speed_index] = dt_s;
    s_encoder_delta_sum += delta_count;
    s_encoder_dt_sum += dt_s;

    s_encoder_speed_index++;
    if (s_encoder_speed_index >= ENCODER_SPEED_WINDOW_SIZE)
    {
        s_encoder_speed_index = 0U;
    }

    s_encoder.delta_count = delta_count;
    s_encoder.speed_raw_rpm =
        ENCODER_DIRECTION * ((float)delta_count / dt_s) * ENCODER_RPM_PER_COUNT_PER_SEC;

    if (s_encoder_dt_sum > 0.0f)
    {
        s_encoder.speed_window_rpm =
            ENCODER_DIRECTION *
            ((float)s_encoder_delta_sum / s_encoder_dt_sum) *
            ENCODER_RPM_PER_COUNT_PER_SEC;
    }
    else
    {
        s_encoder.speed_window_rpm = 0.0f;
    }

    s_encoder.speed_lpf_rpm +=
        ENCODER_SPEED_LPF_ALPHA * (s_encoder.speed_window_rpm - s_encoder.speed_lpf_rpm);
}

void Encoder_SetZeroCurrentPosition(void)
{
    uint8_t i;

    s_encoder.zero_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    Encoder_Update();
    s_encoder_last_relative_count = s_encoder.relative_count;
    s_encoder_delta_sum = 0;
    s_encoder_dt_sum = 0.0f;
    s_encoder_speed_index = 0U;
    s_encoder_speed_count = 0U;
    s_encoder_speed_valid = 0U;
    s_encoder.delta_count = 0;
    s_encoder.speed_raw_rpm = 0.0f;
    s_encoder.speed_window_rpm = 0.0f;
    s_encoder.speed_lpf_rpm = 0.0f;

    for (i = 0U; i < ENCODER_SPEED_WINDOW_SIZE; i++)
    {
        s_encoder_delta_window[i] = 0;
        s_encoder_dt_window[i] = 0.0f;
    }
}

int32_t Encoder_GetCount(void)
{
    return s_encoder.count;
}

const Encoder_State_t *Encoder_GetState(void)
{
    return &s_encoder;
}
