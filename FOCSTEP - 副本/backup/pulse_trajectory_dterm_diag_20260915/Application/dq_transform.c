#include "dq_transform.h"

#include "motor_parameters.h"

#include <math.h>

#define DQ_TRANSFORM_ELECTRICAL_COUNTS       \
    (MOTOR_ENCODER_COUNTS_PER_REVOLUTION)
#define DQ_TRANSFORM_FULL_CYCLE_INDEXES      (1024U)
#define DQ_TRANSFORM_QUARTER_INDEXES         (256U)
#define DQ_TRANSFORM_Q15_SCALE               (32768L)
#define DQ_TRANSFORM_Q15_MAXIMUM             (32767L)
#define DQ_TRANSFORM_HALF_PI_RAD             (1.5707963267948966192f)

static int16_t s_sine_quarter_q15[DQ_TRANSFORM_QUARTER_INDEXES + 1U];

static int32_t DqTransform_RoundFloat(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                             (int32_t)(value - 0.5f);
}

static int32_t DqTransform_RoundQ15(int64_t value)
{
    value += (value >= 0LL) ? (DQ_TRANSFORM_Q15_SCALE / 2L) :
                              -(DQ_TRANSFORM_Q15_SCALE / 2L);
    return (int32_t)(value / DQ_TRANSFORM_Q15_SCALE);
}

static uint32_t DqTransform_NormalizeElectricalCount(int32_t count)
{
    int32_t normalized = count % DQ_TRANSFORM_ELECTRICAL_COUNTS;

    if (normalized < 0L)
    {
        normalized += DQ_TRANSFORM_ELECTRICAL_COUNTS;
    }
    return (uint32_t)normalized;
}

static void DqTransform_GetSinCosQ15FromPhaseIndex(int32_t requested_index,
                                                   int32_t *sine_q15,
                                                   int32_t *cosine_q15)
{
    uint32_t phase_index =
        (uint32_t)requested_index & (DQ_TRANSFORM_FULL_CYCLE_INDEXES - 1U);
    uint32_t quadrant;
    uint32_t index;

    quadrant = phase_index / DQ_TRANSFORM_QUARTER_INDEXES;
    index = phase_index % DQ_TRANSFORM_QUARTER_INDEXES;

    switch (quadrant)
    {
        case 0U:
            *sine_q15 = s_sine_quarter_q15[index];
            *cosine_q15 = s_sine_quarter_q15[
                DQ_TRANSFORM_QUARTER_INDEXES - index];
            break;

        case 1U:
            *sine_q15 = s_sine_quarter_q15[
                DQ_TRANSFORM_QUARTER_INDEXES - index];
            *cosine_q15 = -s_sine_quarter_q15[index];
            break;

        case 2U:
            *sine_q15 = -s_sine_quarter_q15[index];
            *cosine_q15 = -s_sine_quarter_q15[
                DQ_TRANSFORM_QUARTER_INDEXES - index];
            break;

        default:
            *sine_q15 = -s_sine_quarter_q15[
                DQ_TRANSFORM_QUARTER_INDEXES - index];
            *cosine_q15 = s_sine_quarter_q15[index];
            break;
    }
}

static int32_t DqTransform_ElectricalCountToPhaseIndex(
    int32_t electrical_count)
{
    uint32_t normalized =
        DqTransform_NormalizeElectricalCount(electrical_count);

    return (int32_t)((((normalized * DQ_TRANSFORM_FULL_CYCLE_INDEXES) +
                       ((uint32_t)DQ_TRANSFORM_ELECTRICAL_COUNTS / 2U)) /
                      (uint32_t)DQ_TRANSFORM_ELECTRICAL_COUNTS) &
                     (DQ_TRANSFORM_FULL_CYCLE_INDEXES - 1U));
}

void DqTransform_Init(void)
{
    uint32_t index;

    for (index = 0U; index <= DQ_TRANSFORM_QUARTER_INDEXES; index++)
    {
        float angle_rad = DQ_TRANSFORM_HALF_PI_RAD * (float)index /
                          (float)DQ_TRANSFORM_QUARTER_INDEXES;
        int32_t value = DqTransform_RoundFloat(
            sinf(angle_rad) * (float)DQ_TRANSFORM_Q15_MAXIMUM);

        s_sine_quarter_q15[index] = (int16_t)value;
    }
}

void DqTransform_Park(int32_t alpha,
                      int32_t beta,
                      int32_t electrical_count,
                      int32_t *direct,
                      int32_t *quadrature)
{
    DqTransform_ParkPhaseIndex(
        alpha,
        beta,
        DqTransform_ElectricalCountToPhaseIndex(electrical_count),
        direct,
        quadrature);
}

void DqTransform_ParkPhaseIndex(int32_t alpha,
                                int32_t beta,
                                int32_t phase_index,
                                int32_t *direct,
                                int32_t *quadrature)
{
    int32_t sine_q15;
    int32_t cosine_q15;

    if ((direct == 0) || (quadrature == 0))
    {
        return;
    }

    DqTransform_GetSinCosQ15FromPhaseIndex(
        phase_index, &sine_q15, &cosine_q15);
    *direct = DqTransform_RoundQ15(
        ((int64_t)alpha * cosine_q15) + ((int64_t)beta * sine_q15));
    *quadrature = DqTransform_RoundQ15(
        -((int64_t)alpha * sine_q15) + ((int64_t)beta * cosine_q15));
}

void DqTransform_InversePark(int32_t direct,
                             int32_t quadrature,
                             int32_t electrical_count,
                             int32_t *alpha,
                             int32_t *beta)
{
    DqTransform_InverseParkPhaseIndex(
        direct,
        quadrature,
        DqTransform_ElectricalCountToPhaseIndex(electrical_count),
        alpha,
        beta);
}

void DqTransform_InverseParkPhaseIndex(int32_t direct,
                                       int32_t quadrature,
                                       int32_t phase_index,
                                       int32_t *alpha,
                                       int32_t *beta)
{
    int32_t sine_q15;
    int32_t cosine_q15;

    if ((alpha == 0) || (beta == 0))
    {
        return;
    }

    DqTransform_GetSinCosQ15FromPhaseIndex(
        phase_index, &sine_q15, &cosine_q15);
    *alpha = DqTransform_RoundQ15(
        ((int64_t)direct * cosine_q15) -
        ((int64_t)quadrature * sine_q15));
    *beta = DqTransform_RoundQ15(
        ((int64_t)direct * sine_q15) +
        ((int64_t)quadrature * cosine_q15));
}
