#include "current_controller.h"

#include "motor_parameters.h"

#include <math.h>

#define CURRENT_CONTROLLER_TWO_PI  (6.2831853071795864769f)

static int32_t CurrentController_RoundToInt32(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                             (int32_t)(value - 0.5f);
}

static float CurrentController_ClampFloat(float value,
                                           float minimum,
                                           float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

uint8_t CurrentController_Init(CurrentController_t *controller,
                               float bandwidth_hz,
                               float sample_frequency_hz,
                               int32_t maximum_voltage_mv)
{
    float angular_bandwidth_rad_s;

    if ((controller == 0) || (bandwidth_hz <= 0.0f) ||
        (sample_frequency_hz <= 0.0f) || (maximum_voltage_mv <= 0L))
    {
        return 0U;
    }

    angular_bandwidth_rad_s = CURRENT_CONTROLLER_TWO_PI * bandwidth_hz;

    /*
     * 对象模型为1/(Ls+R)，采用零极点对消设计：
     * Kp=L*wc，Ki=R*wc。使用mA和mV后，数值比例与A、V单位相同。
     */
    controller->proportional_gain_mv_per_ma =
        MOTOR_PHASE_INDUCTANCE_H * angular_bandwidth_rad_s;
    controller->integral_gain_mv_per_ma_per_step =
        (MOTOR_PHASE_RESISTANCE_OHM * angular_bandwidth_rad_s) /
        sample_frequency_hz;
    controller->maximum_voltage_mv = maximum_voltage_mv;
    CurrentController_Reset(controller);
    return 1U;
}

uint8_t CurrentController_SetMaximumVoltage(
    CurrentController_t *controller,
    int32_t maximum_voltage_mv)
{
    if ((controller == 0) || (maximum_voltage_mv <= 0L))
    {
        return 0U;
    }

    controller->maximum_voltage_mv = maximum_voltage_mv;
    return 1U;
}

void CurrentController_Reset(CurrentController_t *controller)
{
    if (controller == 0)
    {
        return;
    }

    controller->integral_axis_1_mv = 0.0f;
    controller->integral_axis_2_mv = 0.0f;
    controller->output_axis_1_mv = 0L;
    controller->output_axis_2_mv = 0L;
    controller->saturated = 0U;
    controller->axis_1_priority_active = 0U;
}

void CurrentController_Step(CurrentController_t *controller,
                            int32_t target_axis_1_ma,
                            int32_t target_axis_2_ma,
                            int32_t measured_axis_1_ma,
                            int32_t measured_axis_2_ma)
{
    CurrentController_StepWithFeedforward(controller,
                                          target_axis_1_ma,
                                          target_axis_2_ma,
                                          measured_axis_1_ma,
                                          measured_axis_2_ma,
                                          0L,
                                          0L,
                                          0U);
}

void CurrentController_StepWithFeedforward(
    CurrentController_t *controller,
    int32_t target_axis_1_ma,
    int32_t target_axis_2_ma,
    int32_t measured_axis_1_ma,
    int32_t measured_axis_2_ma,
    int32_t feedforward_axis_1_mv,
    int32_t feedforward_axis_2_mv,
    uint8_t axis_1_priority_enabled)
{
    float error_axis_1_ma;
    float error_axis_2_ma;
    float proportional_axis_1_mv;
    float proportional_axis_2_mv;
    float candidate_integral_axis_1_mv;
    float candidate_integral_axis_2_mv;
    float unsaturated_axis_1_mv;
    float unsaturated_axis_2_mv;
    float output_axis_1_mv;
    float output_axis_2_mv;
    float magnitude_squared;
    float maximum_squared;

    if (controller == 0)
    {
        return;
    }

    error_axis_1_ma = (float)(target_axis_1_ma - measured_axis_1_ma);
    error_axis_2_ma = (float)(target_axis_2_ma - measured_axis_2_ma);
    proportional_axis_1_mv =
        controller->proportional_gain_mv_per_ma * error_axis_1_ma;
    proportional_axis_2_mv =
        controller->proportional_gain_mv_per_ma * error_axis_2_ma;
    candidate_integral_axis_1_mv = controller->integral_axis_1_mv +
        (controller->integral_gain_mv_per_ma_per_step * error_axis_1_ma);
    candidate_integral_axis_2_mv = controller->integral_axis_2_mv +
        (controller->integral_gain_mv_per_ma_per_step * error_axis_2_ma);
    unsaturated_axis_1_mv =
        proportional_axis_1_mv + candidate_integral_axis_1_mv +
        (float)feedforward_axis_1_mv;
    unsaturated_axis_2_mv =
        proportional_axis_2_mv + candidate_integral_axis_2_mv +
        (float)feedforward_axis_2_mv;

    magnitude_squared = (unsaturated_axis_1_mv * unsaturated_axis_1_mv) +
                        (unsaturated_axis_2_mv * unsaturated_axis_2_mv);
    maximum_squared = (float)controller->maximum_voltage_mv *
                      (float)controller->maximum_voltage_mv;
    output_axis_1_mv = unsaturated_axis_1_mv;
    output_axis_2_mv = unsaturated_axis_2_mv;
    controller->saturated = 0U;
    controller->axis_1_priority_active = 0U;

    if (magnitude_squared > maximum_squared)
    {
        if ((axis_1_priority_enabled != 0U) &&
            (unsaturated_axis_1_mv < 0.0f))
        {
            float remaining_voltage_squared;
            float remaining_axis_2_voltage_mv;

            /*
             * 弱磁饱和时先保留负d轴电压，再将电压圆内的
             * 剩余容量分配给q轴。这可避免径向缩放使Id长期跟不上，
             * 但必然会在总电压不足时暂时牺牲部分q轴转矩电压。
             */
            output_axis_1_mv = CurrentController_ClampFloat(
                unsaturated_axis_1_mv,
                -(float)controller->maximum_voltage_mv,
                (float)controller->maximum_voltage_mv);
            remaining_voltage_squared = maximum_squared -
                (output_axis_1_mv * output_axis_1_mv);
            if (remaining_voltage_squared < 0.0f)
            {
                remaining_voltage_squared = 0.0f;
            }
            remaining_axis_2_voltage_mv =
                sqrtf(remaining_voltage_squared);
            output_axis_2_mv = CurrentController_ClampFloat(
                unsaturated_axis_2_mv,
                -remaining_axis_2_voltage_mv,
                remaining_axis_2_voltage_mv);
            controller->axis_1_priority_active = 1U;
        }
        else
        {
            float scale = (float)controller->maximum_voltage_mv /
                          sqrtf(magnitude_squared);

            output_axis_1_mv *= scale;
            output_axis_2_mv *= scale;
        }
        controller->saturated = 1U;

        /*
         * 按实际限幅输出回算积分器，避免电压饱和期间积分持续累积。
         * 该思想来自参考代码，但这里使用物理单位且对双相电压联合限幅。
         */
        controller->integral_axis_1_mv =
            output_axis_1_mv - proportional_axis_1_mv -
            (float)feedforward_axis_1_mv;
        controller->integral_axis_2_mv =
            output_axis_2_mv - proportional_axis_2_mv -
            (float)feedforward_axis_2_mv;
    }
    else
    {
        controller->integral_axis_1_mv = candidate_integral_axis_1_mv;
        controller->integral_axis_2_mv = candidate_integral_axis_2_mv;
    }

    controller->output_axis_1_mv =
        CurrentController_RoundToInt32(output_axis_1_mv);
    controller->output_axis_2_mv =
        CurrentController_RoundToInt32(output_axis_2_mv);
}
