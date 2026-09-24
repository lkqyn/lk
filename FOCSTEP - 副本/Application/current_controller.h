#ifndef CURRENT_CONTROLLER_H
#define CURRENT_CONTROLLER_H

/*
 * 通用双轴 PI 电流控制器。
 * 被 d/q 电流环和静止 A/B 相电流环复用，负责限压、抗积分饱和及轴优先级。
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float proportional_gain_mv_per_ma;
    float integral_gain_mv_per_ma_per_step;
    float integral_axis_1_mv;
    float integral_axis_2_mv;
    int32_t output_axis_1_mv;
    int32_t output_axis_2_mv;
    int32_t maximum_voltage_mv;
    uint8_t saturated;
    uint8_t axis_1_priority_active;
} CurrentController_t;

/**
 * @brief 根据电机电气参数和目标带宽初始化双相电流PI控制器。
 * @param controller 控制器实例，由调用方持有，模块内部不使用全局实例。
 * @param bandwidth_hz 电流环目标带宽，单位Hz。
 * @param sample_frequency_hz 电流环执行频率，单位Hz。
 * @param maximum_voltage_mv 输出电压矢量上限，单位mV。
 * @return 1表示参数有效，0表示参数错误。
 */
uint8_t CurrentController_Init(CurrentController_t *controller,
                               float bandwidth_hz,
                               float sample_frequency_hz,
                               int32_t maximum_voltage_mv);

/**
 * @brief 在线更新电压矢量上限。
 * @note 调用方必须保证更新期间不会与电流控制计算并发访问。
 */
uint8_t CurrentController_SetMaximumVoltage(
    CurrentController_t *controller,
    int32_t maximum_voltage_mv);

/**
 * @brief 清零积分器和输出，不改变控制参数。
 */
void CurrentController_Reset(CurrentController_t *controller);

/**
 * @brief 执行一次正交双轴电流PI计算，并进行电压矢量限幅和积分回算。
 * @note 两轴可以表示静止A/B相，也可以表示旋转d/q轴。
 */
void CurrentController_Step(CurrentController_t *controller,
                            int32_t target_axis_1_ma,
                            int32_t target_axis_2_ma,
                            int32_t measured_axis_1_ma,
                            int32_t measured_axis_2_ma);

/**
 * @brief 执行带电压前馈的双轴电流PI计算。
 * @note 前馈参与统一电压矢量限幅和积分回算，单位均为mV。
 * @param axis_1_priority_enabled 非0时，饱和后优先保留轴1电压。
 */
void CurrentController_StepWithFeedforward(
    CurrentController_t *controller,
    int32_t target_axis_1_ma,
    int32_t target_axis_2_ma,
    int32_t measured_axis_1_ma,
    int32_t measured_axis_2_ma,
    int32_t feedforward_axis_1_mv,
    int32_t feedforward_axis_2_mv,
    uint8_t axis_1_priority_enabled);

#ifdef __cplusplus
}
#endif

#endif
