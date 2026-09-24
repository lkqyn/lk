#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    MOTOR_PWM_PHASE_A = 0,
    MOTOR_PWM_PHASE_B
} MotorPWM_Phase_t;

/**
 * @brief 初始化PWM制动状态。该函数不会启动TIM1调制输出。
 */
void MotorPWM_Init(void);

/**
 * @brief 进入低端短路制动状态。
 *
 * @note 当前硬件中EG2131的HIN与低有效LIN共用一路PWM。
 *       PWM全低会使四个低端MOS管导通，两相绕组被短路制动；
 *       这不是上下管全部关断的高阻状态。
 */
void MotorPWM_EnterBrakeState(void);

/**
 * @brief 启动四路20kHz中心对齐PWM，初始占空比均为50%。
 *
 * @note 四个桥臂同步输出50%时，理想差分相电压为0。
 */
void MotorPWM_StartNeutral(void);

/**
 * @brief 设置目标相的差分比较值，另一相保持50%中性调制。
 * @param phase 目标相。
 * @param differential_counts 正值表示正端占空比高于负端。
 */
void MotorPWM_SetPhaseDifferentialCounts(MotorPWM_Phase_t phase,
                                         int16_t differential_counts);

/**
 * @brief 同时设置A、B两相平均差分电压。
 * @param voltage_a_mv A相差分电压，单位mV。
 * @param voltage_b_mv B相差分电压，单位mV。
 * @param bus_voltage_mv 实测直流母线电压，单位mV。
 *
 * @note 两相比较值会成对围绕50%变化，避免产生不必要的共模偏置。
 */
void MotorPWM_SetPhaseVoltagesMv(int32_t voltage_a_mv,
                                 int32_t voltage_b_mv,
                                 uint32_t bus_voltage_mv);

/**
 * @brief 查询PWM是否已启动。
 */
uint8_t MotorPWM_IsEnabled(void);

#ifdef __cplusplus
}
#endif

#endif
