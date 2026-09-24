#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_pwm.h"

#include <stdint.h>

typedef struct
{
    int32_t mean_current_a_ma;
    int32_t mean_current_b_ma;
    int32_t peak_abs_current_a_ma;
    int32_t peak_abs_current_b_ma;
    int32_t peak_to_peak_a_ma;
    int32_t peak_to_peak_b_ma;
    uint32_t noise_rms_a_ma;
    uint32_t noise_rms_b_ma;
    uint32_t synchronized_sample_count;
    uint32_t saturated_sample_count;
    int32_t peak_abs_voltage_a_mv;
    int32_t peak_abs_voltage_b_mv;
    uint32_t elapsed_ms;
    uint8_t overcurrent;
} MotorTest_Result_t;

typedef struct
{
    int32_t target_d_ma;
    int32_t target_q_ma;
    uint32_t bandwidth_hz;
    int32_t voltage_limit_mv;
    int32_t movement_count;
    int32_t final_electrical_count;
    int32_t peak_abs_phase_current_a_ma;
    int32_t peak_abs_phase_current_b_ma;
    int32_t peak_abs_output_d_mv;
    int32_t peak_abs_output_q_mv;
    uint32_t synchronized_sample_count;
    uint32_t saturated_sample_count;
    uint32_t elapsed_ms;
    int32_t response_delay_10_us;
    int32_t rise_time_10_to_90_us;
    int32_t first_target_cross_us;
    int32_t first_enter_5_percent_us;
    int32_t first_stable_5_percent_us;
    int32_t peak_axis_ma;
    int32_t peak_time_us;
    uint32_t overshoot_permille;
    int32_t steady_axis_ma;
    int32_t steady_orthogonal_ma;
    uint32_t steady_axis_error_rms_ma;
    uint32_t steady_orthogonal_rms_ma;
    int32_t steady_axis_ripple_pp_ma;
    int32_t steady_orthogonal_abs_peak_ma;
    int32_t fall_time_90_to_10_us;
    int32_t fall_first_enter_5_percent_us;
    int32_t fall_first_stable_5_percent_us;
    int32_t fall_min_axis_ma;
    uint32_t fall_axis_rms_ma;
    uint32_t fall_orthogonal_rms_ma;
    uint8_t fault;
} MotorTest_DqResult_t;

#define MOTOR_TEST_DQ_TRACE_CAPACITY  (1250U)

typedef struct
{
    int16_t target_d_ma;
    int16_t target_q_ma;
    int16_t measured_d_ma;
    int16_t measured_q_ma;
    int16_t measured_phase_a_ma;
    int16_t measured_phase_b_ma;
    int16_t output_d_mv;
    int16_t output_q_mv;
    int16_t electrical_count;
    uint8_t voltage_saturated;
} MotorTest_DqTraceSample_t;

/**
 * @brief 执行100ms单相小电流极性测试，结束后自动进入低端制动。
 * @param phase 测试相。
 * @param polarity 仅允许+1或-1。
 * @param result 测试结果输出。
 * @return 1表示已执行，0表示参数错误或电流零点无效。
 */
uint8_t MotorTest_RunPhaseCurrent(MotorPWM_Phase_t phase,
                                  int8_t polarity,
                                  MotorTest_Result_t *result);

/**
 * @brief 四路PWM均50%时，统计100ms同步电流采样噪声。
 */
uint8_t MotorTest_RunCurrentNoise(MotorTest_Result_t *result);

/**
 * @brief 执行受限的双相静态电流闭环测试，结束后自动进入低端制动。
 * @param target_a_ma A相目标电流，绝对值不得超过测试上限。
 * @param target_b_ma B相目标电流，绝对值不得超过测试上限。
 */
uint8_t MotorTest_RunClosedLoopCurrent(int32_t target_a_ma,
                                       int32_t target_b_ma,
                                       MotorTest_Result_t *result);

/**
 * @brief 执行100ms受限d/q电流环测试，结束后自动进入低端制动。
 * @note 必须先完成电角度对齐，调试目标电流矢量幅值不得超过1A。
 */
uint8_t MotorTest_RunDqCurrent(int32_t target_d_ma,
                              int32_t target_q_ma,
                              MotorTest_DqResult_t *result);

/**
 * @brief 返回最近一次d/q阶跃测试的20kHz逐周期记录。
 * @param sample_count 输出有效样本数，可传入空指针。
 * @return 样本首地址，数据在下一次测试时被覆盖。
 */
const MotorTest_DqTraceSample_t *MotorTest_GetDqTrace(
    uint16_t *sample_count);

/**
 * @brief 启动非阻塞的d/q单轴周期阶跃测试。
 * @param target_d_ma Id高电平目标，单位mA。
 * @param target_q_ma Iq高电平目标，单位mA。
 * @param half_period_ms 零电流和目标电流各自的保持时间。
 * @param duration_ms 测试总时间，到时自动停止并进入制动状态。
 * @note Id和Iq只允许一个非零；Iq测试必须可靠固定转子。
 * @return 1表示启动成功，0表示参数、目标或运行前提无效。
 */
uint8_t MotorTest_StartDqCycle(int32_t target_d_ma,
                               int32_t target_q_ma,
                               uint32_t half_period_ms,
                               uint32_t duration_ms);

/**
 * @brief 更新d/q周期阶跃状态机，由应用层主循环持续调用。
 */
void MotorTest_UpdateDqCycle(void);

/**
 * @brief 主动停止d/q周期阶跃并进入制动状态。
 */
void MotorTest_StopDqCycle(void);

/**
 * @brief 从1kHz遥测环形缓冲区读取一个样本。
 * @return 1表示读取成功，0表示当前无样本。
 */
uint8_t MotorTest_PopDqCycleSample(MotorTest_DqTraceSample_t *sample);

uint8_t MotorTest_DqCycleIsRunning(void);
uint8_t MotorTest_GetDqCycleFault(void);
uint32_t MotorTest_GetDqCycleDroppedSamples(void);

#ifdef __cplusplus
}
#endif

#endif
