#ifndef DQ_CURRENT_LOOP_H
#define DQ_CURRENT_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    DQ_CURRENT_LOOP_FAULT_NONE = 0,
    DQ_CURRENT_LOOP_FAULT_ANGLE_INVALID,
    DQ_CURRENT_LOOP_FAULT_OVERCURRENT,
    DQ_CURRENT_LOOP_FAULT_POSITION_DEVIATION
} DqCurrentLoop_Fault_t;

typedef struct
{
    float bandwidth_hz;
    float sample_frequency_hz;
    int32_t maximum_voltage_mv;
    int32_t maximum_target_current_ma;
    int32_t overcurrent_limit_ma;
    int32_t maximum_position_deviation_count;
    uint32_t bus_voltage_mv;
} DqCurrentLoop_Config_t;

typedef struct
{
    int32_t target_d_ma;
    int32_t target_q_ma;
    int32_t measured_d_ma;
    int32_t measured_q_ma;
    int32_t measured_phase_a_ma;
    int32_t measured_phase_b_ma;
    int32_t output_d_mv;
    int32_t output_q_mv;
    int32_t feedforward_d_mv;
    int32_t feedforward_q_mv;
    int32_t phase_voltage_a_mv;
    int32_t phase_voltage_b_mv;
    /* Park使用的当前角度，以及反Park使用的预测电压角度。 */
    int32_t electrical_count;
    int32_t voltage_electrical_count;
    int32_t raw_phase_index;
    int32_t observer_phase_index;
    int32_t control_phase_index;
    int32_t voltage_phase_index;
    /* 原始编码器角度与Shadow连续观测角度的相位差，范围-512~511。 */
    int32_t raw_observer_phase_error_index;
    /* 相邻20kHz周期的原始编码器电角度跳变量，范围-512~511。 */
    int32_t raw_phase_step_index;
    int32_t angle_advance_count;
    uint32_t angle_advance_delay_us;
    int32_t encoder_position_count;
    int32_t peak_abs_phase_current_a_ma;
    int32_t peak_abs_phase_current_b_ma;
    int32_t peak_abs_output_d_mv;
    int32_t peak_abs_output_q_mv;
    uint32_t sample_count;
    uint32_t saturated_sample_count;
    uint32_t d_axis_priority_sample_count;
    uint32_t observer_phase_error_exceed_sample_count;
    int32_t peak_abs_observer_phase_error_index;
    uint32_t raw_phase_step_exceed_sample_count;
    int32_t peak_abs_raw_phase_step_index;
    DqCurrentLoop_Fault_t fault;
    uint8_t voltage_saturated;
    uint8_t d_axis_priority_active;
    uint8_t running;
} DqCurrentLoop_State_t;

typedef void (*DqCurrentLoop_SampleObserver_t)(
    const volatile DqCurrentLoop_State_t *state,
    void *context);

/**
 * @brief 启动基于编码器电角度的20kHz d/q电流环。
 * @return 1表示启动成功，0表示参数、电角度或采样状态无效。
 */
uint8_t DqCurrentLoop_Start(const DqCurrentLoop_Config_t *config);

/**
 * @brief 原子更新d/q目标电流，目标矢量幅值不得超过配置上限。
 */
uint8_t DqCurrentLoop_SetTargets(int32_t target_d_ma,
                                 int32_t target_q_ma);

/**
 * @brief 原子更新d/q目标电流及解耦前馈电压。
 * @note 速度环使用该接口，保证20kHz中断读取到同一工作点的数据。
 */
uint8_t DqCurrentLoop_SetOperatingPoint(int32_t target_d_ma,
                                        int32_t target_q_ma,
                                        int32_t feedforward_d_mv,
                                        int32_t feedforward_q_mv);

/**
 * @brief 原子更新母线电压及电流控制器的电压矢量上限。
 * @note 供速度环根据滤波后的实时母线电压更新PWM换算和弱磁预算。
 */
uint8_t DqCurrentLoop_UpdateVoltageBudget(uint32_t bus_voltage_mv,
                                          int32_t maximum_voltage_mv);

/**
 * @brief 设置反Park电压角度的预测延迟，单位us。
 * @note 只允许在电流环停止时修改，范围0~100us；Park测量角度不补偿。
 */
uint8_t DqCurrentLoop_SetAngleAdvanceDelayUs(uint32_t delay_us);

/**
 * @brief 停止d/q电流环并进入当前硬件规定的低端短路制动状态。
 */
void DqCurrentLoop_Stop(void);

const volatile DqCurrentLoop_State_t *DqCurrentLoop_GetState(void);

/**
 * @brief 设置逐周期观测回调，仅用于RAM快速记录，禁止在回调中打印或阻塞。
 * @note 只允许在电流环停止时修改，传入空指针可取消观测。
 */
uint8_t DqCurrentLoop_SetSampleObserver(
    DqCurrentLoop_SampleObserver_t observer,
    void *context);

#ifdef __cplusplus
}
#endif

#endif
