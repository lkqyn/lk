#ifndef PHASE_CURRENT_LOOP_H
#define PHASE_CURRENT_LOOP_H

/*
 * 静止 A/B 两相电流闭环。
 * 用于目标磁场固定在定子坐标的微步试验模式，不使用编码器角度 Park 变换。
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    PHASE_CURRENT_LOOP_FAULT_NONE = 0,
    PHASE_CURRENT_LOOP_FAULT_OVERCURRENT
} PhaseCurrentLoop_Fault_t;

typedef struct
{
    float bandwidth_hz;
    float sample_frequency_hz;
    int32_t maximum_voltage_mv;
    int32_t maximum_target_current_ma;
    int32_t overcurrent_limit_ma;
    uint32_t bus_voltage_mv;
} PhaseCurrentLoop_Config_t;

typedef struct
{
    int32_t target_a_ma;
    int32_t target_b_ma;
    int32_t output_a_mv;
    int32_t output_b_mv;
    int32_t peak_abs_current_a_ma;
    int32_t peak_abs_current_b_ma;
    int32_t peak_abs_output_a_mv;
    int32_t peak_abs_output_b_mv;
    uint32_t sample_count;
    uint32_t saturated_sample_count;
    PhaseCurrentLoop_Fault_t fault;
    uint8_t running;
} PhaseCurrentLoop_State_t;

/**
 * @brief 启动双相20kHz闭环电流执行器，初始目标电流为0。
 * @return 1表示启动成功，0表示配置或电流采样状态无效。
 */
uint8_t PhaseCurrentLoop_Start(const PhaseCurrentLoop_Config_t *config);

/**
 * @brief 原子更新A、B相目标电流。
 * @return 1表示目标在安全范围内，0表示未运行或目标越界。
 */
uint8_t PhaseCurrentLoop_SetTargets(int32_t target_a_ma,
                                    int32_t target_b_ma);

/**
 * @brief 停止电流环并进入当前硬件规定的低端短路制动状态。
 */
void PhaseCurrentLoop_Stop(void);

/**
 * @brief 获取只读运行状态。
 */
const volatile PhaseCurrentLoop_State_t *PhaseCurrentLoop_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
