#ifndef PULSE_INPUT_H
#define PULSE_INPUT_H

#include <stdint.h>

typedef struct
{
    int32_t reference_position_count;
    /* 单个相邻STEP周期换算的原始速度，仅用于脉冲前馈诊断。 */
    int32_t raw_reference_speed_mrpm;
    /* 当前提供给位置环的速度前馈；本版本与raw_reference_speed_mrpm相同。 */
    int32_t reference_speed_mrpm;
    /* 1=最近STEP流仍连续；0=已越过预计下一STEP的允许窗口。 */
    uint8_t pulse_stream_active;
    uint32_t accepted_pulse_count;
    uint32_t ignored_pulse_count;
    uint32_t last_step_tick_ms;
    /* TIM3_CH2边沿硬件时间戳（10.5MHz）与该边沿的编码器位置快照。 */
    uint32_t last_edge_capture_cycle_count;
    int32_t last_edge_reference_position_count;
    int32_t last_edge_measured_position_count;
    int32_t last_step_reference_position_count;
    int32_t last_step_measured_position_count;
    int32_t last_step_error_count;
    int32_t last_step_reference_speed_mrpm;
    int32_t last_step_measured_speed_mrpm;
    int32_t maximum_abs_position_error_count;
    int32_t maximum_abs_speed_error_mrpm;
    uint32_t motion_sample_count;
    uint8_t tracking_error_started;
    uint32_t first_error_pulse_count;
    int32_t first_error_position_count;
    int32_t first_error_reference_speed_mrpm;
    int32_t first_error_measured_speed_mrpm;
    uint32_t peak_error_pulse_count;
    int32_t peak_error_position_count;
    int32_t peak_error_reference_speed_mrpm;
    int32_t peak_error_measured_speed_mrpm;
    uint32_t peak_speed_error_pulse_count;
    int32_t peak_speed_error_position_count;
    int32_t peak_speed_error_reference_mrpm;
    int32_t peak_speed_error_measured_mrpm;
    uint8_t enabled;
    uint8_t input_enabled;
} PulseInput_State_t;

void PulseInput_Init(void);
/*
 * 启动产品态STEP/DIR接口。此函数在上电校准完成后由应用层调用；
 * 成功后接口持续运行，PUL_EN是唯一的外部接收门控。
 */
uint8_t PulseInput_Start(void);
/* 由TIM3_CH2输入捕获回调调用；capture_count是边沿硬件锁存值。 */
void PulseInput_OnStepCaptured(uint16_t capture_count);
void PulseInput_OnCaptureTimerOverflow(void);
void PulseInput_Enable(void);
void PulseInput_Disable(void);
/* 由独立4kHz节拍调用，低延迟提交最新STEP位置参考。 */
void PulseInput_Tick250us(void);
void PulseInput_Tick1ms(void);
const volatile PulseInput_State_t *PulseInput_GetState(void);

#endif
