#ifndef PULSE_INPUT_H
#define PULSE_INPUT_H

#include <stdint.h>

/*
 * 外部 STEP/DIR 输入模块。
 *
 * 数据流（产品态）
 *
 *   PC7 / TIM3_CH2 的一个有效 STEP 边沿
 *       -> reference_position_count（PulsePos）按方向增减 5 encoder count
 *       -> 250 us 内部轨迹（仅 TRAJECTORY 模式）生成 CmdPos / CmdSpeed
 *       -> PositionLoop -> SpeedLoop -> 电流环 -> 电机
 *
 * PulsePos 始终只由真实 STEP 边沿改变，是外部控制器要求的最终位置。
 * 任何 1 ms、4 kHz 或诊断任务均不得自行外推 PulsePos。
 *
 * 当前固定使用标准 FOC 串级三环。该选择不影响 PC7 的脉冲计数规则。
 */
/*
 * 标准脉冲控制链选择。
 *
 * TRAJECTORY：标准 FOC 串级三环（PulsePos -> CmdPos -> 位置 PD ->
 *             速度 PI -> d/q 电流环）。
 *             编码器误差会产生纠偏转矩，可跨越任意数量的电周期恢复位置。
 */
#define PULSE_INPUT_CONTROL_MODE_TRAJECTORY         (0U)
#define PULSE_INPUT_CONTROL_MODE \
    PULSE_INPUT_CONTROL_MODE_TRAJECTORY

typedef struct
{
    /*
     * PulsePos：外部脉冲累计的绝对目标位置。
     * 4000 encoder count/rev、800 STEP/rev，故 1 STEP = 5 count。
     */
    int32_t reference_position_count;
    /* 单个相邻STEP周期换算的原始速度，仅用于脉冲前馈诊断。 */
    int32_t raw_reference_speed_mrpm;
    /* 当前提供给位置环的速度前馈；本版本与raw_reference_speed_mrpm相同。 */
    int32_t reference_speed_mrpm;
    /* 1=最近STEP流仍连续；0=已越过预计下一STEP的允许窗口。 */
    uint8_t pulse_stream_active;
    /* 1=同方向连续实测到脉冲频率下降；不依赖固定脉冲数或距离。 */
    uint8_t pulse_decelerating;
    /* 本次使能以来接收/忽略的 STEP 数；用于确认输入链路和诊断。 */
    uint32_t accepted_pulse_count;
    uint32_t ignored_pulse_count;
    uint32_t last_step_tick_ms;
    /* TIM3_CH2边沿硬件时间戳（10.5MHz）与该边沿的编码器位置快照。 */
    uint32_t last_edge_capture_cycle_count;
    int32_t last_edge_reference_position_count;
    /* 同一硬件STEP边沿时，4kHz内部轨迹CmdPos的原子快照。 */
    int32_t last_edge_command_position_count;
    int32_t last_edge_measured_position_count;
    /* 最近一个STEP边沿到首次进入最终位置±1 count的时间；无下一STEP时即最终记录。 */
    uint32_t last_edge_reach_time_us;
    uint8_t last_edge_reached;
    /* 进入±1 count且低速连续保持20ms后的稳定到位时间。 */
    uint32_t last_edge_stable_time_us;
    uint8_t last_edge_stable;
    /* 下面一组为 1 ms 采样快照；区别于上面的“硬件边沿瞬间”快照。 */
    int32_t last_step_reference_position_count;
    int32_t last_step_measured_position_count;
    int32_t last_step_error_count;
    int32_t last_step_reference_speed_mrpm;
    int32_t last_step_measured_speed_mrpm;
    /* 一次运动期间的统计峰值，仅读取/串口诊断，不参与控制。 */
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
    /* enabled：脉冲控制链已启动；input_enabled：当前 PC9 是否允许接收。 */
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
/* 仅供单板脉冲发生器使用：复用同一计数核心，但不读取物理 DIR/EN 引脚。 */
void PulseInput_OnSimulatedStepCaptured(uint16_t capture_count,
                                        int8_t direction);
void PulseInput_OnCaptureTimerOverflow(void);
void PulseInput_Enable(void);
void PulseInput_Disable(void);
/*
 * 由独立 4 kHz 节拍调用，低延迟提交最新 STEP 位置参考。
 * 不改变 PulsePos；TRAJECTORY 模式仅在这里更新内部 CmdPos。
 */
void PulseInput_Tick250us(void);
/* 由 1 kHz 节拍调用：换算最近 STEP 周期的诊断速度并更新统计数据。 */
void PulseInput_Tick1ms(void);
const volatile PulseInput_State_t *PulseInput_GetState(void);

#endif
