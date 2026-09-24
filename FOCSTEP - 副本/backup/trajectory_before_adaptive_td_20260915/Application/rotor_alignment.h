#ifndef ROTOR_ALIGNMENT_H
#define ROTOR_ALIGNMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    ROTOR_ALIGNMENT_STATE_IDLE = 0,
    ROTOR_ALIGNMENT_STATE_RAMP_TO_90_DEG,
    ROTOR_ALIGNMENT_STATE_HOLD_90_DEG,
    ROTOR_ALIGNMENT_STATE_ROTATE_TO_ZERO,
    ROTOR_ALIGNMENT_STATE_HOLD_ZERO,
    ROTOR_ALIGNMENT_STATE_COMPLETE,
    ROTOR_ALIGNMENT_STATE_FAULT
} RotorAlignment_StateCode_t;

typedef enum
{
    ROTOR_ALIGNMENT_FAULT_NONE = 0,
    ROTOR_ALIGNMENT_FAULT_PRECONDITION,
    ROTOR_ALIGNMENT_FAULT_TARGET_UPDATE,
    ROTOR_ALIGNMENT_FAULT_OVERCURRENT,
    ROTOR_ALIGNMENT_FAULT_MOVEMENT,
    ROTOR_ALIGNMENT_FAULT_ABORTED
} RotorAlignment_Fault_t;

typedef struct
{
    RotorAlignment_StateCode_t state;
    RotorAlignment_Fault_t fault;
    int32_t first_position_count;
    int32_t zero_position_count;
    int32_t movement_count;
    int32_t expected_abs_movement_count;
    int32_t alignment_offset_count;
    int32_t target_a_ma;
    int32_t target_b_ma;
    int32_t hold_90_mean_current_a_ma;
    int32_t hold_90_mean_current_b_ma;
    int32_t hold_zero_mean_current_a_ma;
    int32_t hold_zero_mean_current_b_ma;
    int32_t peak_abs_current_a_ma;
    int32_t peak_abs_current_b_ma;
    int32_t peak_abs_voltage_a_mv;
    int32_t peak_abs_voltage_b_mv;
    uint32_t current_sample_count;
    uint32_t saturated_sample_count;
    int8_t phase_b_axis_sign;
    uint8_t auto_start_pending;
    uint8_t valid;
} RotorAlignment_State_t;

#define ENCODER_CALIBRATION_BOUNDARY_COUNT (201U)

typedef enum
{
    ENCODER_CALIBRATION_STATE_IDLE = 0,
    ENCODER_CALIBRATION_STATE_RAMP_CURRENT,
    ENCODER_CALIBRATION_STATE_HOLD_START,
    ENCODER_CALIBRATION_STATE_MOVE_PRELOAD_REVERSE,
    ENCODER_CALIBRATION_STATE_SETTLE_PRELOAD_REVERSE,
    ENCODER_CALIBRATION_STATE_RETURN_FORWARD_ZERO,
    ENCODER_CALIBRATION_STATE_SETTLE_FORWARD_ZERO,
    ENCODER_CALIBRATION_STATE_MOVE_FORWARD,
    ENCODER_CALIBRATION_STATE_SETTLE_FORWARD,
    ENCODER_CALIBRATION_STATE_MOVE_OVERSHOOT_FORWARD,
    ENCODER_CALIBRATION_STATE_SETTLE_OVERSHOOT_FORWARD,
    ENCODER_CALIBRATION_STATE_MOVE_REVERSE,
    ENCODER_CALIBRATION_STATE_SETTLE_REVERSE,
    ENCODER_CALIBRATION_STATE_COMPLETE,
    ENCODER_CALIBRATION_STATE_FAULT
} EncoderCalibration_StateCode_t;

typedef enum
{
    ENCODER_CALIBRATION_FAULT_NONE = 0,
    ENCODER_CALIBRATION_FAULT_PRECONDITION,
    ENCODER_CALIBRATION_FAULT_TARGET_UPDATE,
    ENCODER_CALIBRATION_FAULT_OVERCURRENT,
    ENCODER_CALIBRATION_FAULT_FORWARD_INTERVAL,
    ENCODER_CALIBRATION_FAULT_REVERSE_INTERVAL,
    ENCODER_CALIBRATION_FAULT_TOTAL_MOVEMENT,
    ENCODER_CALIBRATION_FAULT_TABLE_INVALID,
    ENCODER_CALIBRATION_FAULT_ABORTED
} EncoderCalibration_Fault_t;

typedef struct
{
    EncoderCalibration_StateCode_t state;
    EncoderCalibration_Fault_t fault;
    uint16_t boundary_index;
    int32_t start_position_count;
    int32_t forward_total_count;
    int32_t reverse_total_count;
    int32_t closure_error_q1;
    int32_t minimum_interval_q1;
    int32_t maximum_interval_q1;
    int32_t maximum_hysteresis_count;
    uint32_t saturated_sample_count;
    uint8_t data_valid;
} EncoderCalibration_State_t;

void RotorAlignment_Init(void);

/**
 * @brief 启动一次90°到0°的闭环电流对齐测试。
 * @return 1表示已启动，0表示前置条件不满足。
 */
uint8_t RotorAlignment_Start(void);

/**
 * @brief 推进非阻塞对齐状态机，应在主循环中持续调用。
 */
void RotorAlignment_Update(void);

/**
 * @brief 中止正在进行的对齐并立即进入低端制动。
 */
void RotorAlignment_Abort(void);

uint8_t RotorAlignment_IsRunning(void);
const RotorAlignment_State_t *RotorAlignment_GetState(void);

/**
 * @brief 启动编码器一圈非线性标定数据采集。
 * @note 电机将以约12rpm完成端点预压、正转一圈和反转一圈，
 *       必须空载执行。
 */
uint8_t RotorAlignment_StartEncoderCalibration(void);
void RotorAlignment_AbortEncoderCalibration(void);
uint8_t RotorAlignment_IsEncoderCalibrationRunning(void);
const EncoderCalibration_State_t *RotorAlignment_GetEncoderCalibrationState(void);

/**
 * @brief 读取一个整步边界的正反向原始位置及消隙中点。
 * @param index 边界索引0~200。
 * @param forward_count 正向到达时的累计编码器计数。
 * @param reverse_count 反向返回时的累计编码器计数。
 * @param midpoint_q1 正反向中点的2倍，保留0.5计数分辨率。
 */
uint8_t RotorAlignment_GetEncoderCalibrationPoint(
    uint16_t index,
    int32_t *forward_count,
    int32_t *reverse_count,
    int32_t *midpoint_q1);

#ifdef __cplusplus
}
#endif

#endif
