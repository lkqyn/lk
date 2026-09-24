#ifndef ELECTRICAL_ANGLE_H
#define ELECTRICAL_ANGLE_H

/*
 * 编码器机械位置到 FOC 电角度的映射模块。
 * 管理对齐零偏、B 相方向、编码器非线性标定和可选的连续相位观测器。
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    /* 完全停用分数角度观测，控制始终使用编码器原始整数计数。 */
    ELECTRICAL_ANGLE_OBSERVER_OFF = 0,
    /* 计算并记录观测角度，但控制仍使用原始角度。 */
    ELECTRICAL_ANGLE_OBSERVER_SHADOW,
    /* Park及反Park变换使用观测得到的连续分数角度。 */
    ELECTRICAL_ANGLE_OBSERVER_ON,
    /* 连续预测角叠加非线性标定修正，兼顾高速连续性及标定精度。 */
    ELECTRICAL_ANGLE_OBSERVER_CALIBRATED_PREDICTIVE
} ElectricalAngle_ObserverMode_t;

typedef struct
{
    int32_t alignment_offset_count;
    int32_t offset_in_electrical_cycle;
    int32_t electrical_count;
    int32_t electrical_angle_mdeg;
    int32_t raw_phase_index;
    int32_t corrected_phase_index;
    int32_t calibration_correction_phase_index;
    int32_t calibration_anchor_q1;
    int32_t observer_phase_index;
    int32_t observer_error_q16;
    int32_t observer_velocity_q16_per_tick;
    uint32_t observer_resync_count;
    uint32_t observer_bandwidth_hz;
    uint32_t observer_alpha_q20;
    uint32_t observer_beta_q20;
    ElectricalAngle_ObserverMode_t observer_mode;
    int8_t phase_b_axis_sign;
    uint8_t observer_initialized;
    uint8_t calibration_valid;
    uint8_t calibration_enabled;
    uint8_t aligned;
} ElectricalAngle_State_t;

/**
 * @brief 初始化电角度状态。增量编码器上电后默认未对齐。
 */
void ElectricalAngle_Init(void);

/**
 * @brief 使当前电角度零偏失效。
 */
void ElectricalAngle_Invalidate(void);

/**
 * @brief 将指定编码器累计位置登记为电角度0°。
 */
void ElectricalAngle_SetAlignment(int32_t encoder_position_count,
                                  int8_t phase_b_axis_sign);

/**
 * @brief 根据任意编码器累计位置计算0~3999的实时电角度计数。
 * @return 1表示电角度已对齐，0表示零偏或B相方向尚未有效。
 */
uint8_t ElectricalAngle_CalculateCount(int32_t encoder_position_count,
                                       int32_t *electrical_count);

/**
 * @brief 计算用于Park/反Park的电角度相位索引。
 * @note 校正未启用时与原始整数编码器角度逐位一致。
 */
uint8_t ElectricalAngle_CalculateControlPhaseIndex(
    int32_t encoder_position_count,
    int32_t *phase_index);

/**
 * @brief 装载201个整步边界的正反向中点表。
 * @param midpoint_q1 边界累计编码器位置的2倍。
 * @param point_count 必须为201。
 * @note 装载成功后仅标记数据有效，不会自动启用校正。
 */
uint8_t ElectricalAngle_SetNonlinearityCalibration(
    const int32_t *midpoint_q1,
    uint16_t point_count);

/**
 * @brief 启用或关闭编码器非线性电角度校正。
 * @note 只允许停机调用；仅与未校正的观测器ON模式互斥。
 */
uint8_t ElectricalAngle_EnableNonlinearityCalibration(uint8_t enable);

/**
 * @brief 使非线性校准表失效，并自动关闭校正。
 */
void ElectricalAngle_InvalidateNonlinearityCalibration(void);

/**
 * @brief 重置20kHz分数编码器角度观测器。
 * @note 启动电流环前调用，使观测位置从最新编码器计数开始。
 */
void ElectricalAngle_ResetObserver(int32_t encoder_position_count);

/**
 * @brief 用最新编码器累计计数更新分数角度观测器。
 * @param raw_phase_index 原始整数编码器角度对应的0~1023相位索引。
 * @param observer_phase_index 连续观测位置对应的0~1023相位索引。
 * @return 1表示输出有效，0表示电角度尚未对齐或参数无效。
 */
uint8_t ElectricalAngle_UpdateObserver(int32_t encoder_position_count,
                                       int32_t *raw_phase_index,
                                       int32_t *observer_phase_index);

/**
 * @brief 设置角度观测器工作模式。
 * @note 调用方必须保证电流环及PWM已经停止；calibrated模式保留标定。
 */
uint8_t ElectricalAngle_SetObserverMode(
    ElectricalAngle_ObserverMode_t mode);

/**
 * @brief 设置分数角度观测器带宽，允许范围100~1500Hz。
 * @note 只允许由停机状态下的调试命令调用，设置后应重置观测器。
 */
uint8_t ElectricalAngle_SetObserverBandwidthHz(uint32_t bandwidth_hz);

/**
 * @brief 根据编码器累计位置更新0~360°电角度。
 */
void ElectricalAngle_Update(int32_t encoder_position_count);

/**
 * @brief 获取电角度只读状态。
 */
const ElectricalAngle_State_t *ElectricalAngle_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
