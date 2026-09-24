#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    int32_t raw_count;
    int32_t zero_count;
    int32_t position_count;
    int32_t count_in_revolution;
    /* 最近一次有效测速窗口内的编码器增量。 */
    int32_t delta_count;
    float mechanical_angle_deg;
    int32_t raw_speed_mrpm;
    int32_t filtered_speed_mrpm;
    float speed_rpm;
    uint16_t speed_measurement_period_ms;
    uint8_t speed_valid;
} Encoder_State_t;

/**
 * @brief 供1kHz控制与诊断任务读取的测速快照。
 * @note 三种速度均由同一次1ms位置采样生成，便于逐点比较测速算法。
 */
typedef struct
{
    int32_t sample_delta_count;
    int32_t adaptive_speed_mrpm;
    int32_t fixed_5ms_speed_mrpm;
    int32_t fixed_10ms_speed_mrpm;
} Encoder_SpeedDiagnostic_t;

/**
 * @brief 启动TIM2的AB正交编码器接口。
 */
void Encoder_Init(void);

/**
 * @brief 在后台刷新供监控使用的编码器状态快照。
 */
void Encoder_Update(void);

/**
 * @brief 编码器1ms固定周期采样任务。
 * @note 由SysTick中断调用；函数内禁止加入浮点运算、阻塞操作或串口输出。
 */
void Encoder_Tick1ms(void);

/**
 * @brief 将当前位置设置为机械零点。
 */
void Encoder_SetZero(void);

/**
 * @brief 直接读取当前累计位置，供20kHz控制中断使用。
 * @note 该函数只读取TIM2计数器并减去零点，不更新速度等后台状态。
 */
int32_t Encoder_GetPositionCountFast(void);

/**
 * @brief 读取供速度环使用的5ms滑动窗口转速，单位mrpm。
 * @note 4000计数/圈编码器在200rpm时5ms窗口仍约13计数；速度量化为
 *       3rpm/count，可兼顾低速平滑和速度环动态响应。
 */
int32_t Encoder_GetControlSpeedMrpmFast(void);

/**
 * @brief 读取1ms编码器增量及并行测速结果。
 * @param[out] diagnostic 诊断快照输出地址。
 * @note 仅读取32位原子数据，可在固定周期控制任务中调用。
 */
void Encoder_GetSpeedDiagnosticFast(Encoder_SpeedDiagnostic_t *diagnostic);

/**
 * @brief 获取编码器只读状态。
 */
const Encoder_State_t *Encoder_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
