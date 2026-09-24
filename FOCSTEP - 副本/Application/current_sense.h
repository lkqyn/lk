#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

/*
 * A、B 相电流采样模块。
 * 管理 ADC/DMA 零偏校准，以及由 TIM4_CC2 触发的 20 kHz 同步注入采样。
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    uint16_t raw_a;
    uint16_t raw_b;
    uint16_t offset_a;
    uint16_t offset_b;
    uint16_t calibration_span_a;
    uint16_t calibration_span_b;
    int32_t current_a_ma;
    int32_t current_b_ma;
    uint32_t synchronized_sample_count;
    uint8_t calibrated;
    uint8_t offset_valid;
    uint8_t synchronized;
} CurrentSense_State_t;

/**
 * @brief PWM同步电流采样完成后的中断回调类型。
 * @note 回调运行在ADC中断中，只允许执行有界、非阻塞的实时控制计算。
 */
typedef void (*CurrentSense_SampleCallback_t)(void *context);

/**
 * @brief 启动A、B两相电流ADC DMA，并在零电流条件下自动校零。
 */
void CurrentSense_Init(void);

/**
 * @brief 根据最新ADC原始值更新两相电流。
 */
void CurrentSense_Update(void);

/**
 * @brief 重新校准两路零电流偏置。校准时电机必须静止且无相电流。
 * @return 1表示校准完成，0表示ADC DMA未启动。
 */
uint8_t CurrentSense_CalibrateOffsets(void);

/**
 * @brief 停止自由运行DMA，启动由TIM4_CC2触发的20kHz PWM同步注入组采样。
 * @return 1表示启动成功，0表示启动失败。
 */
uint8_t CurrentSense_StartSynchronizedSampling(void);

/**
 * @brief 停止PWM同步采样，恢复用于静态诊断的自由运行DMA。
 */
void CurrentSense_StopSynchronizedSampling(void);

/**
 * @brief 设置同步采样回调。传入空指针可取消回调。
 * @note 必须在同步采样停止时调用，避免回调指针并发改变。
 */
void CurrentSense_SetSampleCallback(CurrentSense_SampleCallback_t callback,
                                    void *context);

/**
 * @brief 获取电流采样状态。
 */
const CurrentSense_State_t *CurrentSense_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
