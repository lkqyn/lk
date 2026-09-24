#ifndef DQ_TRANSFORM_H
#define DQ_TRANSFORM_H

/*
 * Park / 反 Park 坐标变换。
 * 初始化阶段建立正弦表，实时电流环仅执行定点查表和乘加运算。
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 初始化四分之一周期正弦查找表。
 * @note 初始化阶段允许使用浮点三角函数，20kHz控制中断只执行定点查表。
 */
void DqTransform_Init(void);

/**
 * @brief 将静止alpha/beta坐标量变换到转子d/q坐标。
 * @param electrical_count 0~3999对应0~360°电角度。
 */
void DqTransform_Park(int32_t alpha,
                      int32_t beta,
                      int32_t electrical_count,
                      int32_t *direct,
                      int32_t *quadrature);

/**
 * @brief 使用0~1023连续相位索引执行Park变换。
 * @note 该接口供分数编码器角度观测器使用，避免退化到4000整数计数。
 */
void DqTransform_ParkPhaseIndex(int32_t alpha,
                                int32_t beta,
                                int32_t phase_index,
                                int32_t *direct,
                                int32_t *quadrature);

/**
 * @brief 将转子d/q坐标量反变换到静止alpha/beta坐标。
 * @param electrical_count 0~3999对应0~360°电角度。
 */
void DqTransform_InversePark(int32_t direct,
                             int32_t quadrature,
                             int32_t electrical_count,
                             int32_t *alpha,
                             int32_t *beta);

/**
 * @brief 使用0~1023连续相位索引执行反Park变换。
 */
void DqTransform_InverseParkPhaseIndex(int32_t direct,
                                       int32_t quadrature,
                                       int32_t phase_index,
                                       int32_t *alpha,
                                       int32_t *beta);

#ifdef __cplusplus
}
#endif

#endif
