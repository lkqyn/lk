#ifndef __FOC_H__
#define __FOC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float Vdc;          // 直流母线电压（比如 12.0）
    float Uq;           // 开环 q 轴电压幅值（建议先 0.5~2.0V 等效）
    float Ud;           // 开环 d 轴电压（建议 0）
    float fe;           // 电角频率 Hz（先小：0.5~5Hz 电角）
    float duty_max;     // 最大占空比限制 0..1（建议 0.98）
    float deadband;     // 小于该占空比就直接 OFF（比如 0.01）

    float theta;        // 电角度 [0, 2π)
} foc_ol_t;

/**
 * @brief  开环FOC初始化
 * @param  foc        foc对象
 * @param  Vdc        母线电压
 * @param  Uq         q轴电压幅值（相当于“推力”）
 * @param  fe_hz      电角频率 Hz
 */
void FOC_OpenLoopInit(foc_ol_t *foc, float Vdc, float Uq, float fe_hz);

/**
 * @brief  更新一次开环FOC（建议固定周期调用，例如 1kHz~10kHz）
 * @param  foc        foc对象
 * @param  dt         本次更新周期（秒），比如 0.001f
 */
void FOC_OpenLoopUpdate(foc_ol_t *foc, float dt);

/**
 * @brief  立刻关闭四路半桥（真 OFF）
 */
void FOC_AllOff(void);

/**
 * @brief  设置默认的参数
 * @param  foc        foc对象
 */
void FOC_SetDefaultParams(foc_ol_t *foc);

// 输出Va/Vb到两相H桥（单位：V）
void foc_apply_Vab(const foc_ol_t *foc, float Va, float Vb);

//软启动
void FOC_OpenLoopSetFe(foc_ol_t *foc, float fe_hz);
void FOC_OpenLoopSetUqRatio(foc_ol_t *foc, float uq_ratio); // 0..duty_max
void FOC_OpenLoopRampUqRatio(foc_ol_t *foc, float target_ratio, float step_per_call);

void FOC_OutputByTheta(foc_ol_t *foc, float theta, float Uq);
#ifdef __cplusplus
}
#endif

#endif

