#ifndef PULSE_GENERATOR_H
#define PULSE_GENERATOR_H

#include <stdint.h>

/*
 * 单板 STEP 脉冲发生器。
 *
 * 本模块只负责在 TIM3_CH3 比较事件中安排“下一颗 STEP 的时刻”。它不会
 * 向 PulseInput 传递总脉冲数、剩余脉冲数、减速阶段或最后一颗标记；接收
 * 链仍只能通过相邻边沿及无后续边沿的超时自行判断停止。
 *
 * 默认曲线采用外部板同样的三段结构：前 200 颗加速、恒速、后 200 颗
 * 减速。频率按外部板的“600 Hz 下限 + (aimFreq-600)×因子”计算；斜坡
 * 复用当前选用的第 2 号 expFactor_1000 表并按 BO 的 offset=5 抽样。
 * 它仍不替代真实光耦和线缆输入链路验证。
 */

typedef struct
{
    uint32_t total_pulse_count;
    /* 外部板 MC_refreshSpeed() 算出的 aimFreq，不是实际表内最高频率。 */
    uint32_t aim_frequency_hz;
    int8_t direction;
} PulseGenerator_Config_t;

typedef struct
{
    uint32_t requested_pulse_count;
    uint32_t emitted_pulse_count;
    uint32_t aim_frequency_hz;
    uint16_t ramp_pulse_count;
    int8_t direction;
    uint8_t running;
    uint8_t completed;
    uint8_t fault;
    /* 本轮调度诊断；间隔测量指软件实际注入STEP的时刻，不是计划CCR。 */
    uint32_t late_schedule_count;
    uint32_t maximum_injection_interval_us;
} PulseGenerator_State_t;

/* 清零发生器状态并确保 TIM3_CH3 比较中断关闭。 */
void PulseGenerator_Init(void);
/* 校验参数、建立以 aimFreq 为输入的三段频率曲线并逐颗发送。 */
uint8_t PulseGenerator_Start(const PulseGenerator_Config_t *config);
/* 提前停止发生器；不会向 PulseInput 发送“停止”或“最后一颗”通知。 */
void PulseGenerator_Stop(void);
/* 由TIM3 CH3的HAL输出比较回调调用，禁止IRQ入口再重复分发。 */
void PulseGenerator_OnTim3CompareInterrupt(void);
/* 查询发生器是否独占脉冲输入链路。 */
uint8_t PulseGenerator_IsRunning(void);
/* 返回只读运行状态，供串口报告测试进度。 */
const volatile PulseGenerator_State_t *PulseGenerator_GetState(void);

#endif
