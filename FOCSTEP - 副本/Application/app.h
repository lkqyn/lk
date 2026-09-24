#ifndef APP_H
#define APP_H

/*
 * 应用层调度入口。
 * 负责按主循环、1 kHz、4 kHz 三个节拍调用各控制模块；不在本模块实现
 * 具体控制算法。
 */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化应用层。初始化完成后功率PWM保持关闭。
 */
void App_Init(void);

/**
 * @brief 应用层后台任务，由主循环持续调用。
 */
void App_Loop(void);

/**
 * @brief 应用层1ms固定周期任务，由SysTick中断调用。
 * @note 仅允许放置确定执行时间的轻量任务。
 */
void App_Tick1ms(void);

/**
 * @brief 外部STEP位置跟随专用的250us固定周期任务。
 * @note 由TIM4触发，优先级低于ADC电流环和STEP EXTI。
 */
void App_Tick250us(void);

#ifdef __cplusplus
}
#endif

#endif
