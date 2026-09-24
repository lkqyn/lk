#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

/*
 * 母线电压监测模块。
 * 将 ADC 常规通道原始值换算为 mV，供速度环弱磁和过欠压保护使用。
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    uint16_t bus_voltage_raw;
    uint32_t bus_voltage_mv;
} PowerMonitor_State_t;

/**
 * @brief 初始化母线电压状态。
 */
void PowerMonitor_Init(void);

/**
 * @brief 根据ADC常规组最新值更新母线电压。
 */
void PowerMonitor_Update(void);

/**
 * @brief 获取母线电压状态。
 */
const PowerMonitor_State_t *PowerMonitor_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
