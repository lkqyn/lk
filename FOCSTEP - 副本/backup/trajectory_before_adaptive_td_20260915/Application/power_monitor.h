#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

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
