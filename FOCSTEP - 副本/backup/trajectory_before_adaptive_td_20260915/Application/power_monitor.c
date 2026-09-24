#include "power_monitor.h"

#include "adc.h"

#define POWER_MONITOR_ADC_FULL_SCALE          (4095UL)
#define POWER_MONITOR_ADC_REFERENCE_MV        (3300UL)
#define POWER_MONITOR_DIVIDER_UPPER_OHM        (20000UL)
#define POWER_MONITOR_DIVIDER_LOWER_OHM        (1000UL)
#define POWER_MONITOR_DIVIDER_NUMERATOR       \
    (POWER_MONITOR_DIVIDER_UPPER_OHM + POWER_MONITOR_DIVIDER_LOWER_OHM)

static PowerMonitor_State_t s_power_monitor;

void PowerMonitor_Init(void)
{
    s_power_monitor.bus_voltage_raw = 0U;
    s_power_monitor.bus_voltage_mv = 0UL;
    PowerMonitor_Update();
}

void PowerMonitor_Update(void)
{
    uint32_t raw = ADC_RegularGetRaw(ADC_REGULAR_BUS_VOLTAGE);
    uint64_t scaled_mv = (uint64_t)raw * POWER_MONITOR_ADC_REFERENCE_MV *
                         POWER_MONITOR_DIVIDER_NUMERATOR;

    scaled_mv /= POWER_MONITOR_DIVIDER_LOWER_OHM;
    scaled_mv = (scaled_mv + (POWER_MONITOR_ADC_FULL_SCALE / 2UL)) /
                POWER_MONITOR_ADC_FULL_SCALE;

    s_power_monitor.bus_voltage_raw = (uint16_t)raw;
    s_power_monitor.bus_voltage_mv = (uint32_t)scaled_mv;
}

const PowerMonitor_State_t *PowerMonitor_GetState(void)
{
    return &s_power_monitor;
}
