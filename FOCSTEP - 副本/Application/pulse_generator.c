/*
 * 单板 STEP 发生器实现。
 *
 * TIM3 与外部 STEP 捕获共用同一 10.5 MHz 自由运行时基：CH2 保留给 PC7
 * 输入捕获，CH3 仅使用比较中断、不配置 GPIO 输出。每次 CH3 到点后先
 * 注入一颗模拟 STEP，再只安排下一颗。频率表来自外部板当前
 * CONF_S_1000_FACTOR_NO=2 的 expFactor_1000[]，并按 CONF_S_OFFSET_BO=5
 * 每五点抽样一次；最后一颗后仅关闭 CC3IE，因此 PulseInput 不会提前获知
 * 动作结束。
 */
#include "pulse_generator.h"

#include "pulse_input.h"
#include "tim.h"

#define PULSE_GENERATOR_TIMER_HZ              (10500000UL)
#define PULSE_GENERATOR_EXTERNAL_TIMER_HZ       (2000000UL)
#define PULSE_GENERATOR_MINIMUM_FREQUENCY_HZ      (600UL)
#define PULSE_GENERATOR_MAXIMUM_FREQUENCY_HZ    (50000UL)
#define PULSE_GENERATOR_EXTERNAL_RAMP_PULSES      (200U)

static volatile PulseGenerator_State_t s_pulse_generator;
/* 启动时计算周期，中断内只查表，避免逐颗执行64位除法。 */
static uint16_t s_interval_table[PULSE_GENERATOR_EXTERNAL_RAMP_PULSES];
static uint32_t s_last_injection_cpu_cycle;
static uint32_t s_cpu_cycles_per_us;

/*
 * 按外部板的 2 MHz 整数 ARR 计算周期，再换算为本机 TIM3 的 10.5 MHz
 * 比较间隔。这样保留外部板除法截断带来的实际边沿时间，而非只保留理想 Hz。
 */
static uint16_t PulseGenerator_ExternalRateToInterval(
    uint32_t frequency_x10000)
{
    uint64_t external_arr;
    uint64_t interval;

    external_arr = ((uint64_t)PULSE_GENERATOR_EXTERNAL_TIMER_HZ * 10000ULL) /
                   frequency_x10000;
    interval = (external_arr * 21ULL + 2ULL) / 4ULL;
    if (interval == 0U)
    {
        interval = 1U;
    }
    return (uint16_t)interval;
}

/*
 * 外部板 expFactor_1000[0,5,...,995] × 10000。
 * 外部 BO_MOTOR 使用 1000 点表、offset=5，故前/后 200 脉冲恰好逐项
 * 对应此表。首/末值 0.1192、表末值 0.8787 均保留外部板当前实际行为，
 * 不把“600 Hz 下限”误当成每次动作的第一或最后一个脉冲频率。
 */
static const uint16_t s_external_bo_factor_x10000[
    PULSE_GENERATOR_EXTERNAL_RAMP_PULSES] =
{
    1192,1213,1235,1256,1279,1301,1324,1347,1371,1394,
    1419,1443,1468,1493,1519,1545,1571,1598,1625,1652,
    1680,1708,1736,1765,1795,1824,1854,1885,1915,1947,
    1978,2010,2042,2075,2108,2142,2176,2210,2244,2279,
    2315,2351,2387,2423,2460,2497,2535,2573,2611,2650,
    2689,2729,2769,2809,2850,2891,2932,2973,3015,3058,
    3100,3143,3186,3230,3274,3318,3363,3407,3452,3498,
    3543,3589,3635,3682,3729,3775,3823,3870,3917,3965,
    4013,4061,4110,4158,4207,4256,4305,4354,4403,4452,
    4502,4551,4601,4651,4700,4750,4800,4850,4900,4950,
    5000,5050,5100,5150,5200,5250,5300,5349,5399,5449,
    5498,5548,5597,5646,5695,5744,5793,5842,5890,5939,
    5987,6035,6083,6130,6177,6225,6271,6318,6365,6411,
    6457,6502,6548,6593,6637,6682,6726,6770,6814,6857,
    6900,6942,6985,7027,7068,7109,7150,7191,7231,7271,
    7311,7350,7389,7427,7465,7503,7540,7577,7613,7649,
    7685,7721,7756,7790,7824,7858,7892,7925,7958,7990,
    8022,8053,8085,8115,8146,8176,8205,8235,8264,8292,
    8320,8348,8375,8402,8429,8455,8481,8507,8532,8557,
    8581,8606,8629,8653,8676,8699,8721,8744,8765,8787
};

/*
 * 读取第 index 个 BO 斜坡点对应的预计算定时周期。短行程只走表的前半段，
 * 不会把较短斜坡重新拉伸到第 200 点。
 */
static uint16_t PulseGenerator_GetRampInterval(uint16_t index,
                                                  uint16_t ramp_count)
{
    if (ramp_count == 0U)
    {
        return s_interval_table[
            PULSE_GENERATOR_EXTERNAL_RAMP_PULSES - 1U];
    }
    if (ramp_count == 1U)
    {
        return s_interval_table[0U];
    }
    if (index >= PULSE_GENERATOR_EXTERNAL_RAMP_PULSES)
    {
        index = PULSE_GENERATOR_EXTERNAL_RAMP_PULSES - 1U;
    }
    return s_interval_table[index];
}

/* 按当前脉冲索引查找定时周期，单位为10.5MHz计数。
 * aimFreq 是外部 MC_refreshSpeed() 的输入；第200点只有 0.8787 倍，
 * 因而实际最高频率并不等于 aimFreq，这是对外部板现状的刻意复刻。 */
static uint32_t PulseGenerator_GetInterval(
    uint32_t emitted_pulse_count)
{
    uint32_t interval;
    uint16_t ramp = s_pulse_generator.ramp_pulse_count;
    uint32_t total = s_pulse_generator.requested_pulse_count;

    if ((ramp != 0U) && (emitted_pulse_count < ramp))
    {
        interval = PulseGenerator_GetRampInterval(
            (uint16_t)emitted_pulse_count, ramp);
    }
    else if ((ramp != 0U) && (emitted_pulse_count >= (total - ramp)))
    {
        uint32_t deceleration_index = emitted_pulse_count - (total - ramp);

        interval = PulseGenerator_GetRampInterval(
            (uint16_t)(ramp - 1U - deceleration_index), ramp);
    }
    else
    {
        interval = s_interval_table[
            PULSE_GENERATOR_EXTERNAL_RAMP_PULSES - 1U];
    }
    return interval;
}

/* 短临界区完成检查与CCR写入；过期时延后一个正常周期，不突发补发。 */
static void PulseGenerator_ScheduleNext(uint16_t previous_compare, uint32_t interval)
{
    uint32_t primask = __get_PRIMASK();
    uint16_t now;
    uint16_t next = (uint16_t)(previous_compare + interval);
    __disable_irq();
    now = (uint16_t)TIM3->CNT;
    /* 所有周期均小于半个16位量程；预留约2us完成寄存器写入。 */
    if ((int16_t)(uint16_t)(next - now) <= 21)
    {
        next = (uint16_t)(now + interval);
        s_pulse_generator.late_schedule_count++;
    }
    TIM3->CCR3 = next;
    if (primask == 0U) __enable_irq();
}

/* 关闭 CH3 比较中断；不触碰 CH2 捕获、TIM3 更新中断或计数器。 */
static void PulseGenerator_DisableCompareInterrupt(void)
{
    TIM3->DIER &= ~TIM_DIER_CC3IE;
    TIM3->SR = ~TIM_SR_CC3IF;
}

/* 清零运行状态；此函数不改变 PulseInput 的位置或速度状态。 */
void PulseGenerator_Init(void)
{
    PulseGenerator_DisableCompareInterrupt();
    s_pulse_generator.requested_pulse_count = 0U;
    s_pulse_generator.emitted_pulse_count = 0U;
    s_pulse_generator.aim_frequency_hz = 0U;
    s_pulse_generator.ramp_pulse_count = 0U;
    s_pulse_generator.direction = 0;
    s_pulse_generator.running = 0U;
    s_pulse_generator.completed = 0U;
    s_pulse_generator.fault = 0U;
    s_pulse_generator.late_schedule_count = 0U;
    s_pulse_generator.maximum_injection_interval_us = 0U;
}

/* 建立单板测试脉冲流；PulseInput 仍不知道总数和末脉冲位置。 */
uint8_t PulseGenerator_Start(const PulseGenerator_Config_t *config)
{
    uint32_t interval;
    uint32_t index;
    uint32_t primask = __get_PRIMASK();

    if ((config == 0) || (config->total_pulse_count == 0U) ||
        (config->aim_frequency_hz < PULSE_GENERATOR_MINIMUM_FREQUENCY_HZ) ||
        (config->aim_frequency_hz > PULSE_GENERATOR_MAXIMUM_FREQUENCY_HZ) ||
        ((config->direction != 1) && (config->direction != -1)) ||
        (PulseInput_GetState()->enabled == 0U))
    {
        return 0U;
    }

    if (s_pulse_generator.running != 0U) return 0U;
    /* 主循环中预计算原表对应周期，不占用关中断时间。 */
    for (index = 0U; index < PULSE_GENERATOR_EXTERNAL_RAMP_PULSES; index++)
    {
        uint32_t rate = PULSE_GENERATOR_MINIMUM_FREQUENCY_HZ * 10000U +
            (config->aim_frequency_hz - PULSE_GENERATOR_MINIMUM_FREQUENCY_HZ) *
            s_external_bo_factor_x10000[index];
        s_interval_table[index] = PulseGenerator_ExternalRateToInterval(rate);
    }
    /* DWT只用于观测注入间隔，不清零，避免干扰其他性能测量。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_cpu_cycles_per_us = SystemCoreClock / 1000000U;
    if (s_cpu_cycles_per_us == 0U) return 0U;
    __disable_irq();
    if (s_pulse_generator.running != 0U)
    {
        if (primask == 0U) __enable_irq();
        return 0U;
    }

    s_pulse_generator.requested_pulse_count = config->total_pulse_count;
    s_pulse_generator.emitted_pulse_count = 0U;
    s_pulse_generator.aim_frequency_hz = config->aim_frequency_hz;
    s_pulse_generator.direction = config->direction;
    s_pulse_generator.completed = 0U;
    s_pulse_generator.fault = 0U;
    s_pulse_generator.late_schedule_count = 0U;
    s_pulse_generator.maximum_injection_interval_us = 0U;
    s_last_injection_cpu_cycle = 0U;
    s_pulse_generator.ramp_pulse_count =
        (config->total_pulse_count > (PULSE_GENERATOR_EXTERNAL_RAMP_PULSES * 2U)) ?
        PULSE_GENERATOR_EXTERNAL_RAMP_PULSES :
        (uint16_t)((config->total_pulse_count > 4U) ?
                   ((config->total_pulse_count / 2U) - 2U) : 0U);

    interval = PulseGenerator_GetInterval(0U);
    TIM3->CCR3 = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim3) + interval);
    TIM3->SR = ~TIM_SR_CC3IF;
    s_pulse_generator.running = 1U;
    TIM3->DIER |= TIM_DIER_CC3IE;
    if (primask == 0U) __enable_irq();
    return 1U;
}

/* 用户中止：仅停止未来的模拟边沿，不向接收链发送结束事件。 */
void PulseGenerator_Stop(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    PulseGenerator_DisableCompareInterrupt();
    s_pulse_generator.running = 0U;
    if (primask == 0U) __enable_irq();
}

/* TIM3_CH3 到点：注入一颗边沿，最后一颗之后静默停止比较事件。 */
void PulseGenerator_OnTim3CompareInterrupt(void)
{
    uint16_t capture_count;
    uint16_t actual_count;
    uint32_t interval;
    uint32_t cpu_cycle;

    if (s_pulse_generator.running == 0U)
    {
        return;
    }

    capture_count = (uint16_t)TIM3->CCR3;
    actual_count = (uint16_t)TIM3->CNT;
    cpu_cycle = DWT->CYCCNT;
    /* 软件模拟没有物理捕获边沿，以实际注入时刻作为接收端时间戳。 */
    PulseInput_OnSimulatedStepCaptured(actual_count,
                                       s_pulse_generator.direction);
    if (s_pulse_generator.emitted_pulse_count != 0U)
    {
        uint32_t gap_us = (cpu_cycle - s_last_injection_cpu_cycle) / s_cpu_cycles_per_us;
        if (gap_us > s_pulse_generator.maximum_injection_interval_us)
        {
            s_pulse_generator.maximum_injection_interval_us = gap_us;
        }
    }
    s_last_injection_cpu_cycle = cpu_cycle;
    s_pulse_generator.emitted_pulse_count++;

    if (s_pulse_generator.emitted_pulse_count >=
        s_pulse_generator.requested_pulse_count)
    {
        /* 没有“结束通知”：接收链只能按自身无脉冲逻辑发现停止。 */
        PulseGenerator_DisableCompareInterrupt();
        s_pulse_generator.running = 0U;
        s_pulse_generator.completed = 1U;
        return;
    }

    interval = PulseGenerator_GetInterval(s_pulse_generator.emitted_pulse_count);
    PulseGenerator_ScheduleNext(capture_count, interval);
}

/* 返回发生器是否正在独占模拟 STEP 输入。 */
uint8_t PulseGenerator_IsRunning(void)
{
    return s_pulse_generator.running;
}

/* 返回发生器运行状态；成员均为原子字宽，串口读取仅用于诊断。 */
const volatile PulseGenerator_State_t *PulseGenerator_GetState(void)
{
    return &s_pulse_generator;
}
