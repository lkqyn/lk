#include "adc_task.h"
#include "adc.h"      // hadc1
#include <string.h>

#define CALI_SAMPLES 4000
#define WAIT_MS         300
#define SETTLE_SAMPLES  4000

static volatile uint16_t g_raw_a = 0, g_raw_b = 0;//A相/B相的“零电流偏置”ADC 原始码（0A 对应的 ADC 值）
static volatile uint16_t g_off_a = 2048, g_off_b = 2048;//ADC1 Injected 当前采样的原始码（12bit 一般 0~4095）
static volatile uint8_t  g_cal   = 0;//校准完成标志位
static volatile uint32_t g_sum_a = 0, g_sum_b = 0;//校准阶段累加 rawA/rawB 的求和（用于求平均）
static volatile uint16_t g_cnt   = 0;//校准阶段累计采样次数

static uint32_t start_tick = 0;
static uint16_t settle_cnt = 0;

void ADC_Task_Init(void)
{
  start_tick = HAL_GetTick();
  settle_cnt = 0;

  g_raw_a = g_raw_b = 0;
  g_off_a = g_off_b = 2048;
  g_cal   = 0;
  g_sum_a = g_sum_b = 0;
  g_cnt   = 0;
}

void ADC_Task_StartInjected(void)
{
  // 由 TIM1_TRGO 触发 Injected，启用中断回调
  HAL_ADCEx_InjectedStart_IT(&hadc1);
}

void ADC_Task_OnInjected(uint16_t rawA, uint16_t rawB)
{
  // 不管是否校准，都更新 raw，方便 while 观察
  g_raw_a = rawA;
  g_raw_b = rawB;

  if (g_cal) return;

  // 上电延时等待
  if ((HAL_GetTick() - start_tick) < WAIT_MS) return;

  // 丢弃 settle
  if (settle_cnt < SETTLE_SAMPLES) { settle_cnt++; return; }

  // 平均校准
  g_sum_a += rawA;
  g_sum_b += rawB;
  if (++g_cnt >= CALI_SAMPLES) {
    g_off_a = g_sum_a / g_cnt;
    g_off_b = g_sum_b / g_cnt;
    g_cal = 1;
  }
}

uint8_t  ADC_Task_IsCalibrated(void) { return g_cal; }
uint16_t ADC_Task_OffA(void)         { return g_off_a; }
uint16_t ADC_Task_OffB(void)         { return g_off_b; }
uint16_t ADC_Task_RawA(void)         { return g_raw_a; }
uint16_t ADC_Task_RawB(void)         { return g_raw_b; }
int32_t  ADC_Task_DeltaA(void)       { return (int32_t)g_raw_a - (int32_t)g_off_a; }
int32_t  ADC_Task_DeltaB(void)       { return (int32_t)g_raw_b - (int32_t)g_off_b; }
