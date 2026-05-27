#include "app_motor.h"
#include "i2c.h"
#include "main.h"
#include "cmsis_os.h"

#define MOTOR_ENABLE_LEVEL   GPIO_PIN_RESET
#define MOTOR_DISABLE_LEVEL  GPIO_PIN_SET

#define MOTOR_STEP_IDLE      GPIO_PIN_SET
#define MOTOR_STEP_ACTIVE    GPIO_PIN_RESET

#define MOTOR_STAN_RUN_LEVEL GPIO_PIN_SET
#define REWIND_STEP_INTERVAL_MS 10U

#define MOTOR_LOD_GPIO_Port   LOD_CH1_GPIO_Port
#define MOTOR_LOD_Pin         LOD_CH1_Pin
#define MOTOR_MD0_GPIO_Port   MD0_CH1_GPIO_Port
#define MOTOR_MD0_Pin         MD0_CH1_Pin
#define MOTOR_MD1_GPIO_Port   MD1_CH1_GPIO_Port
#define MOTOR_MD1_Pin         MD1_CH1_Pin
#define MOTOR_MD2_GPIO_Port   MD2_CH1_GPIO_Port
#define MOTOR_MD2_Pin         MD2_CH1_Pin
#define MOTOR_FR_GPIO_Port    FR_CH1_GPIO_Port
#define MOTOR_FR_Pin          FR_CH1_Pin
#define MOTOR_STEP_GPIO_Port  STEP_CH1_GPIO_Port
#define MOTOR_STEP_Pin        STEP_CH1_Pin
#define MOTOR_EN_GPIO_Port    EN_CH1_GPIO_Port
#define MOTOR_EN_Pin          EN_CH1_Pin

#define MCP4728_ADDR         (0x60 << 1)
#define MCP4728_MULTI_WRITE  0x40
#define MCP4728_VDD_MV       3300U
#define MOTOR_VREF_MV        1500U

static uint8_t g_rewindRunning;
static uint8_t g_dacInitOk;
static uint32_t g_lastStepTick;
static uint32_t g_stepCount;
static GPIO_PinState g_stepLevel = MOTOR_STEP_IDLE;

static void AppMotor_SetFullStep(void)
{
  HAL_GPIO_WritePin(MOTOR_MD0_GPIO_Port, MOTOR_MD0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_MD1_GPIO_Port, MOTOR_MD1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_MD2_GPIO_Port, MOTOR_MD2_Pin, GPIO_PIN_RESET);
}

static HAL_StatusTypeDef AppMotor_SetDacChannel(uint8_t channel, uint16_t value)
{
  uint8_t data[3];

  if (channel > 3)
  {
    return HAL_ERROR;
  }

  if (value > 4095)
  {
    value = 4095;
  }

  data[0] = MCP4728_MULTI_WRITE | (channel << 1);
  data[1] = (uint8_t)((value >> 8) & 0x0F);
  data[2] = (uint8_t)(value & 0xFF);

  return HAL_I2C_Master_Transmit(&hi2c2, MCP4728_ADDR, data, sizeof(data), 100);
}

static HAL_StatusTypeDef AppMotor_SetAllVref(uint16_t millivolt)
{
  uint16_t dacValue;
  HAL_StatusTypeDef status = HAL_OK;

  if (millivolt > MCP4728_VDD_MV)
  {
    millivolt = MCP4728_VDD_MV;
  }

  dacValue = (uint16_t)(((uint32_t)millivolt * 4095U) / MCP4728_VDD_MV);

  for (uint8_t channel = 0; channel < 4; channel++)
  {
    if (AppMotor_SetDacChannel(channel, dacValue) != HAL_OK)
    {
      status = HAL_ERROR;
    }
  }

  return status;
}

void AppMotor_Init(void)
{
  g_rewindRunning = 0;
  g_lastStepTick = 0;
  g_stepCount = 0;
  g_stepLevel = MOTOR_STEP_IDLE;

  HAL_GPIO_WritePin(STAN_GPIO_Port, STAN_Pin, MOTOR_STAN_RUN_LEVEL);

  AppMotor_SetFullStep();
  g_dacInitOk = (AppMotor_SetAllVref(MOTOR_VREF_MV) == HAL_OK);

  HAL_GPIO_WritePin(MOTOR_FR_GPIO_Port, MOTOR_FR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, MOTOR_STEP_IDLE);
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, MOTOR_DISABLE_LEVEL);
}

void AppMotor_StartRewind(void)
{
  g_rewindRunning = 1;
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, MOTOR_ENABLE_LEVEL);
}

void AppMotor_StopRewind(void)
{
  g_rewindRunning = 0;
  g_stepLevel = MOTOR_STEP_IDLE;
  HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, MOTOR_STEP_IDLE);
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, MOTOR_DISABLE_LEVEL);
}

void AppMotor_RewindTask(void)
{
  if (!g_rewindRunning)
  {
    return;
  }

  uint32_t now = osKernelGetTickCount();

  if (now - g_lastStepTick >= REWIND_STEP_INTERVAL_MS)
  {
    g_lastStepTick = now;
    g_stepLevel = (g_stepLevel == MOTOR_STEP_IDLE) ? MOTOR_STEP_ACTIVE : MOTOR_STEP_IDLE;
    HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, g_stepLevel);
    if (g_stepLevel == MOTOR_STEP_ACTIVE)
    {
      g_stepCount++;
    }
  }
}

uint8_t AppMotor_IsRewindAlarm(void)
{
  return HAL_GPIO_ReadPin(MOTOR_LOD_GPIO_Port, MOTOR_LOD_Pin) == GPIO_PIN_RESET;
}

uint8_t AppMotor_IsRewindRunning(void)
{
  return g_rewindRunning;
}

uint8_t AppMotor_IsDacReady(void)
{
  return HAL_GPIO_ReadPin(DAC_RDY_GPIO_Port, DAC_RDY_Pin) == GPIO_PIN_SET;
}

uint8_t AppMotor_GetDacInitOk(void)
{
  return g_dacInitOk;
}

uint32_t AppMotor_GetStepCount(void)
{
  return g_stepCount;
}

void AppMotor_ForceStepTest(void)
{
  HAL_GPIO_WritePin(STAN_GPIO_Port, STAN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, MOTOR_ENABLE_LEVEL);
  HAL_GPIO_WritePin(MOTOR_FR_GPIO_Port, MOTOR_FR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_MD0_GPIO_Port, MOTOR_MD0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_MD1_GPIO_Port, MOTOR_MD1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_MD2_GPIO_Port, MOTOR_MD2_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, MOTOR_STEP_ACTIVE);
  osDelay(100);
  HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, MOTOR_STEP_IDLE);
  osDelay(100);
  g_stepCount++;
}
