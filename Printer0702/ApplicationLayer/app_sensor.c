#include "app_sensor.h"

static AppSensorState g_sensorState;

void AppSensor_Update(void)
{
  g_sensorState.feedDetected =
      (HAL_GPIO_ReadPin(IN_CN19_2_GPIO_Port, IN_CN19_2_Pin) == GPIO_PIN_RESET);

  g_sensorState.rewindDetected =
      (HAL_GPIO_ReadPin(IN_CN19_4_GPIO_Port, IN_CN19_4_Pin) == GPIO_PIN_RESET);

  g_sensorState.printRequestDetected =
      (HAL_GPIO_ReadPin(PRINT_REQ_IN_GPIO_Port, PRINT_REQ_IN_Pin) == GPIO_PIN_RESET);
}

AppSensorState AppSensor_GetState(void)
{
  return g_sensorState;
}

uint8_t AppSensor_IsFeedDetected(void)
{
  return g_sensorState.feedDetected;
}

uint8_t AppSensor_IsRewindDetected(void)
{
  return g_sensorState.rewindDetected;
}

uint8_t AppSensor_IsPrintRequestDetected(void)
{
  return g_sensorState.printRequestDetected;
}
