#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#include "main.h"
#include <stdint.h>

typedef struct
{
  uint8_t feedDetected;
  uint8_t rewindDetected;
  uint8_t printRequestDetected;
} AppSensorState;

void AppSensor_Update(void);
AppSensorState AppSensor_GetState(void);
uint8_t AppSensor_IsFeedDetected(void);
uint8_t AppSensor_IsRewindDetected(void);
uint8_t AppSensor_IsPrintRequestDetected(void);

#endif
