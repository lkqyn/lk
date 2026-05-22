#include "app_main.h"
#include "app_motor.h"
#include "app_printer.h"
#include "app_sensor.h"
#include "cmsis_os.h"
#include <stdio.h>

void AppTask(void *argument)
{
  (void)argument;

  printf("Application task start\r\n");
  printf("Feed sensor:   CN19-2 / PD15, detected = 1\r\n");
  printf("Rewind sensor: CN19-4 / PG3, detected = 1\r\n");

  AppMotor_Init();
  osDelay(2000);
  printf("Send printer test ZPL by UART5/CN5\r\n");
  AppPrinter_TestPrint();

  uint32_t lastPrintTick = 0;

  for (;;)
  {
    AppSensor_Update();

    if (osKernelGetTickCount() - lastPrintTick >= 500)
    {
      lastPrintTick = osKernelGetTickCount();

      printf("CH1 Feed=%d Rewind=%d Run=%d Alarm=%d DacOk=%d DacRdy=%d STAN=%d EN=%d STEP=%d StepCnt=%lu FR=%d MD=%d%d%d tick=%lu\r\n",
             AppSensor_IsFeedDetected(),
             AppSensor_IsRewindDetected(),
             AppMotor_IsRewindRunning(),
             AppMotor_IsRewindAlarm(),
             AppMotor_GetDacInitOk(),
             AppMotor_IsDacReady(),
             HAL_GPIO_ReadPin(STAN_GPIO_Port, STAN_Pin),
             HAL_GPIO_ReadPin(EN_CH1_GPIO_Port, EN_CH1_Pin),
             HAL_GPIO_ReadPin(STEP_CH1_GPIO_Port, STEP_CH1_Pin),
             (unsigned long)AppMotor_GetStepCount(),
             HAL_GPIO_ReadPin(FR_CH1_GPIO_Port, FR_CH1_Pin),
             HAL_GPIO_ReadPin(MD0_CH1_GPIO_Port, MD0_CH1_Pin),
             HAL_GPIO_ReadPin(MD1_CH1_GPIO_Port, MD1_CH1_Pin),
             HAL_GPIO_ReadPin(MD2_CH1_GPIO_Port, MD2_CH1_Pin),
             (unsigned long)lastPrintTick);
    }

    osDelay(1);
  }
}
