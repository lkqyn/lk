#include "app_main.h"
#include "app_printer.h"
#include "app_sensor.h"
#include "cmsis_os.h"
#include "usart.h"
#include <stdio.h>

#define PRINT_DONE_ACTIVE_LEVEL   GPIO_PIN_RESET
#define PRINT_DONE_IDLE_LEVEL     GPIO_PIN_SET
#define PRINT_REQ_DEBOUNCE_MS     30U
#define PRINT_MIN_TIME_MS         1200U
#define PRINT_DONE_TIMEOUT_MS     15000U
#define PRINT_DONE_POLL_MS        300U

typedef enum
{
  PRINT_STATE_IDLE = 0,
  PRINT_STATE_WAIT_RELEASE
} PrintState;

static uint8_t AppDebug_ReadManualPrintCommand(void)
{
  uint8_t data = 0;

  if (HAL_UART_Receive(&huart1, &data, 1, 0) != HAL_OK)
  {
    return 0;
  }

  return (data == 'p' || data == 'P');
}

void AppTask(void *argument)
{
  uint32_t lastDebugTick = 0;
  PrintState printState = PRINT_STATE_IDLE;
  uint8_t manualPrintActive = 0;
  uint8_t requestActive = 0;
  uint8_t lastRequestActive = 0;
  uint8_t lastRequestPin = 0;
  uint8_t printTrigger = 0;

  (void)argument;

  printf("Application task start\r\n");
  printf("PRINT_REQ:  PG4 / INPUT_05, low active\r\n");
  printf("PRINT_DONE: PE15 / OUTPUT_02, low active\r\n");
  printf("Manual test: send 'p' from serial tool to print once\r\n");

  HAL_GPIO_WritePin(PRINT_DONE_OUT_GPIO_Port, PRINT_DONE_OUT_Pin, PRINT_DONE_IDLE_LEVEL);
  AppSensor_Update();
  lastRequestActive = AppSensor_IsPrintRequestDetected();
  lastRequestPin = HAL_GPIO_ReadPin(PRINT_REQ_IN_GPIO_Port, PRINT_REQ_IN_Pin);
  if (lastRequestActive)
  {
    printf("Power on with request active, wait request release first\r\n");
  }

  for (;;)
  {
    AppSensor_Update();
    requestActive = AppSensor_IsPrintRequestDetected();
    printTrigger = 0;

    if (HAL_GPIO_ReadPin(PRINT_REQ_IN_GPIO_Port, PRINT_REQ_IN_Pin) != lastRequestPin)
    {
      lastRequestPin = HAL_GPIO_ReadPin(PRINT_REQ_IN_GPIO_Port, PRINT_REQ_IN_Pin);
      printf("REQ_PIN changed to %d, Req=%d\r\n",
             lastRequestPin,
             AppSensor_IsPrintRequestDetected());
    }

    if (printState == PRINT_STATE_IDLE && AppDebug_ReadManualPrintCommand())
    {
      manualPrintActive = 1;
      printTrigger = 1;
      printf("Manual print command received\r\n");
    }
    else if (printState == PRINT_STATE_IDLE && requestActive && !lastRequestActive)
    {
      printTrigger = 1;
      printf("External print falling edge detected\r\n");
    }

    switch (printState)
    {
      case PRINT_STATE_IDLE:
        if (printTrigger)
        {
          osDelay(PRINT_REQ_DEBOUNCE_MS);
          AppSensor_Update();
          requestActive = AppSensor_IsPrintRequestDetected() || manualPrintActive;

          if (requestActive)
          {
            printf("Print request active, send ZPL\r\n");
            HAL_GPIO_WritePin(PRINT_DONE_OUT_GPIO_Port, PRINT_DONE_OUT_Pin, PRINT_DONE_IDLE_LEVEL);
            AppPrinter_TestPrint();

            osDelay(PRINT_MIN_TIME_MS);

            printf("Wait printer done by ~HS\r\n");
            if (AppPrinter_WaitDone(PRINT_DONE_TIMEOUT_MS, PRINT_DONE_POLL_MS))
            {
              printf("Printer done detected\r\n");
            }
            else
            {
              printf("Printer done wait timeout, check printer state\r\n");
            }

            HAL_GPIO_WritePin(PRINT_DONE_OUT_GPIO_Port, PRINT_DONE_OUT_Pin, PRINT_DONE_ACTIVE_LEVEL);
            printf("Print done output active, wait request release\r\n");
            printState = PRINT_STATE_WAIT_RELEASE;
          }
        }
        break;

      case PRINT_STATE_WAIT_RELEASE:
        if (manualPrintActive)
        {
          osDelay(1000);
          manualPrintActive = 0;
          HAL_GPIO_WritePin(PRINT_DONE_OUT_GPIO_Port, PRINT_DONE_OUT_Pin, PRINT_DONE_IDLE_LEVEL);
          printf("Manual print done output idle\r\n");
          printState = PRINT_STATE_IDLE;
        }
        else if (!AppSensor_IsPrintRequestDetected())
        {
          HAL_GPIO_WritePin(PRINT_DONE_OUT_GPIO_Port, PRINT_DONE_OUT_Pin, PRINT_DONE_IDLE_LEVEL);
          printf("Print request released, done output idle\r\n");
          printState = PRINT_STATE_IDLE;
        }
        break;

      default:
        printState = PRINT_STATE_IDLE;
        HAL_GPIO_WritePin(PRINT_DONE_OUT_GPIO_Port, PRINT_DONE_OUT_Pin, PRINT_DONE_IDLE_LEVEL);
        break;
    }

    lastRequestActive = requestActive;

    if (osKernelGetTickCount() - lastDebugTick >= 500)
    {
      lastDebugTick = osKernelGetTickCount();

      printf("State=%d REQ_PIN=%d Req=%d DoneOut=%d\r\n",
             printState,
             HAL_GPIO_ReadPin(PRINT_REQ_IN_GPIO_Port, PRINT_REQ_IN_Pin),
             AppSensor_IsPrintRequestDetected(),
             HAL_GPIO_ReadPin(PRINT_DONE_OUT_GPIO_Port, PRINT_DONE_OUT_Pin));
    }

    osDelay(1);
  }
}
