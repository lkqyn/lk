/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "usbd_cdc.h"
#include "usbd_core.h"
#include "usb_device.h"
#include "vofa.h"
#include "feeder.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile canopen_control_t g_usb_vcp_cmd;
static char g_usb_vcp_line[128];
static uint16_t g_usb_vcp_line_len;
volatile uint8_t g_usb_vcp_wave_enable = 1U;
volatile uint8_t g_usb_vcp_force_pulse_view = 0U;
volatile uint8_t g_usb_vcp_manual_lead_enable = 0U;
volatile int16_t g_usb_vcp_manual_lead_angle = 0;
volatile uint8_t g_usb_vcp_manual_uq_enable = 0U;
volatile int16_t g_usb_vcp_manual_uq_centivolt = 0;
volatile int16_t g_usb_vcp_fwv_centivolt = 0;
volatile uint8_t g_usb_vcp_speed_pi_update_request = 0U;
volatile uint16_t g_usb_vcp_speed_kp_milli = 4U;
volatile uint16_t g_usb_vcp_speed_ki_10000 = 8U;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
static void USBVCP_HandleLine(const char *line);
static uint8_t USBVCP_QueueCommand(uint8_t command, uint8_t mode, int16_t value1, uint16_t value2, uint16_t value3);
static int USBVCP_ParseInt16(const char *text, int16_t *out_value);
static int USBVCP_ParseUint16(const char *text, uint16_t *out_value);
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  uint32_t i;

  for (i = 0U; i < *Len; i++)
  {
    char ch = (char)Buf[i];

    if ((ch == '\r') || (ch == '\n'))
    {
      if (g_usb_vcp_line_len > 0U)
      {
        g_usb_vcp_line[g_usb_vcp_line_len] = '\0';
        USBVCP_HandleLine(g_usb_vcp_line);
        g_usb_vcp_line_len = 0U;
      }
    }
    else if (g_usb_vcp_line_len < (sizeof(g_usb_vcp_line) - 1U))
    {
      g_usb_vcp_line[g_usb_vcp_line_len++] = ch;
    }
    else
    {
      g_usb_vcp_line_len = 0U;
    }
  }

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  VOFA_TxCpltCallback();
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
uint8_t USBVCP_GetCommand(canopen_control_t *out_cmd)
{
  if ((out_cmd == NULL) || (g_usb_vcp_cmd.command_pending == 0U))
  {
    return 0U;
  }

  *out_cmd = g_usb_vcp_cmd;
  g_usb_vcp_cmd.command_pending = 0U;
  return 1U;
}

void USBVCP_SendString(const char *str)
{
  if (str == NULL)
  {
    return;
  }

  (void)CDC_Transmit_FS((uint8_t *)str, (uint16_t)strlen(str));
}

static uint8_t USBVCP_QueueCommand(uint8_t command, uint8_t mode, int16_t value1, uint16_t value2, uint16_t value3)
{
  g_usb_vcp_cmd.command = command;
  g_usb_vcp_cmd.mode = mode;
  g_usb_vcp_cmd.target_speed_rpm = value1;
  g_usb_vcp_cmd.value1 = value1;
  g_usb_vcp_cmd.value2 = value2;
  g_usb_vcp_cmd.value3 = value3;
  g_usb_vcp_cmd.value32 = 0;
  g_usb_vcp_cmd.command_pending = 1U;
  return 1U;
}

static int USBVCP_ParseInt16(const char *text, int16_t *out_value)
{
  char *endptr;
  long value;

  if ((text == NULL) || (out_value == NULL))
  {
    return 0;
  }

  value = strtol(text, &endptr, 10);
  if ((*text == '\0') || (*endptr != '\0') || (value < -32768L) || (value > 32767L))
  {
    return 0;
  }

  *out_value = (int16_t)value;
  return 1;
}

static int USBVCP_ParseUint16(const char *text, uint16_t *out_value)
{
  char *endptr;
  unsigned long value;

  if ((text == NULL) || (out_value == NULL))
  {
    return 0;
  }

  value = strtoul(text, &endptr, 10);
  if ((*text == '\0') || (*endptr != '\0') || (value > 65535UL))
  {
    return 0;
  }

  *out_value = (uint16_t)value;
  return 1;
}

static void USBVCP_HandleLine(const char *line)
{
  char temp[128];
  char *token0;
  char *token1;
  char *token2;
  char *token3;
  char *token4;
  int16_t value1;
  uint16_t value2;
  uint16_t value3;

  if (line == NULL)
  {
    return;
  }

  strncpy(temp, line, sizeof(temp) - 1U);
  temp[sizeof(temp) - 1U] = '\0';

  token0 = strtok(temp, "+");
  token1 = strtok(NULL, "+");
  token2 = strtok(NULL, "+");
  token3 = strtok(NULL, "+");
  token4 = strtok(NULL, "+");

  if ((token0 == NULL) || (strcmp(token0, "yiw") != 0) || (token1 == NULL))
  {
    return;
  }

  if ((strcmp(token1, "STOP") == 0) || (strcmp(token1, "stop") == 0))
  {
    g_usb_vcp_wave_enable = 0U;
    USBVCP_QueueCommand(CANOPEN_CMD_STOP, CANOPEN_MODE_IDLE, 0, 0U, 0U);
    return;
  }

  if ((strcmp(token1, "ZERO") == 0) || (strcmp(token1, "zero") == 0))
  {
    USBVCP_QueueCommand(CANOPEN_CMD_ZERO, CANOPEN_MODE_IDLE, 0, 0U, 0U);
    return;
  }

  if ((strcmp(token1, "PULSEVIEW") == 0) || (strcmp(token1, "pulseview") == 0))
  {
    g_usb_vcp_wave_enable = 1U;
    g_usb_vcp_force_pulse_view = 1U;
    return;
  }

  if ((strcmp(token1, "PULSESIM") == 0) && (token2 != NULL))
  {
    if (strcmp(token2, "ON") == 0)
    {
      g_pulse_sim_enable = 1U;
    }
    else if (strcmp(token2, "OFF") == 0)
    {
      g_pulse_sim_enable = 0U;
    }
    return;
  }

  if ((strcmp(token1, "PULSEDIR") == 0) && (token2 != NULL))
  {
    if ((strcmp(token2, "FWD") == 0) || (strcmp(token2, "1") == 0))
    {
      g_pulse_sim_dir_positive = 1U;
    }
    else if ((strcmp(token2, "REV") == 0) || (strcmp(token2, "0") == 0))
    {
      g_pulse_sim_dir_positive = 0U;
    }
    return;
  }

  if ((strcmp(token1, "PULSEEN") == 0) && (token2 != NULL))
  {
    if ((strcmp(token2, "ON") == 0) || (strcmp(token2, "1") == 0))
    {
      g_pulse_sim_en_level = 1U;
    }
    else if ((strcmp(token2, "OFF") == 0) || (strcmp(token2, "0") == 0))
    {
      g_pulse_sim_en_level = 0U;
    }
    return;
  }

  if ((strcmp(token1, "PULSECLR") == 0) || (strcmp(token1, "PULSERST") == 0))
  {
    g_pulse_reset_origin_request = 1U;
    return;
  }

  if ((strcmp(token1, "CLEARFAULT") == 0) || (strcmp(token1, "CLEAR_FAULT") == 0))
  {
    USBVCP_QueueCommand(CANOPEN_CMD_CLEAR_FAULT, CANOPEN_MODE_IDLE, 0, 0U, 0U);
    return;
  }

  if ((strcmp(token1, "LEAD") == 0) && (token2 != NULL))
  {
    if ((strcmp(token2, "AUTO") == 0) || (strcmp(token2, "auto") == 0))
    {
      g_usb_vcp_manual_lead_enable = 0U;
      return;
    }

    if (USBVCP_ParseInt16(token2, &value1))
    {
      if (value1 < -511)
        value1 = -511;
      else if (value1 > 511)
        value1 = 511;

      g_usb_vcp_manual_lead_angle = value1;
      g_usb_vcp_manual_lead_enable = 1U;
      return;
    }
  }

  if ((strcmp(token1, "UQ") == 0) && (token2 != NULL))
  {
    if ((strcmp(token2, "AUTO") == 0) || (strcmp(token2, "auto") == 0))
    {
      g_usb_vcp_manual_uq_enable = 0U;
      return;
    }

    if (USBVCP_ParseInt16(token2, &value1))
    {
      if (value1 < 0)
        value1 = 0;
      else if (value1 > 1600)
        value1 = 1600;

      g_usb_vcp_manual_uq_centivolt = value1;
      g_usb_vcp_manual_uq_enable = 1U;
      return;
    }
  }

  if ((strcmp(token1, "FWV") == 0) && (token2 != NULL))
  {
    if ((strcmp(token2, "AUTO") == 0) ||
        (strcmp(token2, "auto") == 0) ||
        (strcmp(token2, "OFF") == 0) ||
        (strcmp(token2, "off") == 0))
    {
      g_usb_vcp_fwv_centivolt = 0;
      return;
    }

    if (USBVCP_ParseInt16(token2, &value1))
    {
      if (value1 < 0)
        value1 = 0;
      else if (value1 > 800)
        value1 = 800;

      g_usb_vcp_fwv_centivolt = value1;
      return;
    }
  }

  if (((strcmp(token1, "PI") == 0) || (strcmp(token1, "pi") == 0)) &&
      USBVCP_ParseUint16(token2, &value2) &&
      USBVCP_ParseUint16(token3, &value3))
  {
    if (value2 > 1000U)
      value2 = 1000U;
    if (value3 > 1000U)
      value3 = 1000U;

    g_usb_vcp_speed_kp_milli = value2;
    g_usb_vcp_speed_ki_10000 = value3;
    g_usb_vcp_speed_pi_update_request = 1U;
    return;
  }

  if (((strcmp(token1, "KP") == 0) || (strcmp(token1, "kp") == 0)) &&
      USBVCP_ParseUint16(token2, &value2))
  {
    if (value2 > 1000U)
      value2 = 1000U;

    g_usb_vcp_speed_kp_milli = value2;
    g_usb_vcp_speed_pi_update_request = 1U;
    return;
  }

  if (((strcmp(token1, "KI") == 0) || (strcmp(token1, "ki") == 0)) &&
      USBVCP_ParseUint16(token2, &value2))
  {
    if (value2 > 1000U)
      value2 = 1000U;

    g_usb_vcp_speed_ki_10000 = value2;
    g_usb_vcp_speed_pi_update_request = 1U;
    return;
  }

  if ((strcmp(token1, "ALIGN") == 0) || (strcmp(token1, "align") == 0))
  {
    g_usb_vcp_wave_enable = 0U;
    USBVCP_QueueCommand(CANOPEN_CMD_ALIGN, CANOPEN_MODE_IDLE, 0, 0U, 0U);
    return;
  }

  if (((strcmp(token1, "OPEN") == 0) || (strcmp(token1, "open") == 0)) &&
      USBVCP_ParseInt16(token2, &value1))
  {
    int16_t open_speed_rpm = value1;
    int16_t uq_centivolt = 10;

    if ((token3 != NULL) && USBVCP_ParseInt16(token3, &value1))
    {
      uq_centivolt = value1;
    }

    if (uq_centivolt < 0)
      uq_centivolt = 0;
    if (uq_centivolt > 1600)
      uq_centivolt = 1600;

    g_usb_vcp_manual_uq_centivolt = uq_centivolt;
    g_usb_vcp_manual_uq_enable = 1U;
    g_usb_vcp_wave_enable = 1U;
    USBVCP_QueueCommand(CANOPEN_CMD_START, CANOPEN_MODE_SPEED, open_speed_rpm, 0U, 0U);
    return;
  }

  if ((strcmp(token1, "FEEDSTOP") == 0) || (strcmp(token1, "AUTOSTOP") == 0))
  {
    g_usb_vcp_wave_enable = 0U;
    USBVCP_QueueCommand(CANOPEN_CMD_AUTO_FEED_STOP, CANOPEN_MODE_FEEDER, 0, 0U, 0U);
    return;
  }

  if (((strcmp(token1, "SPEED") == 0) || (strcmp(token1, "speed") == 0)) && USBVCP_ParseInt16(token2, &value1))
  {
    g_usb_vcp_wave_enable = 1U;
    USBVCP_QueueCommand(CANOPEN_CMD_START, CANOPEN_MODE_SPEED, value1, 0U, 0U);
    return;
  }

  if ((strcmp(token1, "POS") == 0) &&
      USBVCP_ParseInt16(token2, &value1) &&
      USBVCP_ParseUint16(token3, &value2) &&
      USBVCP_ParseUint16(token4, &value3))
  {
    g_usb_vcp_wave_enable = 1U;
    USBVCP_QueueCommand(CANOPEN_CMD_START, CANOPEN_MODE_POS_TURNS, value1, value2, value3);
    return;
  }

  if ((strcmp(token1, "TIME") == 0) &&
      USBVCP_ParseInt16(token2, &value1) &&
      USBVCP_ParseUint16(token3, &value2) &&
      USBVCP_ParseUint16(token4, &value3))
  {
    g_usb_vcp_wave_enable = 1U;
    USBVCP_QueueCommand(CANOPEN_CMD_START, CANOPEN_MODE_TIME_TURNS, value1, value2, value3);
    return;
  }

  if (((strcmp(token1, "FEEDAUTO") == 0) || (strcmp(token1, "AUTOSTART") == 0)) &&
      USBVCP_ParseInt16(token2, &value1) &&
      USBVCP_ParseUint16(token3, &value2) &&
      USBVCP_ParseUint16(token4, &value3))
  {
    g_usb_vcp_wave_enable = 1U;
    USBVCP_QueueCommand(CANOPEN_CMD_AUTO_FEED_START, CANOPEN_MODE_FEEDER, value1, value2, value3);
    return;
  }
}

void uart_printf(const char *fmt, ...)
{
  char buffer[160];
  va_list args;
  int len;

  if (fmt == NULL)
  {
    return;
  }

  va_start(args, fmt);
  len = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (len <= 0)
  {
    return;
  }

  if (len > (int)(sizeof(buffer) - 1U))
  {
    len = (int)(sizeof(buffer) - 1U);
  }

  (void)CDC_Transmit_FS((uint8_t *)buffer, (uint16_t)len);
}

int fputc(int ch, FILE *f)
{
  uint8_t buf[1] = {(uint8_t)ch};

  (void)f;
  (void)CDC_Transmit_FS(buf, 1U);
  return ch;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
