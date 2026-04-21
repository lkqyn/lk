/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FOC.h"
#include "adc_task.h"
#include "vofa.h"
#include "PID.h"
#include "connecting.h"
#include "mt6816.h"
#include "myflash.h"
#include "usbd_cdc_if.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
uint32_t add_angle = 0;
float current[2] = {0.0f, 0.0f};
float vol = 0.0f;
float temperature = 0.0f;
uint8_t Iadjust = 1;
uint8_t cchar[4][4] = {0};
float adc_rank_debug[2] = {0.0f, 0.0f};
float adc_cb_debug = 0.0f;
uint16_t iq_step_debug = 0;
extern int16_t Scope[200];
extern uint16_t angledata[200];
extern float speed_send;
uint8_t erro_flag = 0;
uint8_t limit_vol = 8;
static uint32_t motor_protect_current_enable_ms = 0U;

#define CURRENT_LOOP_HZ      10000.0f
#define SPEED_LOOP_DIV       10U
#define SPEED_LOOP_PERIOD    (SPEED_LOOP_DIV / CURRENT_LOOP_HZ)
#define VOFA_DEBUG_DIV       50U
#define FORCE_MT6816_CALIBRATION 0U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void motor_protect(void)
{
  if(temperature > 100 || (vol > 25 || vol < 7))
  {
    connect_crt.motor_mode = 0;
    set_uduq(&m1_foc, 0, 0);
    foc_open(&m1_foc, 0);

//    if(erro_flag == 0)
//    {
//      if(connect_type == 0)
//        CDC_Transmit_FS((uint8_t *)"Protecting!\n", 12);
//      else
//      {
//        set_cantx_buf(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);
//        CAN_Transmit(&my_can_tx);
//      }
//    }
    erro_flag = 1;
  }
  else if(mt6816_flag == 1)
  {
    erro_flag = 0;
  }
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void motor_control_init(void);
static void speed_loop_run(void);
static uint8_t flash_calibration_ready(void);
static void vofa_debug_send(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t flash_calibration_ready(void)
{
  uint16_t dir = flash_read_dir(DIR_AD);
  return (dir == 11U || dir == 22U);
}

static void vofa_debug_send(void)
{
  VOFA_SendFrame6((float)connect_crt.motor_mode,
                  m1_foc.tar_Iq,
                  m1_foc.Iq,
                  m1_foc.Id,
                  (float)m1_foc.angle_sector,
                  (float)m1_foc.angle);
}

static void speed_loop_run(void)
{
  float speed_error;

  if(connect_crt.motor_mode == 1U)
  {
    iq_step_debug++;
    if(iq_step_debug < 1500U)
      set_foc_Iqcurrent(&m1_foc, 0.1f);
    else if(iq_step_debug < 3000U)
      set_foc_Iqcurrent(&m1_foc, 0.5f);
    else
      iq_step_debug = 0U;
    return;
  }

  speed_pi.actual_value = 0.9f * speed_pi.actual_value + 0.1f * Calculate_Speed(SPEED_LOOP_PERIOD);
  speed_send = speed_pi.actual_value;

  if(connect_crt.motor_mode == 2U)
  {
    speed_error = speed_pi.target - speed_pi.actual_value;
    set_foc_Iqcurrent(&m1_foc, PID_Controller_Update(&speed_pi, speed_error));
  }
  else if(connect_crt.motor_mode != 1U)
  {
    speed_pi.integral = 0.0f;
    speed_pi.last_err = 0.0f;
    set_foc_Iqcurrent(&m1_foc, 0.0f);
  }
}

static void motor_control_init(void)
{
  VOFA_Init();
  init_connect_crt(&connect_crt);
  MT6816_SPI_CS_H();
  HAL_Delay(10);

  vol = get_vol();
  temperature = get_temp();

  if(check_mt6816())
  {
    mt6816_flag = 1;
  }
  else
  {
    mt6816_flag = 0;
    Error_Handler();
  }

  can_foc_init();
  if(FORCE_MT6816_CALIBRATION || !flash_calibration_ready())
  {
    REIN_mt6816_spi_data_Signal_Init();
    return;
  }

  calibrate_current_offset(64);
  Iadjust = 0;
  b_foc_init();
  init_encoder_update();
  connect_crt.motor_mode = 1U;
  connect_crt.max_current = 0.5f;
  connect_crt.drive_current = 0.1f;
  m1_foc.tar_Id = 0.0f;
  set_foc_Iqcurrent(&m1_foc, connect_crt.drive_current);
  motor_protect_current_enable_ms = HAL_GetTick() + 1500U;
  HAL_TIM_Base_Start_IT(&htim6);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  motor_control_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    VOFA_Process();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t a_angle = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t speed_loop_count = 0;

    if(htim->Instance == TIM6)
    {
        HAL_ADCEx_InjectedStart_IT(&hadc1);
        speed_loop_count++;
        if(speed_loop_count >= SPEED_LOOP_DIV)
        {
            speed_loop_count = 0;
            speed_loop_run();
            vol = get_vol();
            temperature = get_temp();
            motor_protect();
        }
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    static uint8_t vofa_debug_count = 0;
    uint16_t foc_angle;

    if(hadc == &hadc1)
    {
        adc_rank_debug[0] = (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        adc_rank_debug[1] = (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        adc_cb_debug += 1.0f;
        if(adc_cb_debug > 1000000.0f)
            adc_cb_debug = 0.0f;

        if(encoder_adjust_flag)
            mt6816_count = (uint16_t)REIN_MT6816_GetAngleData();

        if(connect_crt.motor_mode != 0U && Iadjust == 0U)
        {
            Encoder_Update();
            get_AB_current(&m1_foc);
            Sector_tracker();

            foc_angle = (uint16_t)((m1_foc.angle + 256 * (m1_foc.angle_sector % 4) + m1_foc.lead_angle) & 0x03FF);
            current_ctr(&m1_foc, foc_angle);

        }
        else if(Iadjust != 0U)
        {
            adjust_I();
        }

        vofa_debug_count++;
        if(vofa_debug_count >= VOFA_DEBUG_DIV)
        {
            vofa_debug_count = 0;
            vofa_debug_send();
        }

    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
