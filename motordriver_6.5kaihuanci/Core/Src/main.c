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
extern int16_t Scope[200];
extern uint16_t angledata[200];
extern float speed_send;
uint8_t erro_flag = 0;
uint8_t limit_vol = 8;
volatile uint8_t pending_motor_mode = 0U;
float speed_error_debug = 0.0f;

#define CURRENT_LOOP_HZ      10000.0f
#define SPEED_LOOP_DIV       5U
#define SPEED_LOOP_PERIOD    (SPEED_LOOP_DIV / CURRENT_LOOP_HZ)
#define SPEED_SAMPLE_DIV     4U
#define SPEED_SAMPLE_PERIOD  (SPEED_SAMPLE_DIV / CURRENT_LOOP_HZ)
#define FORCE_MT6816_CALIBRATION 0U
#define SPEED_UQ_LIMIT       12.0f
#define SPEED_ERROR_LIMIT    200.0f
#define UQ_STEP_PER_LOOP     0.01f
#define MOTOR_OVERVOLTAGE_LIMIT 30.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void motor_protect(void)
{
  if(temperature > 100 || (vol > MOTOR_OVERVOLTAGE_LIMIT || vol < 7))
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
static void vofa_idle_debug_send(void);
static void open_loop_test_run(void);

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
  static float vofa_tick = 0.0f;

  vofa_tick += 1.0f;
  if(vofa_tick > 100000.0f)
    vofa_tick = 0.0f;

  VOFA_SendFrame6(vofa_tick,
                  speed_pi.actual_value,
                  m1_foc.Uq,
                  (float)mt6816_count,
                  speed_error_debug,
                  vol);
}

static void vofa_idle_debug_send(void)
{
  vofa_debug_send();
}

static void speed_loop_run(void)
{
  float speed_error;

  speed_send = speed_pi.actual_value;

  if(connect_crt.motor_mode == 1U)
  {
    float uq_cmd = m1_foc.Uq;
    float uq_target = connect_crt.drive_current;
    if(uq_target > SPEED_UQ_LIMIT) uq_target = SPEED_UQ_LIMIT;
    else if(uq_target < -SPEED_UQ_LIMIT) uq_target = -SPEED_UQ_LIMIT;

    if(uq_cmd < uq_target)
    {
      uq_cmd += UQ_STEP_PER_LOOP;
      if(uq_cmd > uq_target) uq_cmd = uq_target;
    }
    else if(uq_cmd > uq_target)
    {
      uq_cmd -= UQ_STEP_PER_LOOP;
      if(uq_cmd < uq_target) uq_cmd = uq_target;
    }

    speed_pi.integral = 0.0f;
    speed_pi.last_err = 0.0f;
    set_uduq(&m1_foc, 0.0f, uq_cmd);
    return;
  }

  if(connect_crt.motor_mode == 2U && fabsf(speed_pi.actual_value) > 3000.0f)
  {
    connect_crt.motor_mode = 0U;
    speed_pi.integral = 0.0f;
    speed_pi.last_err = 0.0f;
    set_uduq(&m1_foc, 0.0f, 0.0f);
    FOC_AllOff();
    return;
  }

  if(connect_crt.motor_mode == 2U)
  {
    if(speed_pi.target != connect_crt.speed)
    {
      if(speed_pi.target > connect_crt.speed)
      {
        speed_pi.target -= connect_crt.s_acc * SPEED_LOOP_PERIOD;
        if(speed_pi.target < connect_crt.speed)
          speed_pi.target = connect_crt.speed;
      }
      else
      {
        speed_pi.target += connect_crt.s_acc * SPEED_LOOP_PERIOD;
        if(speed_pi.target > connect_crt.speed)
          speed_pi.target = connect_crt.speed;
      }
    }

    speed_error = speed_pi.target - speed_pi.actual_value;
    if(speed_error > SPEED_ERROR_LIMIT) speed_error = SPEED_ERROR_LIMIT;
    else if(speed_error < -SPEED_ERROR_LIMIT) speed_error = -SPEED_ERROR_LIMIT;
    speed_error_debug = speed_error;

    float uq_cmd = PID_Controller_Update(&speed_pi, speed_error);
    set_uduq(&m1_foc, 0.0f, uq_cmd);
  }
  else if(connect_crt.motor_mode != 1U && connect_crt.motor_mode != 3U)
  {
    speed_pi.integral = 0.0f;
    speed_pi.last_err = 0.0f;
    set_uduq(&m1_foc, 0.0f, 0.0f);
    FOC_AllOff();
  }
}

static void open_loop_test_run(void)
{
  return;
}

static void motor_control_init(void)
{
  VOFA_Init();
  init_connect_crt(&connect_crt);
  MT6816_SPI_CS_H();
  HAL_Delay(300);

  vol = get_vol();
  temperature = get_temp();

  if(check_mt6816() || check_mt6816())
  {
    mt6816_flag = 1;
  }
  else
  {
    mt6816_flag = 0;
    erro_flag = 1;
  }

  can_foc_init();
  calibrate_current_offset(100);
  Iadjust = 0;

  if(FORCE_MT6816_CALIBRATION || !flash_calibration_ready())
  {
    REIN_mt6816_spi_data_Signal_Init();
    return;
  }

  if(mt6816_flag == 0U)
  {
    connect_crt.motor_mode = 0U;
    set_uduq(&m1_foc, 0.0f, 0.0f);
    FOC_AllOff();
    return;
  }

  b_foc_init();
  init_encoder_update();
  connect_crt.motor_mode = 0U;
  connect_crt.max_current = 0.8f;
  connect_crt.drive_current = 0.0f;
  connect_crt.speed = 0.0f;
  connect_crt.s_acc = 1000.0f;
  speed_pi.kp = 0.004f;
  speed_pi.ki = 0.00004f;
  speed_pi.kd = 0.0f;
  speed_pi.target = 0.0f;
  speed_pi.actual_value = 0.0f;
  speed_pi.integral = 0.0f;
  speed_pi.last_err = 0.0f;
  speed_pi.output_min = -SPEED_UQ_LIMIT;
  speed_pi.output_max = SPEED_UQ_LIMIT;
  m1_foc.tar_Id = 0.0f;
  set_uduq(&m1_foc, 0.0f, 0.0f);
  FOC_AllOff();
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
    static uint32_t idle_debug_tick = 0U;
    static uint32_t adc_slow_tick = 0U;

    if(pending_motor_mode != 0U)
    {
      uint8_t mode = pending_motor_mode;
      pending_motor_mode = 0U;

      speed_pi.integral = 0.0f;
      speed_pi.last_err = 0.0f;
      speed_pi.target = 0.0f;
      set_uduq(&m1_foc, 0.0f, 0.0f);
      init_encoder_update();
      if(mt6816_flag == 1U && erro_flag == 0U)
        connect_crt.motor_mode = mode;
    }

    if(HAL_GetTick() - idle_debug_tick >= 50U)
    {
      idle_debug_tick = HAL_GetTick();

      if(HAL_GetTick() - adc_slow_tick >= 1000U)
      {
        adc_slow_tick = HAL_GetTick();
        vol = get_vol();
        temperature = get_temp();
        motor_protect();
      }

      if(mt6816_flag == 0U)
      {
        vofa_idle_debug_send();
      }
      else
      {
        vofa_debug_send();
      }
    }

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
uint8_t jj = 0;
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
            open_loop_test_run();
            speed_loop_run();
        }
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint16_t foc_angle;

    if(hadc == &hadc1)
    {
        if(encoder_adjust_flag)
            mt6816_count = (uint16_t)REIN_MT6816_GetAngleData();

        if(connect_crt.motor_mode != 0U && connect_crt.motor_mode != 3U && Iadjust == 0U)
        {
            Encoder_Update();
            jj++;
            if(jj >= SPEED_SAMPLE_DIV)
            {
                jj = 0;
                speed_pi.actual_value = 0.9f * speed_pi.actual_value + 0.1f * Calculate_Speed(SPEED_SAMPLE_PERIOD);
            }

            Sector_tracker();

            foc_angle = (uint16_t)((m1_foc.angle + 256 * (m1_foc.angle_sector % 4) + m1_foc.lead_angle) & 0x03FF);
            foc_open(&m1_foc, foc_angle);

        }
        else if(connect_crt.motor_mode == 0U)
        {
            adjust_I();
            mt6816_count = (uint16_t)REIN_MT6816_GetAngleData();
            Encoder_Update();
            jj++;
            if(jj >= SPEED_SAMPLE_DIV)
            {
                jj = 0;
                speed_pi.actual_value = 0.9f * speed_pi.actual_value + 0.1f * Calculate_Speed(SPEED_SAMPLE_PERIOD);
            }
        }
        else if(Iadjust != 0U)
        {
            adjust_I();
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
