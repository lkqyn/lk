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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dip_switch.h"
#include "motor_driver.h"
#include "stepper_driver.h"
#include "driver_eg2131_gpio.h"
#include "FOC.h"
#include "adc_task.h"
#include "motion_trap2.h"
#include "encoder.h"
#include "vofa.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VDC_NOMINAL   24.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CTRL_DT             0.0001f   // TIM6 = 10kHz
#define ELEC_CYCLE_PER_REV  50.0f     // 1.8°步进常用：每机械转 50 个电角周期

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
DipSwitch_TypeDef dip_switch;

encoder_t enc;

volatile float g_uq_target_ratio = 0.40f;
volatile float g_test_theta = 0.0f;
volatile uint8_t g_test_run = 1;

// m1_foc 在 FOC.c 里定义，这里直接用 extern
extern foc_TypeDef m1_foc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */
  VOFA_Init();

  // FOC初始化（使用你现在的新FOC）
  FOC_Init();
  FOC_OpenLoopInit(&m1_foc, VDC_NOMINAL, 0.0f, 0.0f);

  // 可按需要修改
  m1_foc.Udc = VDC_NOMINAL;
  m1_foc.outmax = VDC_NOMINAL;
  m1_foc.duty_max = 0.90f;
  m1_foc.deadband = 0.00f;

  //MotionTrap2_Init(&motion);
// 先给一个固定超前角，直接沿用第一份经验
m1_foc.lead_angle = 256;
  Encoder_Init(&enc);
  Encoder_Start(&enc, &htim2);
  Encoder_Zero(&enc, &htim2);
  g_test_theta = 0.0f;
  g_test_run = 1;
  g_uq_target_ratio = 0.40f;  // 改uq
  // 启动控制中断
  HAL_TIM_Base_Start_IT(&htim6);

  // 上电先关输出
  FOC_AllOff();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t last_vofa = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_vofa >= 20)
    {
        last_vofa = now;

        VOFA_SendFrame6(
            g_test_theta,
            m1_foc.Uq,
            g_uq_target_ratio,
            enc.pos_rev,
            enc.vel_rpm_f,
            (float)m1_foc.sector
        );
    }

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

// TIM6中断回调函数，10kHz / 100us
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        Encoder_Update(&enc, &htim2, CTRL_DT);

        m1_foc.Udc = VDC_NOMINAL;
        FOC_OpenLoopSetUqRatio(&m1_foc, g_uq_target_ratio);

        if (g_test_run)
        {
            static float mech_rpm_cmd = 0.0f;
            const float mech_rpm_target = 200.0f;//改速度

            // 最简单升速斜坡
            if (mech_rpm_cmd < mech_rpm_target)
            {
                mech_rpm_cmd += 0.05f;
                if (mech_rpm_cmd > mech_rpm_target)
                    mech_rpm_cmd = mech_rpm_target;
            }

            const float mech_rps = mech_rpm_cmd / 60.0f;
            const float fe_hz = mech_rps * ELEC_CYCLE_PER_REV;
            const float dtheta = 2.0f * (float)M_PI * fe_hz * CTRL_DT;

            g_test_theta += dtheta;
            while (g_test_theta >= 2.0f * (float)M_PI)
            {
                g_test_theta -= 2.0f * (float)M_PI;
            }

            float theta_cmd = g_test_theta
                            + 2.0f * (float)M_PI * ((float)m1_foc.lead_angle / 1024.0f);

            while (theta_cmd >= 2.0f * (float)M_PI)
            {
                theta_cmd -= 2.0f * (float)M_PI;
            }

            FOC_OutputByTheta(&m1_foc, theta_cmd, m1_foc.Uq);
        }
        else
        {
            FOC_AllOff();
        }
    }
}

// 外部中断回调
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ENCODER_Z_Pin)
    {
        g_z_seen++;

        if (!g_z_homed)
        {
            g_z_homed = 1;
            Encoder_Zero(&enc, &htim2);
        }
    }
}
*/
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
