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
#include "dip_switch.h"
#include "FOC.h"
#include "adc_task.h"
#include "motion_trap2.h"
#include "vofa.h"
#include "math.h"
#include "mt6816.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


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


// m1_foc 在 FOC.c 里定义，这里直接用 extern
extern foc_TypeDef m1_foc;
extern uint16_t angledata[200];

extern int16_t Scope[200];

float vol = 24.0f;
uint32_t add_angle;//角度累加，初始化用

volatile float g_theta_cmd = 0.0f;
volatile float g_rpm_ref = 1000.0f;
volatile float g_uq_cmd = 3.0f;   // 先从小一点开始，后面再加

//计算速度
volatile int16_t motor_speed_diff = 0;
volatile float motor_speed_rps = 0.0f;
volatile float motor_speed_rpm = 0.0f;

volatile uint16_t b_step = 0;
volatile float Uq_ref = 0.0f;
volatile float Ud_ref = 0.0f;     // 先沿用你现在能跑起来的工作点
volatile uint8_t speed_div = 0;     // 10kHz / 20 = 500Hz, 对应2ms
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
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  VOFA_Init();

  MT6816_SPI_CS_H();   // 先释放片选
  HAL_Delay(10);
  // 先确认编码器在线
  if (check_mt6816())
  {
    mt6816_flag = 1;
 // 先校准，写Flash，最后自动复位
//        REIN_mt6816_spi_data_Signal_Init();
  }
  else
  {
    mt6816_flag = 0;
    Error_Handler();
  }
  // 正常运行初始化：读取Flash里的DIR和补偿表
  FOC_Init();

  PID_Controller_Init(&speed_pi, 0.004f, 0.00004f, 0.0f, 800.0f, -16.0f, 16.0f);
  // 目标速度，单位 rpm
  speed_pi.target = 800.0f;
  // 初始输出
  Uq_ref = 0.0f;
//  // 初始输出清零
  set_uduq(&m1_foc, 0, 0);
  FOC_AllOff();

//  // 启动控制中断
  HAL_TIM_Base_Start_IT(&htim6);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
static uint32_t last_print = 0;
uint32_t now = HAL_GetTick();

if (now - last_print >= 10)
{
    last_print = now;

    printf("target=%.2f, actual=%.2f, uq=%.2f, enc=%u, angle=%u, step=%u\r\n",
           speed_pi.target,
           speed_pi.actual_value,
           Uq_ref,
           mt6816_count,
           m1_foc.angle,
           b_step);
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
uint16_t a_angle=0;


// TIM6中断回调函数，10kHz / 100us
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        // 1. 读取当前编码器角度
        mt6816_count = (uint16_t)REIN_MT6816_GetAngleData();

        // 2. 每20次中断更新一次速度（10kHz / 20 = 500Hz，周期2ms）
        speed_div++;
        if (speed_div >= 20)
        {
            speed_div = 0;

            Encoder_Update();
            speed_pi.actual_value = Calculate_Speed(0.002f);

            float speed_error = speed_pi.target - speed_pi.actual_value;
            Uq_ref = PID_Controller_Update(&speed_pi, speed_error);
        }

        // 3. 根据编码器位置求当前电角度
        Sector_tracker();

        b_step = (uint16_t)((m1_foc.angle
                           + 256 * (m1_foc.angle_sector % 4)
                           + m1_foc.lead_angle) % 1024);

        // 4. 直接用速度环输出作为Uq，开环输出
        set_uduq(&m1_foc, Ud_ref, Uq_ref);
        foc_open(&m1_foc, b_step);
    }
}
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
     
//}

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
