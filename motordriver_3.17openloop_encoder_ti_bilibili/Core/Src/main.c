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

#define UQ_RUN              0.50f
#define UQ_HOLD             0.08f

#define K_POS_HOLD          (2.0f * (float)M_PI * 0.30f)

#define ERR_LIM_HOLD        0.03f     // 停止收尾时的误差限幅 rev
#define ERR_DONE_REV        0.003f    // 到位阈值 rev，大约1.08°
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
DipSwitch_TypeDef dip_switch;
motion_trap2_t motion;
encoder_t enc;

volatile float g_uq_target_ratio = 0.12f;
volatile float g_theta_offset = 0.0f;
volatile uint32_t g_z_seen = 0;
volatile uint8_t  g_z_homed = 0;
volatile uint8_t  g_settle_ok_cnt = 0;

// VOFA按钮命令
volatile uint8_t g_cmd_start = 0;
volatile uint8_t g_cmd_stop  = 0;
volatile uint8_t g_cmd_zero  = 0;

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

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  FOC_SetDefaultParams(&m1_foc);
  FOC_OpenLoopInit(&m1_foc, VDC_NOMINAL, 0.0f, 0.0f);

  // 可按需要修改
  m1_foc.Udc = VDC_NOMINAL;
  m1_foc.outmax = VDC_NOMINAL;
  m1_foc.duty_max = 0.50f;
  m1_foc.deadband = 0.00f;

  MotionTrap2_Init(&motion);

  Encoder_Init(&enc);
  Encoder_Start(&enc, &htim2);
  Encoder_Zero(&enc, &htim2);

  // 启动控制中断
  HAL_TIM_Base_Start_IT(&htim6);

  // 上电先关输出
  FOC_AllOff();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (g_cmd_zero)
    {
        g_cmd_zero = 0;
        Encoder_Zero(&enc, &htim2);
        MotionTrap2_Init(&motion);
        FOC_AllOff();
    }

    if (g_cmd_stop)
    {
        g_cmd_stop = 0;
        MotionTrap2_Stop(&motion);
        FOC_AllOff();
    }

    if (g_cmd_start)
    {
        g_cmd_start = 0;

        MotionTrap2_Init(&motion);
        MotionTrap2_StartMoveAbs(&motion,
                                 enc.pos_rev,
                                 enc.pos_rev + 40.0f,
                                 4.0f,
                                 1.5f,
                                 1.5f);

        // 若要连续转，可换成：
        // MotionTrap2_StartRunForever(&motion, enc.pos_rev, 3.0f, 1.5f);
    }

    // VOFA发送
    static uint32_t last_vofa = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_vofa >= 20)
    {
        last_vofa = now;

        VOFA_SendFrame6(
            motion.pos,
            enc.pos_rev,
            motion.vel * 60.0f,
            enc.vel_rpm_f,
            motion.target_pos - enc.pos_rev,
            g_uq_target_ratio
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
        m1_foc.Udc = VDC_NOMINAL;

        // 1) 编码器更新
        Encoder_Update(&enc, &htim2, CTRL_DT);

        // 2) 梯形轨迹更新
        MotionTrap2_Update(&motion, CTRL_DT);

        // 3) 轨迹前馈角度
        float theta_ff = 2.0f * (float)M_PI * (motion.pos * ELEC_CYCLE_PER_REV) + g_theta_offset;

        // 4) 目标停止角
        float theta_target = 2.0f * (float)M_PI * (motion.target_pos * ELEC_CYCLE_PER_REV) + g_theta_offset;

        // 5) 停止后的收尾误差
        float pos_err_hold = motion.target_pos - enc.pos_rev;
        float err_abs = (pos_err_hold >= 0.0f) ? pos_err_hold : -pos_err_hold;

        if (pos_err_hold > ERR_LIM_HOLD)  pos_err_hold = ERR_LIM_HOLD;
        if (pos_err_hold < -ERR_LIM_HOLD) pos_err_hold = -ERR_LIM_HOLD;

        if (motion.busy)
        {
            // 运行阶段：纯前馈
            g_uq_target_ratio = UQ_RUN;
            FOC_OpenLoopRampUqRatio(&m1_foc, g_uq_target_ratio, 0.002f);
            FOC_OutputByTheta(&m1_foc, theta_ff, m1_foc.Uq);

            g_settle_ok_cnt = 0;
        }
        else
        {
            // 停止阶段：收尾
            g_uq_target_ratio = UQ_HOLD;
            FOC_OpenLoopRampUqRatio(&m1_foc, g_uq_target_ratio, 0.002f);

            float theta_cmd = theta_target + K_POS_HOLD * pos_err_hold;

            if (err_abs < ERR_DONE_REV)
            {
                if (g_settle_ok_cnt < 65535) {
                    g_settle_ok_cnt++;
                }
            }
            else
            {
                g_settle_ok_cnt = 0;
            }

            // 到位后关输出
            if (g_settle_ok_cnt >= 50)   // 50个周期 = 5ms
            {
                FOC_AllOff();
            }
            else
            {
                FOC_OutputByTheta(&m1_foc, theta_cmd, m1_foc.Uq);
            }

            // 若你想停住后持续保力矩，把上面if整段替换成：
            // FOC_OutputByTheta(&m1_foc, theta_cmd, m1_foc.Uq);
        }
    }
}

// 外部中断回调
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