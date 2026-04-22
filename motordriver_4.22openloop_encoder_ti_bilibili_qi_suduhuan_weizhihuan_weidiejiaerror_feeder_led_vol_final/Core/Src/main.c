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
#include "can.h"
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
#include "pid.h"
#include "connecting.h"
#include "feeder.h"
#include "led_task.h"
#include "can_open.h"
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
// m1_foc 在 FOC.c 里定义，这里直接用 extern
extern foc_TypeDef m1_foc;

volatile float Uq_ref = 0.0f;                // 速度环输出，直接给Uq
volatile float Ud_ref = 0.0f;                // 先固定

volatile uint8_t speed_loop_div = 0;         // 10kHz -> 2kHz 分频作用

//测试指定时间内转指定圈数的变量
uint8_t  feeder_auto_test_enable = 1;   // 1: 自动重复送料测试
uint8_t  feeder_wait_done = 0;          // 当前这次送料是否已经发出
uint32_t feeder_last_tick = 0;          // 上次送料完成时间
uint32_t feeder_interval_ms = 1000;     // 每次送料间隔

float feeder_single_err_turns = 0.0f;   // 单次误差(圈)
float feeder_total_err_turns  = 0.0f;   // 累计误差(圈)

float feeder_last_actual_total_turns = 0.0f;
float feeder_last_expected_total_turns = 0.0f;

float current[2],vol=0,temperature=0 ;//定义ADC物理量

canopen_control_t can_cmd;
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  //CANOpen_Init(&hcan1,1);
  DipSwitch_Read(&dip_switch);
  CANOpen_Init(&hcan1, dip_switch.comm_addr);

  VOFA_Init();
  LED_Task_Init();
  // FOC初始化
  FOC_Init();
  FOC_OpenLoopInit(&m1_foc, VDC_NOMINAL, 0.0f, 0.0f);

  // 参数 可去掉
  m1_foc.Udc = VDC_NOMINAL;
  m1_foc.outmax = 24.0f;
  m1_foc.duty_max = 0.90f;
  m1_foc.deadband = 0.00f;

  // 编码器初始化
  Encoder_Init(&enc);
  Encoder_Start(&enc, &htim2);

  // 当前跑得比较稳的固定偏置
  m1_foc.lead_angle = 200;

  // 上电对齐 吸住转子
  m1_foc.Ud = 4.0f;
  m1_foc.Uq = 0.0f;
  foc_open(&m1_foc, 0);

  // 等待对齐
  HAL_Delay(5000);

  // 编码器清零
  Encoder_Zero(&enc, &htim2);

  // 撤掉对齐电压
  m1_foc.Ud = 0.0f;
  m1_foc.Uq = 0.0f;

  // 初始化 connect 控制参数
  init_connect_crt(&connect_crt);
  //运动曲线
  MotionTrap2_Init(&g_motion_trap2);


  Feeder_Init(&g_feeder,
            &enc,
            20.0f,     // 每次送料20圈
            1.0f,      // 每次送料3秒
            1800.0f,   // 速度上限1800rpm
            0.2f,     // 加速时间占比
            0.2f);    // 减速时间占比

  feeder_last_tick = HAL_GetTick();
  // 测试：圈数，最大速度，加速度
  //MotionTrap2_StartByVA(&g_motion_trap2, &enc, 20.0f, 1500.0f, 5000.0f);

//  int ret = MotionTrap2_StartByTimeWithVmax(&g_motion_trap2,
//                                          &enc,
//                                          20.0f,      // 20圈
//                                          5.0f,       // 5秒
//                                          1500.0f,    // 最高速度上限
//                                          0.25f,      // 加速时间占比
//                                          0.25f);     // 减速时间占比
  // 进入速度环模式
  connect_crt.motor_mode = 2;
  connect_crt.speed = 0.0f;         // 默认静止
  connect_crt.s_acc = 20000.0f;         // 设大，避免破坏motion_trap2轨迹
  connect_crt.drive_current = 16.0f;  // 先作为Uq限幅

  // 更新速度环参数
  speed_pi.target = 0.0f;             // 从0开始斜坡上升
  speed_pi.actual_value = 0.0f;
  speed_pi.output_max = fabsf(connect_crt.drive_current);
  speed_pi.output_min = -fabsf(connect_crt.drive_current);

  Uq_ref = 0.0f;
  Ud_ref = 0.0f;

  // 启动控制中断
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
    uint32_t now = HAL_GetTick();
    static uint32_t last_vofa = 0;

    // -----------------------------------------
    CANOpen_Process();
    // ----------------------------------------
    if (CANOpen_GetCommand(&can_cmd))
    {
      switch (can_cmd.command)
      {
      case CANOPEN_CMD_STOP:
          connect_crt.speed = 0.0f;
          Uq_ref = 0.0f;
          Ud_ref = 0.0f;
          PID_Controller_Reset(&speed_pi);
          break;

      case CANOPEN_CMD_START:
          if (can_cmd.mode == CANOPEN_MODE_SPEED)
          {
              connect_crt.motor_mode = 2;
              connect_crt.speed = (float)can_cmd.target_speed_rpm;
          }
          break;

      case CANOPEN_CMD_ZERO:
          Encoder_Zero(&enc, &htim2);
          Feeder_SetBaseHere(&g_feeder, &enc);
          break;

      case CANOPEN_CMD_FEED_ONCE:
          if (!MotionTrap2_IsBusy(&g_motion_trap2))
          {
              Feeder_Once(&g_feeder, &g_motion_trap2, &enc);
          }
          break;

      default:
          break;
      }
    }
    // -----------------------------------------
    // 自动送料测试：每隔feeder_interval_ms一段时间送料一次
    // -----------------------------------------
    if (feeder_auto_test_enable)
    {
      if ((!MotionTrap2_IsBusy(&g_motion_trap2)) &&
          (!feeder_wait_done) &&
          ((now - feeder_last_tick) >= feeder_interval_ms))
      {
        int ret = Feeder_Once(&g_feeder, &g_motion_trap2, &enc);

        if (ret == MOTION_OK)
        {
          feeder_wait_done = 1;
        }
      }

      if (feeder_wait_done && MotionTrap2_IsDone(&g_motion_trap2))
      {
        float actual_total_turns;
        float expected_total_turns;
        float actual_move_turns;
        float expected_move_turns;

        actual_total_turns =
            (float)(enc.pos_cnt_total - g_feeder.base_cnt) / 4000.0f;

        expected_total_turns =
            (float)g_feeder.feed_index * g_feeder.turns_per_feed;

        actual_move_turns =
            actual_total_turns - feeder_last_actual_total_turns;

        expected_move_turns =
            expected_total_turns - feeder_last_expected_total_turns;

        feeder_single_err_turns =
            actual_move_turns - expected_move_turns;

        feeder_total_err_turns =
            actual_total_turns - expected_total_turns;

        feeder_last_actual_total_turns = actual_total_turns;
        feeder_last_expected_total_turns = expected_total_turns;

        // 清速度环积分残留
        PID_Controller_Reset(&speed_pi);
        Uq_ref = 0.0f;
        Ud_ref = 0.0f;

        feeder_wait_done = 0;
        feeder_last_tick = now;
      }
    }


    // -----------------------------------------
    // VOFA 打印：10ms一帧，防止USB队列塞满
    // CH1: 第几次送料
    // CH2: 单次误差(圈)
    // CH3: 累计误差(圈)
    // CH4: 实际总圈数
    // CH5: 理论总圈数
    // CH6: 当前目标误差(cnt)
    // -----------------------------------------
    if ((now - last_vofa) >= 10)
    {
        float actual_total_turns;
        float expected_total_turns;
        float pos_err_cnt;

        last_vofa = now;

        actual_total_turns =
            (float)(enc.pos_cnt_total - g_feeder.base_cnt) / 4000.0f;

        expected_total_turns =
            (float)g_feeder.feed_index * g_feeder.turns_per_feed;

        pos_err_cnt =
            (float)(g_motion_trap2.target_cnt - enc.pos_cnt_total);

        vol = get_vol();
        temperature = get_temp();

        VOFA_SendFrame6(
            vol,  // CH1
            temperature,     // CH2
            feeder_total_err_turns,      // CH3
            actual_total_turns,          // CH4
            expected_total_turns,        // CH5
            enc.vel_rpm_f                  // CH6
        );
    }

    VOFA_Process();
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
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
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM6)
//    {
//        Encoder_Update(&enc, &htim2, CTRL_DT);
//        Sector_tracker_inc_encoder(&m1_foc, &enc);

//        m1_foc.Uq = 16;
//        m1_foc.Ud = -16;

//        foc_open(&m1_foc, (uint16_t)((m1_foc.angle + m1_foc.lead_angle) % 1024));
//    }
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        // 10kHz：更新编码器和电角
        Encoder_Update(&enc, &htim2, CTRL_DT);
        Sector_tracker_inc_encoder(&m1_foc, &enc);

        // 实际速度
        speed_pi.actual_value = enc.vel_rpm_f;

        // -----------------------------
        // 10kHz 分频成 2kHz 跑速度环
        // -----------------------------
        speed_loop_div++;
        if (speed_loop_div >= 5)
        {
            speed_loop_div = 0;
            // 先更新轨迹规划器
            MotionTrap2_Update(&g_motion_trap2, &enc, enc.vel_rpm_f, 0.0005f);
            if (connect_crt.motor_mode == 2)//速度环模式
            {
                // 2kHz周期 = 0.0005s
                if (speed_pi.target > connect_crt.speed) //0>-1000
                {
                    speed_pi.target -= connect_crt.s_acc * 0.0005f;
                    if (speed_pi.target < connect_crt.speed)
                        speed_pi.target = connect_crt.speed;
                }
                else if (speed_pi.target < connect_crt.speed)
                {
                    speed_pi.target += connect_crt.s_acc * 0.0005f;
                    if (speed_pi.target > connect_crt.speed)
                        speed_pi.target = connect_crt.speed;
                }

                float speed_error = speed_pi.target - speed_pi.actual_value;

                // 速度环输出先直接作为Uq_ref
                Uq_ref = PID_Controller_Update(&speed_pi, speed_error);

                // 先固定Ud
                Ud_ref = 0.0f;
            }
            else
            {
                Uq_ref = 0.0f;
                Ud_ref = 0.0f;
            }
        }

        // -----------------------------
        // 下发到FOC
        // -----------------------------
        m1_foc.Uq = Uq_ref;
        m1_foc.Ud = Ud_ref;

        foc_open(&m1_foc, (uint16_t)((m1_foc.angle + m1_foc.lead_angle) % 1024));

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
