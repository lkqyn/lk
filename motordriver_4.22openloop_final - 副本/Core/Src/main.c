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
#include "usbd_cdc_if.h"
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
typedef enum
{
    LOCAL_MODE_IDLE = 0,
    LOCAL_MODE_SPEED,
    LOCAL_MODE_POS_TURNS,
    LOCAL_MODE_TIME_TURNS,
    LOCAL_MODE_AUTO_FEEDER
} local_run_mode_t;

typedef enum
{
    VOFA_VIEW_SPEED = 0,
    VOFA_VIEW_POSITION,
    VOFA_VIEW_FEEDER,
    VOFA_VIEW_PULSE,
    VOFA_VIEW_PULSEDBG
} vofa_view_t;

typedef enum
{
    CONTROL_SOURCE_NONE = 0,
    CONTROL_SOURCE_CANOPEN,
    CONTROL_SOURCE_VOFA,
    CONTROL_SOURCE_PULSE
} control_source_t;

typedef struct
{
    float rpm_min;
    float rpm_max;
    int16_t lead_angle;
} motor_lead_range_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VDC_NOMINAL   24.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CTRL_DT             0.00005f
#define ELEC_CYCLE_PER_REV  50.0f
#define OPEN_LOOP_ANGLE_STEP_PER_RPM  ((ELEC_CYCLE_PER_REV * 1024.0f * CTRL_DT) / 60.0f)
#define OPEN_LOOP_MIN_UQ            0.3f

#define LOCAL_RUN_MODE           LOCAL_MODE_IDLE

#define LOCAL_SPEED_RPM          40.0f
#define LOCAL_MOVE_TURNS         20.0f
#define LOCAL_MOVE_VMAX_RPM      1000.0f
#define LOCAL_MOVE_ACC_RPM_S     5000.0f
#define LOCAL_MOVE_TIME_S        3.0f
#define LOCAL_MOVE_ACC_RATIO     0.20f
#define LOCAL_MOVE_DEC_RATIO     0.20f

#define LOCAL_FEED_TURNS         20.0f
#define LOCAL_FEED_TIME_S        2.0f
#define LOCAL_FEED_VMAX_RPM      1300.0f
#define LOCAL_FEED_ACC_RATIO     0.20f
#define LOCAL_FEED_DEC_RATIO     0.20f
#define LOCAL_FEED_INTERVAL_MS   1000u

#define SPEED_UQ_FF_GAIN_V_PER_RPM   0.0085f
#define SPEED_LOOP_DT                (CTRL_DT * 10.0f)
#define SPEED_UQ_SLEW_V_PER_S        200.0f

#define MOTOR_PROTECT_PERIOD_MS      20U
#define MOTOR_UNDERVOLTAGE_V         20.0f
#define MOTOR_OVERVOLTAGE_V          80.0f
#define MOTOR_OVERTEMP_C             100.0f
#define MOTOR_ALIGN_VOLTAGE          4.0f
#define MOTOR_ALIGN_TIME_MS          2000U
#define MOTOR_LEAD_ANGLE_FORWARD     0U
#define MOTOR_LEAD_ANGLE_REVERSE     0U

#define MOTOR_FAULT_NONE             0U
#define MOTOR_FAULT_UNDERVOLTAGE     1U
#define MOTOR_FAULT_OVERVOLTAGE      2U
#define MOTOR_FAULT_OVERTEMP         3U
#define MOTOR_FAULT_TEMP_SENSOR      4U
#define MOTOR_FAULT_BRAKE_INPUT      5U

#define MOTOR_FAULT_SIM_MODE         0U
#define MOTOR_RESULT_FAULT           (-20)
#define MOTOR_RESULT_BRAKE           (-30)
#define MOTOR_BRAKE_SIM_ENABLE       0U

#define PULSE_TRACK_MAX_RPM           1200.0f
#define PULSE_SPEED_WINDOW_S          0.030f
#define PULSE_STOP_TIMEOUT_S          0.080f
#define PULSE_SPEED_ALPHA             0.20f
#define PULSE_TARGET_SIGN             (1.0f)
#define PULSE_HW_AUTO_ENABLE          0U
#define PULSE_OPEN_LOOP_UQ            0.3f
#define USB_ASCII_DEBUG_ENABLE       0U
#define VOFA_DIRECT_DEBUG_ENABLE     0U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
DipSwitch_TypeDef dip_switch;
encoder_t enc;
extern foc_TypeDef m1_foc;

volatile float Uq_ref = 0.0f;
volatile float Ud_ref = 0.0f;
volatile uint8_t speed_loop_div = 0;
uint8_t feeder_auto_test_enable = 0;

uint8_t  feeder_wait_done = 0;
uint32_t feeder_last_tick = 0;
uint32_t feeder_interval_ms = 1000;

float feeder_single_err_turns = 0.0f;
float feeder_total_err_turns  = 0.0f;
float feeder_last_actual_total_turns = 0.0f;
float feeder_last_expected_total_turns = 0.0f;

float current[2], vol = 0, temperature = 0;
canopen_control_t can_cmd;
volatile int16_t canopen_last_motion_ret = 0;
volatile uint8_t motor_fault_code = MOTOR_FAULT_NONE;
volatile uint8_t motor_aligning = 0;
static uint32_t motor_align_start_tick = 0;
static int8_t motor_dir_sign = 1;
static CtrlMode_TypeDef g_ctrl_mode = CTRL_MODE_CLOSED_LOOP;
static volatile float motor_open_loop_angle = 0.0f;
static volatile uint8_t g_open_loop_debug_sector = 0U;
static volatile uint32_t g_open_loop_debug_update_count = 0U;
static volatile float g_open_loop_debug_step = 0.0f;
static vofa_view_t g_vofa_view = VOFA_VIEW_SPEED;
volatile uint8_t g_pulse_sim_enable = 0U;
volatile uint8_t g_pulse_sim_dir_positive = 1U;
volatile uint8_t g_pulse_sim_en_level = 1U;
volatile int32_t g_pulse_count = 0;
volatile uint16_t g_pulse_microstep_value = 1U;
volatile uint32_t g_pulse_per_rev = 200U;
volatile float g_pulse_target_turns = 0.0f;
volatile uint8_t g_pulse_reset_origin_request = 0U;
static uint32_t g_pulse_sim_last_tick = 0U;
static uint16_t g_pulse_hw_last_count = 0U;
static float g_pulse_origin_turns = 0.0f;
static uint8_t g_pulse_follow_active = 0U;
static control_source_t g_control_source = CONTROL_SOURCE_NONE;
static int32_t g_pulse_follow_last_count = 0;
static int32_t g_pulse_speed_accum_count = 0;
static float g_pulse_speed_window_s = 0.0f;
static float g_pulse_idle_s = 0.0f;
static float g_pulse_speed_ff_rpm = 0.0f;
static float g_pulse_follow_desired_turns = 0.0f;
static float g_pulse_follow_pos_err_turns = 0.0f;
static float g_pulse_follow_speed_cmd_rpm = 0.0f;
static uint8_t g_pulse_manual_holdoff = 0U;
static int32_t g_pulse_open_loop_last_count = 0;
static float g_pulse_open_loop_idle_s = 0.0f;
static int64_t g_relative_motion_expected_cnt = 0;
static uint8_t g_relative_motion_expected_valid = 0U;
static int64_t g_canopen_position_base_cnt = 0;
static int64_t g_canopen_position_command_pulses = 0;
static uint8_t g_canopen_position_base_valid = 0U;

static const motor_lead_range_t motor_lead_range_table[] =
{
    {   0.0f,  50.0f,   4 },
    {  50.0f,  70.0f,  10 },
    {  70.0f, 100.0f,  30 },
    { 100.0f, 130.0f,  60 },
    { 130.0f, 160.0f,  90 },
    { 160.0f, 190.0f, 120 },
    { 190.0f, 220.0f, 150 },
    { 220.0f, 250.0f, 180 },
    { 250.0f, 700.0f, 200 },
    { 700.0f, 800.0f, 230 },
    { 800.0f, 820.0f, 210 },
    { 820.0f, 840.0f, 220 },
    { 840.0f, 860.0f, 230 },
    { 860.0f, 900.0f, 240 },
    { 900.0f, 10000.0f, 200 }
};

#define MOTOR_LEAD_RANGE_TABLE_SIZE  ((uint32_t)(sizeof(motor_lead_range_table) / sizeof(motor_lead_range_table[0])))
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void LocalMode_Start(local_run_mode_t mode);
static void Motor_StopAll(void);
static void Motor_PrepareMotionStart(void);
static void Motor_ApplyDipSwitchConfig(const DipSwitch_TypeDef *sw);
static void Motor_ApplyPulseDirection(uint8_t dir_positive);
static float Motor_ApplyDirectionFloat(float value);
static float Motor_SlewFloat(float current, float target, float max_step);
static uint8_t Motor_IsClosedLoopMode(void);
static uint16_t Motor_GetLeadAngleBySpeed(float speed_rpm);
static uint8_t Motor_EvaluateFault(void);
static uint8_t Motor_IsBrakeInputActive(void);
static int16_t Motor_GetFaultResult(void);
static void Motor_StartAlign(uint32_t now_ms);
static void Motor_AlignTask(uint32_t now_ms);
static void Motor_ProtectionTask(uint32_t now_ms);
static void Motor_BrakeTask(uint32_t now_ms);
static void Pulse_UpdateConfigFromDipSwitch(const DipSwitch_TypeDef *sw);
static void Pulse_SimTask(uint32_t now_ms);
static void Pulse_InputTask(void);
static void Pulse_ResetOriginToCurrentPosition(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Motor_StopAll(void)
{
    MotionTrap2_Abort(&g_motion_trap2);
    feeder_auto_test_enable = 0;
    feeder_wait_done = 0;
    motor_aligning = 0;
    g_pulse_follow_active = 0U;
    g_control_source = CONTROL_SOURCE_NONE;
    connect_crt.motor_mode = 0;
    connect_crt.speed = 0.0f;
    speed_pi.target = 0.0f;
    PID_Controller_Reset(&speed_pi);
    Uq_ref = 0.0f;
    Ud_ref = 0.0f;
    motor_open_loop_angle = 0.0f;
    g_vofa_view = VOFA_VIEW_SPEED;
}

static void Motor_PrepareMotionStart(void)
{
    MotionTrap2_Abort(&g_motion_trap2);
    feeder_auto_test_enable = 0;
    feeder_wait_done = 0;
    motor_aligning = 0;
    g_pulse_follow_active = 0U;
    connect_crt.motor_mode = 0;
    connect_crt.speed = 0.0f;
    speed_pi.target = 0.0f;
    PID_Controller_Reset(&speed_pi);
    Uq_ref = 0.0f;
    Ud_ref = 0.0f;
    motor_open_loop_angle = 0.0f;
}

static void Motor_ApplyDipSwitchConfig(const DipSwitch_TypeDef *sw)
{
    g_ctrl_mode = (sw != 0) ? sw->ctrl_mode : CTRL_MODE_CLOSED_LOOP;
    Pulse_UpdateConfigFromDipSwitch(sw);

    if ((sw != 0) && (sw->direction == DIR_REVERSE))
    {
        motor_dir_sign = 1;
        m1_foc.lead_angle = MOTOR_LEAD_ANGLE_FORWARD;
    }
    else
    {
        motor_dir_sign = -1;
        m1_foc.lead_angle = MOTOR_LEAD_ANGLE_REVERSE;
    }
}

static void Motor_ApplyPulseDirection(uint8_t dir_positive)
{
    if (dir_positive)
    {
        motor_dir_sign = -1;
        m1_foc.lead_angle = MOTOR_LEAD_ANGLE_REVERSE;
    }
    else
    {
        motor_dir_sign = 1;
        m1_foc.lead_angle = MOTOR_LEAD_ANGLE_FORWARD;
    }
}

static float Motor_ApplyDirectionFloat(float value)
{
    return value * (float)motor_dir_sign;
}

static int64_t Motor_CommandPulsesToCountOffset(int64_t command_pulses)
{
    int64_t numerator;
    int64_t denominator;

    if (g_canopen_od.motor_resolution == 0U)
    {
        return 0;
    }

    numerator = command_pulses * 4000LL;
    denominator = (int64_t)g_canopen_od.motor_resolution;

    if (numerator >= 0)
    {
        return (numerator + (denominator / 2)) / denominator;
    }

    return (numerator - (denominator / 2)) / denominator;
}

static float Motor_SlewFloat(float current, float target, float max_step)
{
    float delta = target - current;

    if (delta > max_step)
        return current + max_step;

    if (delta < -max_step)
        return current - max_step;

    return target;
}

static uint16_t Motor_GetLeadAngleBySpeed(float speed_rpm)
{
    float rpm_abs = fabsf(speed_rpm);
    int16_t lead;
    uint32_t range_i;

    lead = motor_lead_range_table[MOTOR_LEAD_RANGE_TABLE_SIZE - 1U].lead_angle;

    for (range_i = 0U; range_i < MOTOR_LEAD_RANGE_TABLE_SIZE; range_i++)
    {
        const motor_lead_range_t *range = &motor_lead_range_table[range_i];

        if ((rpm_abs >= range->rpm_min) && (rpm_abs < range->rpm_max))
        {
            lead = range->lead_angle;
            break;
        }
    }

    if (motor_dir_sign < 0)
        lead = (int16_t)(-lead);

    while (lead < 0)
        lead = (int16_t)(lead + 1024);
    while (lead >= 1024)
        lead = (int16_t)(lead - 1024);

    return (uint16_t)lead;
}

static uint8_t Motor_IsClosedLoopMode(void)
{
    return (g_ctrl_mode == CTRL_MODE_CLOSED_LOOP) ? 1U : 0U;
}

static void Pulse_UpdateConfigFromDipSwitch(const DipSwitch_TypeDef *sw)
{
    uint16_t microstep_value = 1U;

    if (sw != 0)
    {
        microstep_value = (uint16_t)(1U << ((uint8_t)sw->microstep & 0x07U));
    }

    g_pulse_microstep_value = microstep_value;
    g_pulse_per_rev = (uint32_t)200U * (uint32_t)microstep_value;
    CANOpen_SetMotorResolution(g_pulse_per_rev);
    g_pulse_target_turns = PULSE_TARGET_SIGN * ((float)g_pulse_count / (float)g_pulse_per_rev);
}

static void Pulse_ResetOriginToCurrentPosition(void)
{
    g_pulse_origin_turns = enc.pos_rev;
    g_pulse_count = 0;
    g_pulse_target_turns = 0.0f;
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    g_pulse_hw_last_count = 0U;
    g_pulse_reset_origin_request = 0U;
    g_pulse_manual_holdoff = 0U;
    g_pulse_follow_last_count = 0;
    g_pulse_speed_accum_count = 0;
    g_pulse_speed_window_s = 0.0f;
    g_pulse_idle_s = 0.0f;
    g_pulse_speed_ff_rpm = 0.0f;
    g_pulse_follow_desired_turns = g_pulse_origin_turns;
    g_pulse_follow_pos_err_turns = 0.0f;
    g_pulse_follow_speed_cmd_rpm = 0.0f;
    g_pulse_open_loop_last_count = 0;
    g_pulse_open_loop_idle_s = 0.0f;
}

static void Pulse_SimTask(uint32_t now_ms)
{
    if (!g_pulse_sim_enable)
    {
        g_pulse_target_turns = PULSE_TARGET_SIGN * ((float)g_pulse_count / (float)g_pulse_per_rev);
        return;
    }

    if (!g_pulse_sim_en_level)
    {
        g_pulse_target_turns = PULSE_TARGET_SIGN * ((float)g_pulse_count / (float)g_pulse_per_rev);
        return;
    }

    if ((now_ms - g_pulse_sim_last_tick) >= 20U)
    {
        g_pulse_sim_last_tick = now_ms;
        if (g_pulse_sim_dir_positive)
        {
            g_pulse_count++;
        }
        else
        {
            g_pulse_count--;
        }
    }

    g_pulse_target_turns = PULSE_TARGET_SIGN * ((float)g_pulse_count / (float)g_pulse_per_rev);
}

static void Pulse_InputTask(void)
{
    static uint8_t pulse_en_last = 0U;

#if (PULSE_HW_AUTO_ENABLE == 0U)
    (void)pulse_en_last;
    if (g_pulse_reset_origin_request)
    {
        Pulse_ResetOriginToCurrentPosition();
    }
    return;
#else
    if (g_pulse_reset_origin_request)
    {
        Pulse_ResetOriginToCurrentPosition();
    }

    if (g_pulse_sim_enable)
    {
        return;
    }

    g_pulse_sim_dir_positive =
        (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET) ? 1U : 0U;
    g_pulse_sim_en_level =
        (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET) ? 1U : 0U;
    {
        uint16_t pulse_hw_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        int16_t pulse_hw_delta = (int16_t)(pulse_hw_now - g_pulse_hw_last_count);
        g_pulse_hw_last_count = pulse_hw_now;

        if (g_pulse_sim_en_level && (pulse_hw_delta != 0))
        {
            if (g_pulse_sim_dir_positive)
                g_pulse_count += (int32_t)pulse_hw_delta;
            else
                g_pulse_count -= (int32_t)pulse_hw_delta;
        }
    }
    g_pulse_target_turns = PULSE_TARGET_SIGN * ((float)g_pulse_count / (float)g_pulse_per_rev);

    if (g_control_source == CONTROL_SOURCE_PULSE)
    {
        Motor_ApplyPulseDirection(g_pulse_sim_dir_positive);
    }

    if ((g_control_source == CONTROL_SOURCE_PULSE) || g_pulse_follow_active)
    {
        g_vofa_view = VOFA_VIEW_PULSE;
    }

    if (g_pulse_sim_en_level)
    {
        if (!g_pulse_manual_holdoff)
        {
            if ((!pulse_en_last) ||
                ((g_control_source != CONTROL_SOURCE_PULSE) && !g_pulse_follow_active))
            {
                MotionTrap2_Abort(&g_motion_trap2);
                Pulse_ResetOriginToCurrentPosition();
                g_pulse_follow_active = 1U;
                g_control_source = CONTROL_SOURCE_PULSE;
                g_pulse_follow_last_count = g_pulse_count;
                g_pulse_open_loop_last_count = g_pulse_count;
                g_pulse_open_loop_idle_s = 0.0f;
                g_pulse_follow_pos_err_turns = 0.0f;
                g_pulse_follow_speed_cmd_rpm = 0.0f;
                Motor_ApplyDipSwitchConfig(&dip_switch);
            }
        }
        else if ((g_control_source == CONTROL_SOURCE_PULSE) || g_pulse_follow_active)
        {
            MotionTrap2_Abort(&g_motion_trap2);
            g_pulse_follow_active = 0U;
            g_control_source = CONTROL_SOURCE_NONE;
            connect_crt.motor_mode = 0;
            connect_crt.speed = 0.0f;
            speed_pi.target = 0.0f;
            PID_Controller_Reset(&speed_pi);
            Uq_ref = 0.0f;
            Ud_ref = 0.0f;
            g_pulse_follow_last_count = g_pulse_count;
            g_pulse_open_loop_last_count = g_pulse_count;
            g_pulse_open_loop_idle_s = 0.0f;
            g_pulse_follow_pos_err_turns = 0.0f;
            g_pulse_follow_speed_cmd_rpm = 0.0f;
            g_pulse_speed_accum_count = 0;
            g_pulse_speed_window_s = 0.0f;
            g_pulse_idle_s = 0.0f;
            g_pulse_speed_ff_rpm = 0.0f;
            Motor_ApplyDipSwitchConfig(&dip_switch);
        }
    }
    else
    {
        g_pulse_manual_holdoff = 0U;
        Motor_ApplyDipSwitchConfig(&dip_switch);
        if (g_control_source == CONTROL_SOURCE_PULSE)
        {
            g_pulse_follow_active = 0U;
            connect_crt.motor_mode = 0;
            connect_crt.speed = 0.0f;
            speed_pi.target = 0.0f;
            PID_Controller_Reset(&speed_pi);
            Uq_ref = 0.0f;
            Ud_ref = 0.0f;
            g_control_source = CONTROL_SOURCE_NONE;
            g_pulse_follow_last_count = g_pulse_count;
            g_pulse_open_loop_last_count = g_pulse_count;
            g_pulse_open_loop_idle_s = 0.0f;
            g_pulse_follow_pos_err_turns = 0.0f;
            g_pulse_follow_speed_cmd_rpm = 0.0f;
            g_pulse_speed_accum_count = 0;
            g_pulse_speed_window_s = 0.0f;
            g_pulse_idle_s = 0.0f;
            g_pulse_speed_ff_rpm = 0.0f;
        }
    }

    pulse_en_last = g_pulse_sim_en_level;
#endif
}

static uint8_t Motor_EvaluateFault(void)
{
    if (temperature < -100.0f)
        return MOTOR_FAULT_TEMP_SENSOR;
    else if (temperature > MOTOR_OVERTEMP_C)
        return MOTOR_FAULT_OVERTEMP;
    else if (vol > MOTOR_OVERVOLTAGE_V)
        return MOTOR_FAULT_OVERVOLTAGE;
    /* Undervoltage protection is temporarily disabled for open-loop bring-up. */

    return MOTOR_FAULT_NONE;
}

static uint8_t Motor_IsBrakeInputActive(void)
{
#if (MOTOR_BRAKE_SIM_ENABLE != 0U)
    return 1U;
#else
    return ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) ||
            (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)) ? 1U : 0U;
#endif
}

static int16_t Motor_GetFaultResult(void)
{
    return (motor_fault_code == MOTOR_FAULT_BRAKE_INPUT) ? MOTOR_RESULT_BRAKE : MOTOR_RESULT_FAULT;
}

static void Motor_StartAlign(uint32_t now_ms)
{
    MotionTrap2_Abort(&g_motion_trap2);
    feeder_auto_test_enable = 0;
    feeder_wait_done = 0;
    motor_aligning = 1;
    motor_align_start_tick = now_ms;
    Uq_ref = 0.0f;
    Ud_ref = MOTOR_ALIGN_VOLTAGE;
    m1_foc.Uq = 0.0f;
    m1_foc.Ud = MOTOR_ALIGN_VOLTAGE;
    foc_open(&m1_foc, 0);
}

static void Motor_AlignTask(uint32_t now_ms)
{
    if (!motor_aligning)
        return;

    if ((now_ms - motor_align_start_tick) >= MOTOR_ALIGN_TIME_MS)
    {
        Encoder_Zero(&enc, &htim2);
        Feeder_SetBaseHere(&g_feeder, &enc);
        Pulse_ResetOriginToCurrentPosition();
        motor_aligning = 0;
        motor_fault_code = MOTOR_FAULT_NONE;
        Uq_ref = 0.0f;
        Ud_ref = 0.0f;
        m1_foc.Uq = 0.0f;
        m1_foc.Ud = 0.0f;
        PID_Controller_Reset(&speed_pi);
        canopen_last_motion_ret = 0;
        LED_SetNormal();

    }
}

static void Motor_ProtectionTask(uint32_t now_ms)
{
    static uint32_t last_check = 0;
    uint8_t new_fault;

    if ((now_ms - last_check) < MOTOR_PROTECT_PERIOD_MS)
    {
        if (motor_fault_code != MOTOR_FAULT_NONE)
            LED_ShowFaultCode(motor_fault_code, now_ms);
        return;
    }

    last_check = now_ms;

    vol = get_vol();
    temperature = get_temp();

#if (MOTOR_FAULT_SIM_MODE == MOTOR_FAULT_UNDERVOLTAGE)
    vol = MOTOR_UNDERVOLTAGE_V - 1.0f;
#elif (MOTOR_FAULT_SIM_MODE == MOTOR_FAULT_OVERVOLTAGE)
    vol = MOTOR_OVERVOLTAGE_V + 1.0f;
#elif (MOTOR_FAULT_SIM_MODE == MOTOR_FAULT_OVERTEMP)
    temperature = MOTOR_OVERTEMP_C + 1.0f;
#elif (MOTOR_FAULT_SIM_MODE == MOTOR_FAULT_TEMP_SENSOR)
    temperature = -101.0f;
#endif

    new_fault = Motor_EvaluateFault();

    if (new_fault != MOTOR_FAULT_NONE)
    {
        if ((motor_fault_code == MOTOR_FAULT_NONE) || motor_aligning)
        {
            Motor_StopAll();
            canopen_last_motion_ret = MOTOR_RESULT_FAULT;
        }
        motor_fault_code = new_fault;
    }

    if ((motor_fault_code == MOTOR_FAULT_NONE) && !motor_aligning)
        LED_SetNormal();
    else
        LED_ShowFaultCode(motor_fault_code, now_ms);
}

static void Motor_BrakeTask(uint32_t now_ms)
{
    if (Motor_IsBrakeInputActive())
    {
        if ((motor_fault_code != MOTOR_FAULT_BRAKE_INPUT) || motor_aligning)
        {
            Motor_StopAll();
            motor_fault_code = MOTOR_FAULT_BRAKE_INPUT;
            canopen_last_motion_ret = MOTOR_RESULT_BRAKE;
        }
        LED_ShowFaultCode(motor_fault_code, now_ms);
    }
}

static void LocalMode_Start(local_run_mode_t mode)
{
    switch (mode)
    {
    case LOCAL_MODE_SPEED:
        connect_crt.motor_mode = 2;
        connect_crt.speed = Motor_ApplyDirectionFloat(LOCAL_SPEED_RPM);
        break;

    case LOCAL_MODE_POS_TURNS:
        if (!Motor_IsClosedLoopMode())
            break;
        (void)MotionTrap2_StartByVA(&g_motion_trap2,
                                    &enc,
                                    Motor_ApplyDirectionFloat(LOCAL_MOVE_TURNS),
                                    LOCAL_MOVE_VMAX_RPM,
                                    LOCAL_MOVE_ACC_RPM_S);
        break;

    case LOCAL_MODE_TIME_TURNS:
        if (!Motor_IsClosedLoopMode())
            break;
        (void)MotionTrap2_StartByTimeWithVmax(&g_motion_trap2,
                                              &enc,
                                              Motor_ApplyDirectionFloat(LOCAL_MOVE_TURNS),
                                              LOCAL_MOVE_TIME_S,
                                              LOCAL_MOVE_VMAX_RPM,
                                              LOCAL_MOVE_ACC_RATIO,
                                              LOCAL_MOVE_DEC_RATIO);
        break;

    case LOCAL_MODE_AUTO_FEEDER:
        if (!Motor_IsClosedLoopMode())
            break;
        feeder_auto_test_enable = 1;
        feeder_wait_done = 0;
        feeder_last_tick = HAL_GetTick();
        feeder_interval_ms = LOCAL_FEED_INTERVAL_MS;
        feeder_single_err_turns = 0.0f;
        feeder_total_err_turns = 0.0f;
        feeder_last_actual_total_turns = 0.0f;
        feeder_last_expected_total_turns = 0.0f;
        Feeder_SetBaseHere(&g_feeder, &enc);
        connect_crt.motor_mode = 0;
        connect_crt.speed = 0.0f;
        break;

    case LOCAL_MODE_IDLE:
    default:
        connect_crt.motor_mode = 0;
        connect_crt.speed = 0.0f;
        break;
    }
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
  MX_CAN1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  DipSwitch_Read(&dip_switch);
  CANOpen_Init(&hcan1, dip_switch.comm_addr);

  VOFA_Init();
  LED_Task_Init();
  FOC_Init();
  FOC_OpenLoopInit(&m1_foc, VDC_NOMINAL, 0.0f, 0.0f);

  Encoder_Init(&enc);
  Encoder_Start(&enc, &htim2);
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start(&htim3);

  Motor_ApplyDipSwitchConfig(&dip_switch);
  m1_foc.Ud = MOTOR_ALIGN_VOLTAGE;
  m1_foc.Uq = 0.0f;
  foc_open(&m1_foc, 0);
  HAL_Delay(MOTOR_ALIGN_TIME_MS);
  Encoder_Zero(&enc, &htim2);
  m1_foc.Ud = 0.0f;
  m1_foc.Uq = 0.0f;
  FOC_AllOff();

  init_connect_crt(&connect_crt);
  MotionTrap2_Init(&g_motion_trap2);

  Feeder_Init(&g_feeder,
              &enc,
              Motor_ApplyDirectionFloat(LOCAL_FEED_TURNS),
              LOCAL_FEED_TIME_S,
              LOCAL_FEED_VMAX_RPM,
              LOCAL_FEED_ACC_RATIO,
              LOCAL_FEED_DEC_RATIO);

  feeder_last_tick = HAL_GetTick();
  feeder_interval_ms = LOCAL_FEED_INTERVAL_MS;
  connect_crt.speed = 0.0f;
  connect_crt.s_acc = 1500.0f;

  speed_pi.target = 0.0f;
  speed_pi.actual_value = 0.0f;
  speed_pi.output_max = fabsf(connect_crt.drive_current);
  speed_pi.output_min = -fabsf(connect_crt.drive_current);

  Uq_ref = 0.0f;
  Ud_ref = 0.0f;

  LocalMode_Start(LOCAL_RUN_MODE);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    static uint32_t last_vofa = 0;
    uint8_t cmd_from_usb = 0U;

#if (USB_ASCII_DEBUG_ENABLE != 0U)
    {
        static uint32_t last_usb_debug_top = 0U;
        if ((now - last_usb_debug_top) >= 500U)
        {
            last_usb_debug_top = now;
            USBVCP_SendString("USB OK\r\n");
        }
        continue;
    }
#endif

#if (VOFA_DIRECT_DEBUG_ENABLE != 0U)
    {
        static uint32_t last_vofa_direct_debug = 0U;
        if ((now - last_vofa_direct_debug) >= 10U)
        {
            float t = (float)(now % 1000U) * 0.001f;
            last_vofa_direct_debug = now;
            VOFA_SendFrame6(t, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f);
        }
        VOFA_Process();
        continue;
    }
#endif

    CANOpen_Process();
    Motor_ProtectionTask(now);
    Motor_BrakeTask(now);
    Motor_AlignTask(now);
    Pulse_InputTask();
    Pulse_SimTask(now);

    if (g_usb_vcp_force_pulse_view)
    {
        g_vofa_view = VOFA_VIEW_PULSE;
        g_usb_vcp_force_pulse_view = 0U;
    }

    if (g_usb_vcp_speed_pi_update_request)
    {
        speed_pi.kp = (float)g_usb_vcp_speed_kp_milli * 0.001f;
        speed_pi.ki = (float)g_usb_vcp_speed_ki_10000 * 0.0001f;
        speed_pi.kd = 0.0f;
        PID_Controller_Reset(&speed_pi);
        g_vofa_view = VOFA_VIEW_SPEED;
        g_usb_vcp_speed_pi_update_request = 0U;
    }

#if (USB_ASCII_DEBUG_ENABLE != 0U)
    {
        static uint32_t last_usb_debug = 0U;
        if ((now - last_usb_debug) >= 500U)
        {
            last_usb_debug = now;
            USBVCP_SendString("USB OK\r\n");
        }
    }
#else

    if (CANOpen_GetCommand(&can_cmd) || ((cmd_from_usb = USBVCP_GetCommand(&can_cmd)) != 0U))
    {
      uint8_t command_rejected = 0U;

      g_pulse_manual_holdoff = 1U;
      g_control_source = cmd_from_usb ? CONTROL_SOURCE_VOFA : CONTROL_SOURCE_CANOPEN;
      g_pulse_follow_active = 0U;
      DipSwitch_Read(&dip_switch);
      Motor_ApplyDipSwitchConfig(&dip_switch);

      canopen_last_motion_ret = 0;

      if (((motor_fault_code != MOTOR_FAULT_NONE) || motor_aligning) &&
          (can_cmd.command != CANOPEN_CMD_STOP) &&
          (can_cmd.command != CANOPEN_CMD_ZERO) &&
          (can_cmd.command != CANOPEN_CMD_CLEAR_FAULT) &&
          (can_cmd.command != CANOPEN_CMD_ALIGN))
      {
          Motor_StopAll();
          canopen_last_motion_ret = Motor_GetFaultResult();
          command_rejected = 1U;
      }

      if ((!command_rejected) &&
          (!cmd_from_usb) &&
          (g_canopen.state != CANOPEN_STATE_OPERATIONAL) &&
          (can_cmd.command != CANOPEN_CMD_STOP) &&
          (can_cmd.command != CANOPEN_CMD_ZERO) &&
          (can_cmd.command != CANOPEN_CMD_CLEAR_FAULT) &&
          (can_cmd.command != CANOPEN_CMD_ALIGN))
      {
          Motor_StopAll();
          canopen_last_motion_ret = -10;
          command_rejected = 1U;
      }

      if (command_rejected)
      {
          continue;
      }

      if ((!Motor_IsClosedLoopMode()) &&
          (can_cmd.command != CANOPEN_CMD_STOP) &&
          (can_cmd.command != CANOPEN_CMD_ZERO) &&
          (can_cmd.command != CANOPEN_CMD_CLEAR_FAULT) &&
          (can_cmd.command != CANOPEN_CMD_ALIGN))
      {
          if (!((can_cmd.command == CANOPEN_CMD_START) &&
                (can_cmd.mode == CANOPEN_MODE_SPEED)))
          {
              Motor_StopAll();
              canopen_last_motion_ret = MOTION_ERR_PARAM;
              continue;
          }
      }

      switch (can_cmd.command)
      {
      case CANOPEN_CMD_STOP:
          Motor_StopAll();
          canopen_last_motion_ret = (motor_fault_code == MOTOR_FAULT_NONE) ? 0 : Motor_GetFaultResult();
          break;

      case CANOPEN_CMD_START:
          if (can_cmd.mode == CANOPEN_MODE_SPEED)
          {
              float speed_abs_rpm;
              float speed_cmd_rpm;

              if (can_cmd.target_speed_rpm == 0)
              {
                  Motor_StopAll();
                  canopen_last_motion_ret = 0;
                  break;
              }

              speed_abs_rpm = fabsf((float)can_cmd.target_speed_rpm);
              speed_cmd_rpm = Motor_ApplyDirectionFloat(speed_abs_rpm);

              if (connect_crt.motor_mode != 2)
              {
                  Motor_PrepareMotionStart();
                  speed_pi.target = 0.0f;
                  motor_open_loop_angle = (speed_cmd_rpm >= 0.0f) ? 256.0f : 768.0f;
              }

              connect_crt.motor_mode = 2;
              connect_crt.speed = speed_cmd_rpm;
              g_vofa_view = VOFA_VIEW_SPEED;
              canopen_last_motion_ret = 0;
          }
          else if (can_cmd.mode == CANOPEN_MODE_POS_TURNS)
          {
              int32_t move_delta_cnt;
              int64_t compensated_target_cnt;

              if (!Motor_IsClosedLoopMode())
              {
                  canopen_last_motion_ret = MOTION_ERR_PARAM;
                  break;
              }
              float turns = Motor_ApplyDirectionFloat(((float)can_cmd.value1) * 0.01f);
              float vmax_rpm = (float)can_cmd.value2;
              float acc_rpm_s = (float)can_cmd.value3;

              if (MotionTrap2_IsBusy(&g_motion_trap2))
              {
                  canopen_last_motion_ret = MOTION_ERR_BUSY;
                  break;
              }

              if ((fabsf(turns) < 0.001f) || (vmax_rpm <= 0.0f) || (acc_rpm_s <= 0.0f))
              {
                  canopen_last_motion_ret = MOTION_ERR_PARAM;
                  break;
              }

              move_delta_cnt = (int32_t)(turns * MOTION_COUNTS_PER_REV);
              if (!g_relative_motion_expected_valid)
              {
                  g_relative_motion_expected_cnt = enc.pos_cnt_total;
                  g_relative_motion_expected_valid = 1U;
              }
              g_relative_motion_expected_cnt += move_delta_cnt;
              compensated_target_cnt = g_relative_motion_expected_cnt;

              Motor_PrepareMotionStart();
              g_vofa_view = VOFA_VIEW_POSITION;
              canopen_last_motion_ret = MotionTrap2_StartToAbsCountByVA(&g_motion_trap2,
                                                                        &enc,
                                                                        compensated_target_cnt,
                                                                        vmax_rpm,
                                                                        acc_rpm_s);
          }
          else if (can_cmd.mode == CANOPEN_MODE_POS_PULSES)
          {
              int64_t command_delta_pulses;
              int64_t compensated_target_cnt;
              float vmax_rpm = (float)can_cmd.value2;
              float acc_rpm_s = (float)can_cmd.value3;

              if (!Motor_IsClosedLoopMode())
              {
                  canopen_last_motion_ret = MOTION_ERR_PARAM;
                  break;
              }

              if (MotionTrap2_IsBusy(&g_motion_trap2))
              {
                  canopen_last_motion_ret = MOTION_ERR_BUSY;
                  break;
              }

              command_delta_pulses = (motor_dir_sign < 0) ? -(int64_t)can_cmd.value32 : (int64_t)can_cmd.value32;
              if ((command_delta_pulses == 0) || (vmax_rpm <= 0.0f) || (acc_rpm_s <= 0.0f))
              {
                  canopen_last_motion_ret = MOTION_ERR_PARAM;
                  break;
              }

              if (!g_canopen_position_base_valid)
              {
                  g_canopen_position_base_cnt = enc.pos_cnt_total;
                  g_canopen_position_command_pulses = 0;
                  g_canopen_position_base_valid = 1U;
              }
              g_canopen_position_command_pulses += command_delta_pulses;
              compensated_target_cnt = g_canopen_position_base_cnt +
                                       Motor_CommandPulsesToCountOffset(g_canopen_position_command_pulses);

              Motor_PrepareMotionStart();
              g_vofa_view = VOFA_VIEW_POSITION;
              canopen_last_motion_ret = MotionTrap2_StartToAbsCountByVA(&g_motion_trap2,
                                                                        &enc,
                                                                        compensated_target_cnt,
                                                                        vmax_rpm,
                                                                        acc_rpm_s);
          }
          else if (can_cmd.mode == CANOPEN_MODE_TIME_TURNS)
          {
              if (!Motor_IsClosedLoopMode())
              {
                  canopen_last_motion_ret = MOTION_ERR_PARAM;
                  break;
              }
              float turns = Motor_ApplyDirectionFloat(((float)can_cmd.value1) * 0.01f);
              float total_time_s = ((float)can_cmd.value2) * 0.001f;
              float vmax_rpm = (float)can_cmd.value3;

              if (MotionTrap2_IsBusy(&g_motion_trap2))
              {
                  canopen_last_motion_ret = MOTION_ERR_BUSY;
                  break;
              }

              if ((fabsf(turns) < 0.001f) || (total_time_s <= 0.0f) || (vmax_rpm <= 0.0f))
              {
                  canopen_last_motion_ret = MOTION_ERR_PARAM;
                  break;
              }

              Motor_PrepareMotionStart();
              g_vofa_view = VOFA_VIEW_POSITION;
              canopen_last_motion_ret = MotionTrap2_StartByTimeWithVmax(&g_motion_trap2,
                                                                        &enc,
                                                                        turns,
                                                                        total_time_s,
                                                                        vmax_rpm,
                                                                        LOCAL_FEED_ACC_RATIO,
                                                                        LOCAL_FEED_DEC_RATIO);
          }
          else
          {
              canopen_last_motion_ret = MOTION_ERR_PARAM;
          }
          break;

      case CANOPEN_CMD_ZERO:
          Encoder_Zero(&enc, &htim2);
          Feeder_SetBaseHere(&g_feeder, &enc);
          Pulse_ResetOriginToCurrentPosition();
          g_relative_motion_expected_cnt = enc.pos_cnt_total;
          g_relative_motion_expected_valid = 1U;
          g_canopen_position_base_cnt = enc.pos_cnt_total;
          g_canopen_position_command_pulses = 0;
          g_canopen_position_base_valid = 1U;
          canopen_last_motion_ret = 0;
          break;

      case CANOPEN_CMD_FEED_ONCE:
          if (!Motor_IsClosedLoopMode())
          {
              canopen_last_motion_ret = MOTION_ERR_PARAM;
              break;
          }
          if (MotionTrap2_IsBusy(&g_motion_trap2))
          {
              canopen_last_motion_ret = MOTION_ERR_BUSY;
              break;
          }
          Motor_PrepareMotionStart();
          if (!g_feeder.initialized)
          {
              Feeder_SetBaseHere(&g_feeder, &enc);
          }
          g_vofa_view = VOFA_VIEW_FEEDER;
          canopen_last_motion_ret = Feeder_Once(&g_feeder, &g_motion_trap2, &enc);
          break;

      case CANOPEN_CMD_AUTO_FEED_START:
          if (!Motor_IsClosedLoopMode())
          {
              canopen_last_motion_ret = MOTION_ERR_PARAM;
              break;
          }
          if (can_cmd.mode == CANOPEN_MODE_FEEDER)
          {
              float turns_per_feed = Motor_ApplyDirectionFloat(((float)can_cmd.value1) * 0.01f);
              float time_per_feed_s = ((float)can_cmd.value2) * 0.001f;
              uint32_t interval_ms = (uint32_t)can_cmd.value3;

              if (feeder_auto_test_enable)
              {
                  canopen_last_motion_ret = 0;
                  break;
              }

              if ((fabsf(turns_per_feed) < 0.001f) ||
                  (time_per_feed_s <= 0.0f) ||
                  (interval_ms == 0U))
              {
                  canopen_last_motion_ret = MOTION_ERR_PARAM;
                  break;
              }

              Motor_PrepareMotionStart();
              Feeder_SetBaseHere(&g_feeder, &enc);
              g_feeder.turns_per_feed = turns_per_feed;
              g_feeder.time_per_feed_s = time_per_feed_s;
              g_feeder.vmax_limit_rpm = LOCAL_FEED_VMAX_RPM;
              g_feeder.acc_ratio = LOCAL_FEED_ACC_RATIO;
              g_feeder.dec_ratio = LOCAL_FEED_DEC_RATIO;
              g_feeder.initialized = 1;

              feeder_interval_ms = interval_ms;
              feeder_last_tick = HAL_GetTick() - feeder_interval_ms;
              feeder_single_err_turns = 0.0f;
              feeder_total_err_turns = 0.0f;
              feeder_last_actual_total_turns = (float)(enc.pos_cnt_total - g_feeder.base_cnt) / 4000.0f;
              feeder_last_expected_total_turns = (float)g_feeder.feed_index * g_feeder.turns_per_feed;

              feeder_auto_test_enable = 1;
              g_vofa_view = VOFA_VIEW_FEEDER;
              canopen_last_motion_ret = 0;
          }
          else
          {
              canopen_last_motion_ret = MOTION_ERR_PARAM;
          }
          break;

      case CANOPEN_CMD_AUTO_FEED_STOP:
          Motor_StopAll();
          canopen_last_motion_ret = 0;
          break;

      case CANOPEN_CMD_CLEAR_FAULT:
      case CANOPEN_CMD_ALIGN:
          vol = get_vol();
          temperature = get_temp();

#if (MOTOR_FAULT_SIM_MODE == MOTOR_FAULT_UNDERVOLTAGE)
          vol = MOTOR_UNDERVOLTAGE_V - 1.0f;
#elif (MOTOR_FAULT_SIM_MODE == MOTOR_FAULT_OVERVOLTAGE)
          vol = MOTOR_OVERVOLTAGE_V + 1.0f;
#elif (MOTOR_FAULT_SIM_MODE == MOTOR_FAULT_OVERTEMP)
          temperature = MOTOR_OVERTEMP_C + 1.0f;
#elif (MOTOR_FAULT_SIM_MODE == MOTOR_FAULT_TEMP_SENSOR)
          temperature = -101.0f;
#endif

          motor_fault_code = Motor_EvaluateFault();
          if ((motor_fault_code == MOTOR_FAULT_NONE) && Motor_IsBrakeInputActive())
          {
              motor_fault_code = MOTOR_FAULT_BRAKE_INPUT;
          }

          if (motor_fault_code == MOTOR_FAULT_NONE)
          {
              Motor_StopAll();
              motor_fault_code = MOTOR_FAULT_NONE;
              Motor_StartAlign(now);
              canopen_last_motion_ret = 0;
          }
          else
          {
              Motor_StopAll();
              canopen_last_motion_ret = Motor_GetFaultResult();
          }
          break;

      default:
          canopen_last_motion_ret = MOTION_ERR_PARAM;
          break;
      }
    }

    if (feeder_auto_test_enable)
    {
      if ((!MotionTrap2_IsBusy(&g_motion_trap2)) &&
          (!feeder_wait_done) &&
          ((now - feeder_last_tick) >= feeder_interval_ms))
      {
        int ret = Feeder_Once(&g_feeder, &g_motion_trap2, &enc);
        canopen_last_motion_ret = ret;
        if (ret == MOTION_OK)
        {
          feeder_wait_done = 1;
        }
      }

      if (feeder_wait_done && MotionTrap2_IsDone(&g_motion_trap2))
      {
        float actual_total_turns = (float)(enc.pos_cnt_total - g_feeder.base_cnt) / 4000.0f;
        float expected_total_turns = (float)g_feeder.feed_index * g_feeder.turns_per_feed;
        float actual_move_turns = actual_total_turns - feeder_last_actual_total_turns;
        float expected_move_turns = expected_total_turns - feeder_last_expected_total_turns;

        feeder_single_err_turns = actual_move_turns - expected_move_turns;
        feeder_total_err_turns = actual_total_turns - expected_total_turns;
        feeder_last_actual_total_turns = actual_total_turns;
        feeder_last_expected_total_turns = expected_total_turns;

        Uq_ref = 0.0f;
        Ud_ref = 0.0f;
        feeder_wait_done = 0;
        feeder_last_tick = now;
      }
    }

    if (g_usb_vcp_wave_enable && ((now - last_vofa) >= 10))
    {
        float ch1;
        float ch2;
        float ch3;
        float ch4;
        float ch5;
        float ch6;
        float actual_total_turns;
        float expected_total_turns;
        float pulse_target_turns_disp;
        float pulse_actual_turns_disp;
        float pulse_err_turns_disp;
        float pulse_target_speed_disp;
        float pulse_actual_speed_disp;

        last_vofa = now;

        actual_total_turns = (float)(enc.pos_cnt_total - g_feeder.base_cnt) / 4000.0f;
        expected_total_turns = (float)g_feeder.feed_index * g_feeder.turns_per_feed;
        pulse_target_turns_disp = g_pulse_target_turns;
        pulse_actual_turns_disp = enc.pos_rev - g_pulse_origin_turns;
        pulse_err_turns_disp = pulse_target_turns_disp - pulse_actual_turns_disp;
        pulse_target_speed_disp = g_pulse_follow_speed_cmd_rpm;
        pulse_actual_speed_disp = (g_pulse_follow_speed_cmd_rpm >= 0.0f) ? fabsf(enc.vel_rpm_f) : -fabsf(enc.vel_rpm_f);

        if (g_vofa_view == VOFA_VIEW_FEEDER)
        {
            ch1 = expected_total_turns;
            ch2 = actual_total_turns;
            ch3 = feeder_single_err_turns;
            ch4 = feeder_total_err_turns;
            ch5 = (float)g_feeder.feed_index;
            ch6 = enc.vel_rpm_f;
        }
        else if (g_vofa_view == VOFA_VIEW_POSITION)
        {
            ch1 = MotionTrap2_IsBusy(&g_motion_trap2) ? g_motion_trap2.pos_ref_turns : enc.pos_rev;
            ch2 = enc.pos_rev;
            ch3 = MotionTrap2_IsBusy(&g_motion_trap2) ? g_motion_trap2.last_err_turns : 0.0f;
            ch4 = enc.vel_rpm_f;
            ch5 = Uq_ref;
            ch6 = vol;
        }
        else if (g_vofa_view == VOFA_VIEW_PULSE)
        {
            ch1 = (float)g_pulse_count;
            ch2 = (float)g_pulse_per_rev;
            ch3 = pulse_target_speed_disp;
            ch4 = pulse_actual_speed_disp;
            ch5 = Uq_ref;
            ch6 = (float)g_pulse_sim_en_level;
        }
        else if (g_vofa_view == VOFA_VIEW_PULSEDBG)
        {
            ch1 = pulse_target_turns_disp;
            ch2 = pulse_actual_turns_disp;
            ch3 = g_pulse_follow_desired_turns - g_pulse_origin_turns;
            ch4 = pulse_err_turns_disp;
            ch5 = pulse_target_speed_disp;
            ch6 = pulse_actual_speed_disp;
        }
        else
        {
            if (!Motor_IsClosedLoopMode())
            {
                ch1 = speed_pi.target;
                ch2 = (float)g_open_loop_debug_update_count;
                ch3 = (float)g_ctrl_mode;
                ch4 = Uq_ref;
                ch5 = (float)motor_aligning;
                ch6 = (float)connect_crt.motor_mode;
            }
            else
            {
                ch1 = speed_pi.target;
                ch2 = enc.vel_rpm_f;
                ch3 = speed_pi.target - enc.vel_rpm_f;
                ch4 = Uq_ref;
                ch5 = speed_pi.kp * 1000.0f;
                ch6 = (float)m1_foc.lead_angle;
            }
        }

        VOFA_SendFrame6(ch1, ch2, ch3, ch4, ch5, ch6);
    }

    if (g_usb_vcp_wave_enable)
    {
        VOFA_Process();
    }
#endif
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
uint16_t a_angle = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        if (!Motor_IsClosedLoopMode())
        {
            if (motor_aligning)
            {
                m1_foc.Uq = 0.0f;
                m1_foc.Ud = MOTOR_ALIGN_VOLTAGE;
                foc_open(&m1_foc, 0);
            }
            else
            {
                if ((g_control_source == CONTROL_SOURCE_PULSE) &&
                        g_pulse_follow_active &&
                        g_pulse_sim_en_level)
                {
                    int32_t pulse_count_snapshot;
                    int32_t pulse_delta;
                    float angle_step;

                    pulse_count_snapshot = g_pulse_count;
                    pulse_delta = pulse_count_snapshot - g_pulse_open_loop_last_count;
                    g_pulse_open_loop_last_count = pulse_count_snapshot;

                    if (pulse_delta != 0)
                        g_pulse_open_loop_idle_s = 0.0f;
                    else
                        g_pulse_open_loop_idle_s += CTRL_DT;

                    angle_step = 1024.0f / (4.0f * (float)g_pulse_microstep_value);
                    motor_open_loop_angle += (float)pulse_delta * angle_step;
                    g_open_loop_debug_step = (float)pulse_delta * angle_step;
                    g_open_loop_debug_update_count++;

                    if (g_pulse_open_loop_idle_s >= PULSE_STOP_TIMEOUT_S)
                    {
                        Uq_ref = 0.0f;
                        Ud_ref = 0.0f;
                    }
                    else
                    {
                        Uq_ref = PULSE_OPEN_LOOP_UQ;
                        if (Uq_ref > connect_crt.drive_current)
                            Uq_ref = connect_crt.drive_current;
                        Ud_ref = 0.0f;
                    }
                }
                else if (connect_crt.motor_mode == 2)
                {
                    speed_pi.target = connect_crt.speed;
                    g_open_loop_debug_step = speed_pi.target * OPEN_LOOP_ANGLE_STEP_PER_RPM;
                    motor_open_loop_angle += g_open_loop_debug_step;
                    g_open_loop_debug_update_count++;

                    if (g_usb_vcp_manual_uq_enable)
                    {
                        Uq_ref = (float)g_usb_vcp_manual_uq_centivolt * 0.01f;
                        if (Uq_ref < 0.0f)
                            Uq_ref = 0.0f;
                        if (Uq_ref > connect_crt.drive_current)
                            Uq_ref = connect_crt.drive_current;
                    }
                    else
                    {
                        Uq_ref = SPEED_UQ_FF_GAIN_V_PER_RPM * fabsf(speed_pi.target);
                        if ((fabsf(speed_pi.target) > 1.0f) && (Uq_ref < OPEN_LOOP_MIN_UQ))
                            Uq_ref = OPEN_LOOP_MIN_UQ;
                        if (Uq_ref > connect_crt.drive_current)
                            Uq_ref = connect_crt.drive_current;
                    }
                    Ud_ref = 0.0f;
                }
                else
                {
                    speed_pi.target = 0.0f;
                    Uq_ref = 0.0f;
                    Ud_ref = 0.0f;
                }

                while (motor_open_loop_angle >= 1024.0f)
                    motor_open_loop_angle -= 1024.0f;
                while (motor_open_loop_angle < 0.0f)
                    motor_open_loop_angle += 1024.0f;

                m1_foc.Uq = Uq_ref;
                m1_foc.Ud = Ud_ref;
                if ((fabsf(Uq_ref) < 0.001f) && (fabsf(Ud_ref) < 0.001f))
                {
                    FOC_AllOff();
                    g_open_loop_debug_sector = 0U;
                }
                else
                {
                    foc_open(&m1_foc, motor_open_loop_angle);
                    g_open_loop_debug_sector = m1_foc.sector;
                }
            }
            return;
        }

        Encoder_Update(&enc, &htim2, CTRL_DT);
        Sector_tracker_inc_encoder(&m1_foc, &enc);

        speed_pi.actual_value = enc.vel_rpm_f;

        speed_loop_div++;
        if (speed_loop_div >= 10)
        {
            speed_loop_div = 0;

            if ((g_control_source == CONTROL_SOURCE_PULSE) && g_pulse_follow_active && g_pulse_sim_en_level)
            {
                float desired_turns;
                float actual_turns;
                float pos_err_turns;
                int32_t pulse_count_snapshot;
                int32_t pulse_delta;
                float speed_cmd_rpm;

                desired_turns = g_pulse_origin_turns + g_pulse_target_turns;
                actual_turns = enc.pos_rev;
                pos_err_turns = desired_turns - actual_turns;

                g_pulse_follow_desired_turns = desired_turns;
                g_pulse_follow_pos_err_turns = pos_err_turns;

                pulse_count_snapshot = g_pulse_count;
                pulse_delta = pulse_count_snapshot - g_pulse_follow_last_count;
                g_pulse_follow_last_count = pulse_count_snapshot;
                g_pulse_speed_accum_count += pulse_delta;
                g_pulse_speed_window_s += SPEED_LOOP_DT;

                if (pulse_delta != 0)
                    g_pulse_idle_s = 0.0f;
                else
                    g_pulse_idle_s += SPEED_LOOP_DT;

                if (g_pulse_speed_window_s >= PULSE_SPEED_WINDOW_S)
                {
                    float pulse_delta_turns = PULSE_TARGET_SIGN *
                                             ((float)g_pulse_speed_accum_count / (float)g_pulse_per_rev);
                    float pulse_speed_rpm;

                    pulse_speed_rpm = (pulse_delta_turns / g_pulse_speed_window_s) * 60.0f;
                    g_pulse_speed_ff_rpm += PULSE_SPEED_ALPHA * (pulse_speed_rpm - g_pulse_speed_ff_rpm);
                    g_pulse_speed_accum_count = 0;
                    g_pulse_speed_window_s = 0.0f;
                }

                if (g_pulse_idle_s >= PULSE_STOP_TIMEOUT_S)
                {
                    g_pulse_speed_ff_rpm = 0.0f;
                    g_pulse_speed_accum_count = 0;
                    g_pulse_speed_window_s = 0.0f;
                    speed_cmd_rpm = 0.0f;
                    connect_crt.motor_mode = 2;
                    connect_crt.speed = 0.0f;
                }
                else
                {
                    speed_cmd_rpm = g_pulse_speed_ff_rpm;
                    if (speed_cmd_rpm > PULSE_TRACK_MAX_RPM)
                        speed_cmd_rpm = PULSE_TRACK_MAX_RPM;
                    else if (speed_cmd_rpm < -PULSE_TRACK_MAX_RPM)
                        speed_cmd_rpm = -PULSE_TRACK_MAX_RPM;

                    connect_crt.motor_mode = 2;
                    connect_crt.speed = (float)motor_dir_sign * fabsf(speed_cmd_rpm);
                }
                g_pulse_follow_speed_cmd_rpm = speed_cmd_rpm;
            }
            else
            {
                g_pulse_follow_pos_err_turns = 0.0f;
                g_pulse_follow_speed_cmd_rpm = 0.0f;
                g_pulse_speed_accum_count = 0;
                g_pulse_speed_window_s = 0.0f;
                g_pulse_speed_ff_rpm = 0.0f;
                MotionTrap2_Update(&g_motion_trap2, &enc, enc.vel_rpm_f, SPEED_LOOP_DT);
            }
            if ((g_control_source != CONTROL_SOURCE_PULSE) && MotionTrap2_IsDone(&g_motion_trap2))
            {
                connect_crt.motor_mode = 0;
                connect_crt.speed = 0.0f;
                speed_pi.target = 0.0f;
                PID_Controller_Reset(&speed_pi);
                Uq_ref = 0.0f;
                Ud_ref = 0.0f;
            }
            else if (connect_crt.motor_mode == 2)
            {
                if (speed_pi.target > connect_crt.speed)
                {
                    speed_pi.target -= connect_crt.s_acc * SPEED_LOOP_DT;
                    if (speed_pi.target < connect_crt.speed)
                        speed_pi.target = connect_crt.speed;
                }
                else if (speed_pi.target < connect_crt.speed)
                {
                    speed_pi.target += connect_crt.s_acc * SPEED_LOOP_DT;
                    if (speed_pi.target > connect_crt.speed)
                        speed_pi.target = connect_crt.speed;
                }

                if (g_usb_vcp_manual_uq_enable)
                {
                    Uq_ref = (float)g_usb_vcp_manual_uq_centivolt * 0.01f;
                    if (Uq_ref > connect_crt.drive_current)
                        Uq_ref = connect_crt.drive_current;
                }
                else
                {
                    float speed_error = speed_pi.target - speed_pi.actual_value;
                    float speed_dir = (speed_pi.target >= 0.0f) ? 1.0f : -1.0f;
                    float uq_ff = SPEED_UQ_FF_GAIN_V_PER_RPM * fabsf(speed_pi.target);
                    float integral_before = speed_pi.integral;
                    float uq_pi = PID_Controller_Update(&speed_pi, speed_error);
                    float uq_cmd;

                    uq_cmd = uq_ff + speed_dir * uq_pi;
                    if (uq_cmd > connect_crt.drive_current)
                    {
                        uq_cmd = connect_crt.drive_current;
                        if ((speed_dir * speed_error) > 0.0f)
                            speed_pi.integral = integral_before;
                    }
                    else if (uq_cmd < 0.0f)
                    {
                        uq_cmd = 0.0f;
                        if ((speed_dir * speed_error) < 0.0f)
                            speed_pi.integral = integral_before;
                    }

                    Uq_ref = Motor_SlewFloat(Uq_ref,
                                             uq_cmd,
                                             SPEED_UQ_SLEW_V_PER_S * SPEED_LOOP_DT);
                }

                Ud_ref = -((float)g_usb_vcp_fwv_centivolt * 0.01f);
            }
            else
            {
                Uq_ref = 0.0f;
                Ud_ref = 0.0f;
            }
        }

        if (motor_aligning)
        {
            m1_foc.Uq = 0.0f;
            m1_foc.Ud = MOTOR_ALIGN_VOLTAGE;
            foc_open(&m1_foc, 0);
        }
        else
        {
            if (g_usb_vcp_manual_lead_enable)
            {
                int16_t lead = g_usb_vcp_manual_lead_angle;

                if (motor_dir_sign < 0)
                    lead = (int16_t)(-lead);

                while (lead < 0)
                    lead = (int16_t)(lead + 1024);
                while (lead >= 1024)
                    lead = (int16_t)(lead - 1024);

                m1_foc.lead_angle = (uint16_t)lead;
            }
            else
            {
                m1_foc.lead_angle = Motor_GetLeadAngleBySpeed(speed_pi.target);
            }
            m1_foc.Uq = Uq_ref * (float)motor_dir_sign;
            m1_foc.Ud = Ud_ref;
            foc_open(&m1_foc, (uint16_t)((m1_foc.angle + m1_foc.lead_angle) % 1024));
        }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    (void)htim;
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
