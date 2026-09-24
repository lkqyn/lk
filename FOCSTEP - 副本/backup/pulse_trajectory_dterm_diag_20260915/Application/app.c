#include "app.h"

#include "current_sense.h"
#include "debug.h"
#include "dq_transform.h"
#include "electrical_angle.h"
#include "encoder.h"
#include "motor_pwm.h"
#include "motor_test.h"
#include "power_monitor.h"
#include "position_loop.h"
#include "pulse_input.h"
#include "rotor_alignment.h"
#include "speed_loop.h"

/*
 * 上电标定链：先完成小角度电角零点对齐，再扫描一整圈编码器非线性表，
 * 最后启用该表。每个状态只前进一次；任何故障均停留在故障状态，禁止反复驱动。
 */
typedef enum
{
    APP_AUTO_CALIBRATION_WAIT_ALIGNMENT = 0,
    APP_AUTO_CALIBRATION_RUNNING,
    APP_AUTO_CALIBRATION_ENABLE_TABLE,
    APP_AUTO_CALIBRATION_COMPLETE,
    APP_AUTO_CALIBRATION_FAULT
} App_AutoCalibrationState_t;

static App_AutoCalibrationState_t s_auto_calibration_state;

static void App_UpdateAutoCalibration(void)
{
    const RotorAlignment_State_t *alignment;
    const EncoderCalibration_State_t *calibration;

    switch (s_auto_calibration_state)
    {
        case APP_AUTO_CALIBRATION_WAIT_ALIGNMENT:
            alignment = RotorAlignment_GetState();
            if ((alignment->state == ROTOR_ALIGNMENT_STATE_COMPLETE) &&
                (alignment->valid != 0U))
            {
                if (RotorAlignment_StartEncoderCalibration() != 0U)
                {
                    s_auto_calibration_state = APP_AUTO_CALIBRATION_RUNNING;
                }
                else
                {
                    s_auto_calibration_state = APP_AUTO_CALIBRATION_FAULT;
                }
            }
            else if (alignment->state == ROTOR_ALIGNMENT_STATE_FAULT)
            {
                s_auto_calibration_state = APP_AUTO_CALIBRATION_FAULT;
            }
            break;

        case APP_AUTO_CALIBRATION_RUNNING:
            calibration = RotorAlignment_GetEncoderCalibrationState();
            if ((calibration->state == ENCODER_CALIBRATION_STATE_COMPLETE) &&
                (calibration->data_valid != 0U))
            {
                s_auto_calibration_state = APP_AUTO_CALIBRATION_ENABLE_TABLE;
            }
            else if (calibration->state == ENCODER_CALIBRATION_STATE_FAULT)
            {
                s_auto_calibration_state = APP_AUTO_CALIBRATION_FAULT;
            }
            break;

        case APP_AUTO_CALIBRATION_ENABLE_TABLE:
            if (ElectricalAngle_EnableNonlinearityCalibration(1U) != 0U)
            {
                /*
                 * 产品态：校准完成即永久启动STEP/DIR接口。之后PUL_EN为
                 * 唯一接收门控；不依赖USB或任何调试命令。
                 */
                if (PulseInput_Start() != 0U)
                {
                    s_auto_calibration_state = APP_AUTO_CALIBRATION_COMPLETE;
                }
                else
                {
                    s_auto_calibration_state = APP_AUTO_CALIBRATION_FAULT;
                }
            }
            else
            {
                s_auto_calibration_state = APP_AUTO_CALIBRATION_FAULT;
            }
            break;

        default:
            /* 已完成或故障后均不再自动重试，等待人工通过串口诊断。 */
            break;
    }
}

void App_Init(void)
{
    /* 安全优先：控制参数未确认前不启动PWM调制，保持低端制动。 */
    MotorPWM_Init();
    Encoder_Init();
    ElectricalAngle_Init();
    DqTransform_Init();
    CurrentSense_Init();
    PowerMonitor_Init();
    RotorAlignment_Init();
    SpeedLoop_Init();
    PositionLoop_Init();
    PulseInput_Init();
    Debug_Init();
    s_auto_calibration_state = APP_AUTO_CALIBRATION_WAIT_ALIGNMENT;
}

void App_Loop(void)
{
    Encoder_Update();
    ElectricalAngle_Update(Encoder_GetState()->position_count);
    CurrentSense_Update();
    PowerMonitor_Update();
    RotorAlignment_Update();
    App_UpdateAutoCalibration();
    MotorTest_UpdateDqCycle();
    Debug_Loop();
    Debug_TelemetryLoop();
}

void App_Tick1ms(void)
{
    Encoder_Tick1ms();
    PulseInput_Tick1ms();
    PositionLoop_Tick1ms();
    SpeedLoop_Tick1ms();
}

void App_Tick250us(void)
{
    /* TIM4优先级低于ADC电流环；此处只运行无阻塞的外部脉冲外环。 */
    PulseInput_Tick250us();
    PositionLoop_Tick250us();
}
