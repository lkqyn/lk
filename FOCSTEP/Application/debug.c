#include "debug.h"

#include "app.h"
#include "current.h"
#include "encoder.h"
#include "foc.h"
#include "foc_v.h"
#include "main.h"
#include "motion_ctrl.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG_RX_BUF_SIZE       (128U)
#define DEBUG_TX_BUF_SIZE       (384U)
#define DEBUG_JUSTFLOAT_CH      (6U)
#define DEBUG_JUSTFLOAT_PERIOD  (5U)
#define DEBUG_JUSTFLOAT_TAIL0   (0x00U)
#define DEBUG_JUSTFLOAT_TAIL1   (0x00U)
#define DEBUG_JUSTFLOAT_TAIL2   (0x80U)
#define DEBUG_JUSTFLOAT_TAIL3   (0x7FU)
#define DEBUG_DEFAULT_CURRENT_VOLTAGE_LIMIT (0.95f)
#define DEBUG_DEFAULT_SPEED_KP  (0.0022f)
#define DEBUG_DEFAULT_SPEED_KI  (0.0200f)
#define DEBUG_DEFAULT_SPEED_MAX_IQ (0.50f)
#define DEBUG_DEFAULT_SPEED_MAX_INTEGRAL_IQ (0.12f)
#define DEBUG_DEFAULT_BUS_VOLTAGE_V (24.0f)
#define DEBUG_SPEED_CMD_LIMIT_RPM (500.0f)
#define DEBUG_TRACE_VIEW_LIVE   (0U)
#define DEBUG_TRACE_VIEW_CROSS  (1U)
#define DEBUG_TRACE_VIEW_PEAK   (2U)
#define DEBUG_TRACE_VIEW_ENCODER (3U)
#define DEBUG_LIVE_VIEW_PHASE   (0U)
#define DEBUG_LIVE_VIEW_CURR    (1U)
#define DEBUG_LIVE_VIEW_VOLT    (2U)
#define DEBUG_LIVE_VIEW_AB      (3U)
#define DEBUG_RAD_TO_DEG        (57.2957795130823208768f)

static char s_debug_rx_buf[DEBUG_RX_BUF_SIZE];
static volatile uint16_t s_debug_rx_len;
static volatile uint8_t s_debug_cmd_ready;
static char s_debug_cmd_buf[DEBUG_RX_BUF_SIZE];
static char s_debug_tx_buf[DEBUG_TX_BUF_SIZE];
static uint8_t s_debug_justfloat_enabled;
static uint8_t s_debug_trace_view;
static uint8_t s_debug_live_view;
static uint32_t s_debug_last_telemetry_tick;
static uint8_t s_debug_justfloat_frame[(DEBUG_JUSTFLOAT_CH * sizeof(float)) + 4U];

typedef struct
{
    uint8_t active;
    uint8_t crossed;
    float command_rpm;
    float direction;
    float peak_speed_rpm;
    float peak_iq;
    float peak_iq_ref;
    float peak_id;
    float peak_uq;
    float peak_ud;
    float cross_speed_rpm;
    float cross_iq;
    float cross_iq_ref;
    float cross_id;
    float cross_uq;
    float cross_ud;
} Debug_SpeedTrace_t;

static Debug_SpeedTrace_t s_debug_speed_trace;

static float Debug_GetBusVoltage(void)
{
    const Current_State_t *current = Current_GetState();

    if (current->bus_v > 1.0f)
    {
        return current->bus_v;
    }

    return DEBUG_DEFAULT_BUS_VOLTAGE_V;
}

static void Debug_SendBusySafe(const char *text)
{
    uint16_t len = (uint16_t)strlen(text);

    if (len == 0U)
    {
        return;
    }

    (void)CDC_Transmit_FS((uint8_t *)text, len);
}

static void Debug_PrintHelp(void)
{
    Debug_SendBusySafe(
        "\r\ncmd:\r\n"
        "  set <uq> <freq_hz>\r\n"
        "  venc <uq_volt> <lead_deg>\r\n"
        "  stop\r\n"
        "  run\r\n"
        "  mode open|enc\r\n"
        "  torque <iq>\r\n"
        "  lock <angle_rad> <ud> <uq>\r\n"
        "  curr <id> <iq>\r\n"
        "  currpi <kp> <ki> <limit>\r\n"
        "  currbw <hz> [limit]\r\n"
        "  model <psi> <ld> <lq>\r\n"
        "  ff on|off\r\n"
        "  ffscale <0..1>\r\n"
        "  angleoff <deg>\r\n"
        "  vlead <deg>\r\n"
        "  speed <rpm> [kp ki max_iq max_i]\r\n"
        "  speedid <id>\r\n"
        "  view phase|curr|volt|ab\r\n"
        "  encview on|off\r\n"
        "  trace\r\n"
        "  pos <speed_cps> <kp> <max_uq>\r\n"
        "  jf on|off\r\n"
        "  status\r\n"
        "  help\r\n");
}

static void Debug_PrintStatus(void)
{
    const App_SpeedPIState_t *speed = App_GetSpeedPIState();
    App_RunMode_t mode = App_GetRunMode();
    const FocV_State_t *foc = FocV_GetState();
    const FOC_State_t *foc_current = FOC_GetState();
    const Current_State_t *current = Current_GetState();
    const Encoder_State_t *encoder = Encoder_GetState();

    (void)snprintf(s_debug_tx_buf, sizeof(s_debug_tx_buf),
                   "mode=%ld fault=%u ff=%u ffscale=%.2f angleoff=%.1f vlead=%.1f ud=%.4f uq=%.4f ua=%.4f ub=%.4f ia=%.3f ib=%.3f raw=%u/%u off=%.1f/%.1f id=%.3f/%.3f iq=%.3f/%.3f kp=%.4f ki=%.2f vlim=%.3f max_iq=%.3f max_i=%.3f model=%.5f/%.5f/%.5f uff=%.3f/%.3f bus=%.2f speed=%.1f/%.1f target=%.1f cmd=%.1f err=%.1f\r\n",
                   (long)mode,
                   App_IsFaulted(),
                   App_GetCurrentFeedforward(),
                   foc_current->feedforward_scale,
                   App_GetAngleOffsetDeg(),
                   App_GetVoltageLeadDeg(),
                   foc->ud,
                   foc->uq,
                   foc->ua,
                   foc->ub,
                   current->ia,
                   current->ib,
                   current->raw_a,
                   current->raw_b,
                   current->offset_a,
                   current->offset_b,
                   foc_current->id,
                   foc_current->id_ref,
                   foc_current->iq,
                   foc_current->iq_ref,
                   foc_current->kp,
                   foc_current->ki,
                   foc_current->voltage_limit,
                   speed->max_iq,
                   speed->max_integral_iq,
                   foc_current->psi_wb,
                   foc_current->ld_h,
                   foc_current->lq_h,
                   foc_current->uq_ff,
                   foc_current->ud_ff,
                   current->bus_v,
                   encoder->speed_raw_rpm,
                   encoder->speed_lpf_rpm,
                   speed->target_rpm,
                   speed->command_rpm,
                   speed->error_rpm);
    Debug_SendBusySafe(s_debug_tx_buf);
}

static void Debug_ProcessCommand(char *cmd)
{
    char *argv[6] = {0};
    uint8_t argc = 0U;
    char *token = strtok(cmd, " \t\r\n");

    while ((token != NULL) && (argc < 6U))
    {
        argv[argc] = token;
        argc++;
        token = strtok(NULL, " \t\r\n");
    }

    if (argc == 0U)
    {
        return;
    }

    if (strcmp(argv[0], "set") == 0)
    {
        if (argc < 3U)
        {
            Debug_SendBusySafe("err: set <uq> <freq_hz>\r\n");
            return;
        }

        if (s_debug_trace_view != DEBUG_TRACE_VIEW_ENCODER)
        {
            s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        }
        App_SetOpenLoop((float)atof(argv[1]), (float)atof(argv[2]));
    }
    else if (strcmp(argv[0], "stop") == 0)
    {
        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        App_ClearFault();
        App_ClearVoltageLock();
        App_ClearCurrentLoop();
        App_SetRunMode(APP_RUN_MODE_OPEN_LOOP);
        App_SetPositionP(0.0f, 0.0f, 0.0f);
        App_SetSpeedPI(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        App_SetOpenLoop(0.0f, 0.0f);
        FocV_Stop();
    }
    else if (strcmp(argv[0], "venc") == 0)
    {
        if (argc < 3U)
        {
            Debug_SendBusySafe("err: venc <uq_volt> <lead_deg>\r\n");
            return;
        }

        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        App_SetEncoderVoltage((float)atof(argv[1]), (float)atof(argv[2]));
    }
    else if (strcmp(argv[0], "run") == 0)
    {
        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        App_SetRunMode(APP_RUN_MODE_OPEN_LOOP);
        App_SetOpenLoop(0.04f, 0.5f);
    }
    else if (strcmp(argv[0], "mode") == 0)
    {
        if (argc < 2U)
        {
            Debug_SendBusySafe("err: mode open|enc\r\n");
            return;
        }

        if (strcmp(argv[1], "open") == 0)
        {
            s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
            App_SetRunMode(APP_RUN_MODE_OPEN_LOOP);
        }
        else if (strcmp(argv[1], "enc") == 0)
        {
            s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
            App_SetRunMode(APP_RUN_MODE_ENCODER_ANGLE);
        }
        else
        {
            Debug_SendBusySafe("err: mode open|enc\r\n");
        }
    }
    else if (strcmp(argv[0], "pos") == 0)
    {
        if (argc < 4U)
        {
            Debug_SendBusySafe("err: pos <speed_cps> <kp> <max_uq>\r\n");
            return;
        }

        App_SetPositionP((float)atof(argv[1]), (float)atof(argv[2]), (float)atof(argv[3]));
        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        App_SetRunMode(APP_RUN_MODE_POSITION_P);
    }
    else if (strcmp(argv[0], "torque") == 0)
    {
        if (argc < 2U)
        {
            Debug_SendBusySafe("err: torque <iq>\r\n");
            return;
        }

        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        App_SetCurrentLoop(0.0f, (float)atof(argv[1]));
    }
    else if (strcmp(argv[0], "lock") == 0)
    {
        if (argc < 4U)
        {
            Debug_SendBusySafe("err: lock <angle_rad> <ud> <uq>\r\n");
            return;
        }

        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        App_SetVoltageLock((float)atof(argv[1]),
                           (float)atof(argv[2]),
                           (float)atof(argv[3]));
    }
    else if (strcmp(argv[0], "curr") == 0)
    {
        if (argc < 2U)
        {
            Debug_SendBusySafe("err: curr <iq> | curr <id> <iq>\r\n");
            return;
        }

        if (argc >= 3U)
        {
            s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
            App_SetCurrentLoop((float)atof(argv[1]), (float)atof(argv[2]));
        }
        else
        {
            s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
            App_SetCurrentLoop(0.0f, (float)atof(argv[1]));
        }
    }
    else if (strcmp(argv[0], "currpi") == 0)
    {
        if (argc < 4U)
        {
            Debug_SendBusySafe("err: currpi <kp> <ki> <limit>\r\n");
            return;
        }

        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        App_SetCurrentLoopGains((float)atof(argv[1]),
                                (float)atof(argv[2]),
                                (float)atof(argv[3]));
    }
    else if (strcmp(argv[0], "currbw") == 0)
    {
        float voltage_limit = DEBUG_DEFAULT_CURRENT_VOLTAGE_LIMIT;

        if (argc < 2U)
        {
            Debug_SendBusySafe("err: currbw <hz> [limit]\r\n");
            return;
        }

        if (argc >= 3U)
        {
            voltage_limit = (float)atof(argv[2]);
        }

        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        App_SetCurrentLoopBandwidth((float)atof(argv[1]), voltage_limit);
    }
    else if (strcmp(argv[0], "model") == 0)
    {
        if (argc < 4U)
        {
            Debug_SendBusySafe("err: model <psi> <ld> <lq>\r\n");
            return;
        }

        FOC_SetMotorModel((float)atof(argv[1]),
                          (float)atof(argv[2]),
                          (float)atof(argv[3]));
        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        FOC_ResetCurrentLoop();
    }
    else if (strcmp(argv[0], "ff") == 0)
    {
        if (argc < 2U)
        {
            Debug_SendBusySafe("err: ff on|off\r\n");
            return;
        }

        if (strcmp(argv[1], "on") == 0)
        {
            App_SetCurrentFeedforward(1U);
        }
        else if (strcmp(argv[1], "off") == 0)
        {
            App_SetCurrentFeedforward(0U);
        }
        else
        {
            Debug_SendBusySafe("err: ff on|off\r\n");
            return;
        }

        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
    }
    else if (strcmp(argv[0], "ffscale") == 0)
    {
        if (argc < 2U)
        {
            Debug_SendBusySafe("err: ffscale <0..1>\r\n");
            return;
        }

        FOC_SetFeedforwardScale((float)atof(argv[1]));
        FOC_ResetCurrentLoop();
        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
    }
    else if (strcmp(argv[0], "angleoff") == 0)
    {
        if (argc < 2U)
        {
            Debug_SendBusySafe("err: angleoff <deg>\r\n");
            return;
        }

        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        App_SetAngleOffsetDeg((float)atof(argv[1]));
    }
    else if (strcmp(argv[0], "vlead") == 0)
    {
        if (argc < 2U)
        {
            Debug_SendBusySafe("err: vlead <deg>\r\n");
            return;
        }

        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        App_SetVoltageLeadDeg((float)atof(argv[1]));
    }
    else if (strcmp(argv[0], "speed") == 0)
    {
        float rpm;
        float kp = DEBUG_DEFAULT_SPEED_KP;
        float ki = DEBUG_DEFAULT_SPEED_KI;
        float max_iq = DEBUG_DEFAULT_SPEED_MAX_IQ;
        float max_integral_iq = DEBUG_DEFAULT_SPEED_MAX_INTEGRAL_IQ;

        if (argc < 2U)
        {
            Debug_SendBusySafe("err: speed <rpm> [kp ki max_iq max_i]\r\n");
            return;
        }

        if (argc >= 5U)
        {
            kp = (float)atof(argv[2]);
            ki = (float)atof(argv[3]);
            max_iq = (float)atof(argv[4]);
        }

        if (argc >= 6U)
        {
            max_integral_iq = (float)atof(argv[5]);
        }

        rpm = (float)atof(argv[1]);
        if (rpm > DEBUG_SPEED_CMD_LIMIT_RPM)
        {
            rpm = DEBUG_SPEED_CMD_LIMIT_RPM;
        }
        else if (rpm < -DEBUG_SPEED_CMD_LIMIT_RPM)
        {
            rpm = -DEBUG_SPEED_CMD_LIMIT_RPM;
        }

        (void)snprintf(s_debug_tx_buf, sizeof(s_debug_tx_buf),
                       "ok: speed rpm=%.1f kp=%.5f ki=%.5f max_iq=%.3f max_i=%.3f\r\n",
                       rpm,
                       kp,
                       ki,
                       max_iq,
                       max_integral_iq);
        Debug_SendBusySafe(s_debug_tx_buf);

        Debug_SpeedTraceReset(rpm);
        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        App_SetSpeedPI(rpm, kp, ki, max_iq, max_integral_iq);
        App_SetRunMode(APP_RUN_MODE_SPEED_PI);
    }
    else if (strcmp(argv[0], "speedid") == 0)
    {
        if (argc < 2U)
        {
            Debug_SendBusySafe("err: speedid <id>\r\n");
            return;
        }

        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        App_SetSpeedIdRef((float)atof(argv[1]));
    }
    else if (strcmp(argv[0], "view") == 0)
    {
        if (argc < 2U)
        {
            Debug_SendBusySafe("err: view phase|curr|volt|ab\r\n");
            return;
        }

        if (strcmp(argv[1], "phase") == 0)
        {
            s_debug_live_view = DEBUG_LIVE_VIEW_PHASE;
        }
        else if (strcmp(argv[1], "curr") == 0)
        {
            s_debug_live_view = DEBUG_LIVE_VIEW_CURR;
        }
        else if (strcmp(argv[1], "volt") == 0)
        {
            s_debug_live_view = DEBUG_LIVE_VIEW_VOLT;
        }
        else if (strcmp(argv[1], "ab") == 0)
        {
            s_debug_live_view = DEBUG_LIVE_VIEW_AB;
        }
        else
        {
            Debug_SendBusySafe("err: view phase|curr|volt|ab\r\n");
            return;
        }

        s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
    }
    else if (strcmp(argv[0], "encview") == 0)
    {
        if (argc < 2U)
        {
            Debug_SendBusySafe("err: encview on|off\r\n");
            return;
        }

        if (strcmp(argv[1], "on") == 0)
        {
            s_debug_trace_view = DEBUG_TRACE_VIEW_ENCODER;
        }
        else if (strcmp(argv[1], "off") == 0)
        {
            s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
        }
        else
        {
            Debug_SendBusySafe("err: encview on|off\r\n");
        }
    }
    else if (strcmp(argv[0], "trace") == 0)
    {
        if (s_debug_speed_trace.active == 0U)
        {
            Debug_SendBusySafe("trace: idle\r\n");
            return;
        }

        if (s_debug_trace_view == DEBUG_TRACE_VIEW_CROSS)
        {
            s_debug_trace_view = DEBUG_TRACE_VIEW_PEAK;
        }
        else
        {
            s_debug_trace_view = DEBUG_TRACE_VIEW_CROSS;
        }

        (void)snprintf(s_debug_tx_buf, sizeof(s_debug_tx_buf),
                       "trace view=%s cmd=%.1f peak_spd=%.1f peak_ref=%.3f peak_iq=%.3f peak_id=%.3f peak_uq=%.3f peak_ud=%.3f crossed=%u cross_spd=%.1f cross_ref=%.3f cross_iq=%.3f cross_id=%.3f cross_uq=%.3f cross_ud=%.3f\r\n",
                       (s_debug_trace_view == DEBUG_TRACE_VIEW_CROSS) ? "cross" : "peak",
                       s_debug_speed_trace.command_rpm,
                       s_debug_speed_trace.peak_speed_rpm,
                       s_debug_speed_trace.peak_iq_ref,
                       s_debug_speed_trace.peak_iq,
                       s_debug_speed_trace.peak_id,
                       s_debug_speed_trace.peak_uq,
                       s_debug_speed_trace.peak_ud,
                       s_debug_speed_trace.crossed,
                       s_debug_speed_trace.cross_speed_rpm,
                       s_debug_speed_trace.cross_iq_ref,
                       s_debug_speed_trace.cross_iq,
                       s_debug_speed_trace.cross_id,
                       s_debug_speed_trace.cross_uq,
                       s_debug_speed_trace.cross_ud);
        Debug_SendBusySafe(s_debug_tx_buf);
    }
    else if (strcmp(argv[0], "jf") == 0)
    {
        if (argc < 2U)
        {
            Debug_SendBusySafe("err: jf on|off\r\n");
            return;
        }

        if (strcmp(argv[1], "on") == 0)
        {
            s_debug_justfloat_enabled = 1U;
        }
        else if (strcmp(argv[1], "off") == 0)
        {
            s_debug_justfloat_enabled = 0U;
            Debug_SendBusySafe("ok: justfloat off\r\n");
        }
        else
        {
            Debug_SendBusySafe("err: jf on|off\r\n");
        }
    }
    else if (strcmp(argv[0], "status") == 0)
    {
        Debug_PrintStatus();
    }
    else if (strcmp(argv[0], "help") == 0)
    {
        Debug_PrintHelp();
    }
    else
    {
        Debug_SendBusySafe("err: unknown cmd\r\n");
        Debug_PrintHelp();
    }
}

void Debug_Init(void)
{
    s_debug_rx_len = 0U;
    s_debug_cmd_ready = 0U;
    s_debug_justfloat_enabled = 1U;
    s_debug_trace_view = DEBUG_TRACE_VIEW_LIVE;
    s_debug_live_view = DEBUG_LIVE_VIEW_PHASE;
    s_debug_last_telemetry_tick = HAL_GetTick();
    s_debug_speed_trace.active = 0U;
}

void Debug_SpeedTraceReset(float command_rpm)
{
    s_debug_speed_trace.active = 1U;
    s_debug_speed_trace.crossed = 0U;
    s_debug_speed_trace.command_rpm = command_rpm;
    s_debug_speed_trace.direction = (command_rpm < 0.0f) ? -1.0f : 1.0f;
    s_debug_speed_trace.peak_speed_rpm = 0.0f;
    s_debug_speed_trace.peak_iq = 0.0f;
    s_debug_speed_trace.peak_iq_ref = 0.0f;
    s_debug_speed_trace.peak_id = 0.0f;
    s_debug_speed_trace.peak_uq = 0.0f;
    s_debug_speed_trace.peak_ud = 0.0f;
    s_debug_speed_trace.cross_speed_rpm = 0.0f;
    s_debug_speed_trace.cross_iq = 0.0f;
    s_debug_speed_trace.cross_iq_ref = 0.0f;
    s_debug_speed_trace.cross_id = 0.0f;
    s_debug_speed_trace.cross_uq = 0.0f;
    s_debug_speed_trace.cross_ud = 0.0f;
}

void Debug_SpeedTraceSample(void)
{
    const Encoder_State_t *encoder;
    const FOC_State_t *foc_current;
    float bus_v;
    float dir_speed;
    float dir_command;

    if (s_debug_speed_trace.active == 0U)
    {
        return;
    }

    encoder = Encoder_GetState();
    foc_current = FOC_GetState();
    bus_v = Debug_GetBusVoltage();

    dir_speed = encoder->speed_lpf_rpm * s_debug_speed_trace.direction;
    dir_command = s_debug_speed_trace.command_rpm * s_debug_speed_trace.direction;

    if (dir_speed > (s_debug_speed_trace.peak_speed_rpm * s_debug_speed_trace.direction))
    {
        s_debug_speed_trace.peak_speed_rpm = encoder->speed_lpf_rpm;
        s_debug_speed_trace.peak_iq = foc_current->iq;
        s_debug_speed_trace.peak_iq_ref = foc_current->iq_ref;
        s_debug_speed_trace.peak_id = foc_current->id;
        s_debug_speed_trace.peak_uq = foc_current->uq * bus_v;
        s_debug_speed_trace.peak_ud = foc_current->ud * bus_v;
    }

    if ((s_debug_speed_trace.crossed == 0U) && (dir_speed >= dir_command))
    {
        s_debug_speed_trace.crossed = 1U;
        s_debug_speed_trace.cross_speed_rpm = encoder->speed_lpf_rpm;
        s_debug_speed_trace.cross_iq = foc_current->iq;
        s_debug_speed_trace.cross_iq_ref = foc_current->iq_ref;
        s_debug_speed_trace.cross_id = foc_current->id;
        s_debug_speed_trace.cross_uq = foc_current->uq * bus_v;
        s_debug_speed_trace.cross_ud = foc_current->ud * bus_v;
    }
}

void Debug_Loop(void)
{
    if (s_debug_cmd_ready == 0U)
    {
        return;
    }

    __disable_irq();
    strncpy(s_debug_cmd_buf, s_debug_rx_buf, sizeof(s_debug_cmd_buf) - 1U);
    s_debug_cmd_buf[sizeof(s_debug_cmd_buf) - 1U] = '\0';
    s_debug_cmd_ready = 0U;
    __enable_irq();

    Debug_ProcessCommand(s_debug_cmd_buf);
    s_debug_justfloat_enabled = 1U;
}

void Debug_TelemetryLoop(void)
{
    uint32_t now_tick = HAL_GetTick();
    float data[DEBUG_JUSTFLOAT_CH];
    const FOC_State_t *foc_current;
    const Current_State_t *current;
    const Encoder_State_t *encoder;
    const MotionCtrl_SpeedPIState_t *speed_pi;

    if (s_debug_justfloat_enabled == 0U)
    {
        return;
    }

    if ((now_tick - s_debug_last_telemetry_tick) < DEBUG_JUSTFLOAT_PERIOD)
    {
        return;
    }

    s_debug_last_telemetry_tick = now_tick;
    foc_current = FOC_GetState();
    current = Current_GetState();
    encoder = Encoder_GetState();
    speed_pi = MotionCtrl_GetSpeedPIState();

    if (s_debug_trace_view == DEBUG_TRACE_VIEW_CROSS)
    {
        data[0] = s_debug_speed_trace.cross_speed_rpm;
        data[1] = s_debug_speed_trace.cross_iq_ref;
        data[2] = s_debug_speed_trace.cross_iq;
        data[3] = s_debug_speed_trace.cross_id;
        data[4] = s_debug_speed_trace.cross_uq;
        data[5] = s_debug_speed_trace.cross_ud;
    }
    else if (s_debug_trace_view == DEBUG_TRACE_VIEW_PEAK)
    {
        data[0] = s_debug_speed_trace.peak_speed_rpm;
        data[1] = s_debug_speed_trace.peak_iq_ref;
        data[2] = s_debug_speed_trace.peak_iq;
        data[3] = s_debug_speed_trace.peak_id;
        data[4] = s_debug_speed_trace.peak_uq;
        data[5] = s_debug_speed_trace.peak_ud;
    }
    else if (s_debug_trace_view == DEBUG_TRACE_VIEW_ENCODER)
    {
        data[0] = (float)encoder->relative_count;
        data[1] = (float)encoder->count_in_rev;
        data[2] = encoder->mechanical_angle_deg;
        data[3] = encoder->electrical_angle_rad;
        data[4] = (float)(encoder->count_in_rev % 80);
        data[5] = encoder->speed_lpf_rpm;
    }
    else
    {
        float bus_v = Debug_GetBusVoltage();
        float phase_deg;

        if (s_debug_live_view == DEBUG_LIVE_VIEW_CURR)
        {
            data[0] = encoder->speed_lpf_rpm;
            data[1] = speed_pi->target_rpm;
            data[2] = foc_current->id;
            data[3] = foc_current->id_ref;
            data[4] = foc_current->iq;
            data[5] = foc_current->iq_ref;
        }
        else if (s_debug_live_view == DEBUG_LIVE_VIEW_VOLT)
        {
            data[0] = encoder->speed_lpf_rpm;
            data[1] = speed_pi->target_rpm;
            data[2] = foc_current->iq;
            data[3] = foc_current->iq_ref;
            data[4] = foc_current->uq * bus_v;
            data[5] = foc_current->ud * bus_v;
        }
        else if (s_debug_live_view == DEBUG_LIVE_VIEW_AB)
        {
            data[0] = encoder->speed_lpf_rpm;
            data[1] = speed_pi->target_rpm;
            data[2] = current->ia;
            data[3] = current->ib;
            data[4] = foc_current->id;
            data[5] = foc_current->iq;
        }
        else
        {
            phase_deg = atan2f(foc_current->id, foc_current->iq) * DEBUG_RAD_TO_DEG;

            data[0] = encoder->speed_lpf_rpm;
            data[1] = speed_pi->target_rpm;
            data[2] = foc_current->id;
            data[3] = foc_current->iq;
            data[4] = phase_deg;
            data[5] = foc_current->uq * bus_v;
        }
    }

    memcpy(&s_debug_justfloat_frame[0], data, sizeof(data));
    s_debug_justfloat_frame[sizeof(data) + 0U] = DEBUG_JUSTFLOAT_TAIL0;
    s_debug_justfloat_frame[sizeof(data) + 1U] = DEBUG_JUSTFLOAT_TAIL1;
    s_debug_justfloat_frame[sizeof(data) + 2U] = DEBUG_JUSTFLOAT_TAIL2;
    s_debug_justfloat_frame[sizeof(data) + 3U] = DEBUG_JUSTFLOAT_TAIL3;

    (void)CDC_Transmit_FS(s_debug_justfloat_frame, sizeof(s_debug_justfloat_frame));
}

void Debug_CdcRxCallback(const uint8_t *data, uint32_t len)
{
    uint32_t i;

    for (i = 0U; i < len; i++)
    {
        char ch = (char)data[i];

        if ((ch == '\r') || (ch == '\n'))
        {
            if (s_debug_rx_len > 0U)
            {
                s_debug_rx_buf[s_debug_rx_len] = '\0';
                s_debug_rx_len = 0U;
                s_debug_cmd_ready = 1U;
            }
        }
        else
        {
            if (s_debug_rx_len < (DEBUG_RX_BUF_SIZE - 1U))
            {
                s_debug_rx_buf[s_debug_rx_len] = ch;
                s_debug_rx_len++;
            }
            else
            {
                s_debug_rx_len = 0U;
            }
        }
    }
}

void Debug_Print(const char *text)
{
    Debug_SendBusySafe(text);
}
