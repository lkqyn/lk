#include "can_open.h"
#include "main.h"
#include "motion_trap2.h"
#include "encoder.h"
#include <string.h>

extern encoder_t enc;
extern motion_trap2_t g_motion_trap2;
extern volatile int16_t canopen_last_motion_ret;
extern volatile uint8_t motor_fault_code;
extern volatile uint8_t motor_aligning;

canopen_handle_t g_canopen;
canopen_object_dictionary_t g_canopen_od;

#define CANOPEN_RESOLUTION_MIN            200U
#define CANOPEN_RESOLUTION_MAX            51200U

/**
 * @brief 初始化 CAN 接收过滤器。
 */
static void CANOpen_FilterInit(void);

/**
 * @brief 将收到的一帧 CAN 报文压入软件接收队列。
 */
static uint8_t CANOpen_RxQueuePush(const CAN_RxHeaderTypeDef *header, const uint8_t *data);

/**
 * @brief 从软件接收队列中取出一帧待处理报文。
 */
static uint8_t CANOpen_RxQueuePop(canopen_rx_frame_t *frame);

/**
 * @brief 解析并处理一帧已出队的 CAN 报文。
 */
static void CANOpen_HandleFrame(const canopen_rx_frame_t *frame);

/**
 * @brief 节点号限制为 0~7。
 */
static uint8_t CANOpen_NormalizeNodeId(uint8_t node_id);

/**
 * @brief 处理 RPDO1 控制报文。
 */
static void CANOpen_HandleRPDO1(const canopen_rx_frame_t *frame);
static void CANOpen_HandleRPDO2(const canopen_rx_frame_t *frame);
static void CANOpen_HandleRPDO3(const canopen_rx_frame_t *frame);
static void CANOpen_HandleRPDO4(const canopen_rx_frame_t *frame);
static void CANOpen_HandleSDO(const canopen_rx_frame_t *frame);
static void CANOpen_ObjectDictionaryInit(void);
static void CANOpen_StatuswordSync(void);
static void CANOpen_ApplyControlword(void);
static HAL_StatusTypeDef CANOpen_SendTPDO2(void);
static HAL_StatusTypeDef CANOpen_SendTPDO3(void);
static HAL_StatusTypeDef CANOpen_SendTPDO4(void);
static int32_t CANOpen_ReadI32(const uint8_t *data);
static uint16_t CANOpen_ReadU16(const uint8_t *data);
static void CANOpen_WriteU16(uint8_t *data, uint16_t value);
static void CANOpen_WriteI32(uint8_t *data, int32_t value);
static int16_t CANOpen_ClampI16(int32_t value);
static HAL_StatusTypeDef CANOpen_SendSDOAbort(uint16_t index, uint8_t subindex, uint32_t abort_code);

typedef struct
{
    uint16_t index;
    uint8_t subindex;
    void *data;
    uint8_t size;
    uint8_t writable;
} canopen_od_entry_t;

#define CANOPEN_ABORT_TOGGLE               0x05030000UL
#define CANOPEN_ABORT_TIMEOUT              0x05040000UL
#define CANOPEN_ABORT_CMD                  0x05040001UL
#define CANOPEN_ABORT_UNSUPPORTED          0x06010000UL
#define CANOPEN_ABORT_RO                   0x06010002UL
#define CANOPEN_ABORT_NOT_EXISTS           0x06020000UL
#define CANOPEN_ABORT_PDO_MAP              0x06040041UL
#define CANOPEN_ABORT_PARAM_INCOMPAT       0x06040043UL
#define CANOPEN_ABORT_HW                   0x06060000UL
#define CANOPEN_ABORT_TYPE_MISMATCH        0x06070010UL
#define CANOPEN_ABORT_DATA_SHORT           0x06070013UL
#define CANOPEN_ABORT_SUBINDEX             0x06090011UL
#define CANOPEN_ABORT_VALUE_RANGE          0x06090030UL

static uint16_t CANOpen_ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int32_t CANOpen_ReadI32(const uint8_t *data)
{
    uint32_t value;

    value = (uint32_t)data[0] |
            ((uint32_t)data[1] << 8) |
            ((uint32_t)data[2] << 16) |
            ((uint32_t)data[3] << 24);

    return (int32_t)value;
}

static void CANOpen_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void CANOpen_WriteI32(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)((raw >> 8) & 0xFFU);
    data[2] = (uint8_t)((raw >> 16) & 0xFFU);
    data[3] = (uint8_t)((raw >> 24) & 0xFFU);
}

static int16_t CANOpen_ClampI16(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }

    if (value < -32768)
    {
        return -32768;
    }

    return (int16_t)value;
}

static canopen_od_entry_t *CANOpen_FindOdEntry(uint16_t index, uint8_t subindex)
{
    static canopen_od_entry_t entries[] =
    {
        {0x1000, 0x00, &g_canopen_od.device_type,            4U, 0U},
        {0x1001, 0x00, &g_canopen_od.error_register,         1U, 0U},
        {0x1018, 0x00, NULL,                                 1U, 0U},
        {0x1018, 0x01, &g_canopen_od.vendor_id,              4U, 0U},
        {0x1018, 0x02, &g_canopen_od.product_code,           4U, 0U},
        {0x1018, 0x03, &g_canopen_od.revision_number,        4U, 0U},
        {0x1018, 0x04, &g_canopen_od.serial_number,          4U, 0U},
        {0x2001, 0x00, &g_canopen_od.motor_resolution,        2U, 1U},
        {0x2006, 0x00, &g_canopen_od.enable_valid_level,      2U, 1U},
        {0x2043, 0x00, &g_canopen_od.target_velocity,        4U, 0U},
        {0x2044, 0x00, &g_canopen_od.velocity_actual_value,  4U, 0U},
        {0x6040, 0x00, &g_canopen_od.controlword,            2U, 1U},
        {0x6041, 0x00, &g_canopen_od.statusword,             2U, 0U},
        {0x6060, 0x00, &g_canopen_od.modes_of_operation,     1U, 1U},
        {0x6061, 0x00, &g_canopen_od.modes_of_operation_display, 1U, 0U},
        {0x6063, 0x00, &g_canopen_od.position_actual_value,  4U, 0U},
        {0x6064, 0x00, &g_canopen_od.position_actual_value,  4U, 0U},
        {0x606C, 0x00, &g_canopen_od.velocity_actual_value,  4U, 0U},
        {0x607A, 0x00, &g_canopen_od.target_position,        4U, 1U},
        {0x6081, 0x00, &g_canopen_od.profile_velocity,       4U, 1U},
        {0x6083, 0x00, &g_canopen_od.profile_acceleration,   4U, 1U},
        {0x6084, 0x00, &g_canopen_od.profile_deceleration,   4U, 1U},
        {0x60FF, 0x00, &g_canopen_od.target_velocity,        4U, 1U},
        {0x5000, 0x00, (void *)&canopen_last_motion_ret,      2U, 0U},
        {0x2151, 0x00, &g_canopen_od.node_baudrate,           2U, 1U}
    };
    uint32_t i;

    if ((index == 0x1018U) && (subindex == 0x00U))
    {
        static uint8_t identity_entries = 4U;
        entries[2].data = &identity_entries;
        return &entries[2];
    }

    for (i = 0U; i < (sizeof(entries) / sizeof(entries[0])); i++)
    {
        if ((entries[i].index == index) && (entries[i].subindex == subindex))
        {
            return &entries[i];
        }
    }

    return NULL;
}

void CANOpen_Init(CAN_HandleTypeDef *hcan, uint8_t node_id)
{
    memset(&g_canopen, 0, sizeof(g_canopen));
    CANOpen_ObjectDictionaryInit();

    g_canopen.hcan = hcan;
    g_canopen.node_id = CANOpen_NormalizeNodeId(node_id);
    g_canopen.state = CANOPEN_STATE_PREOP;
    g_canopen.last_heartbeat_tick = HAL_GetTick();
    g_canopen.last_tpdo1_tick = HAL_GetTick();

    CANOpen_FilterInit();

    if (HAL_CAN_Start(g_canopen.hcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(g_canopen.hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

    // Boot-up message
    g_canopen.state = CANOPEN_STATE_BOOTUP;
    CANOpen_SendHeartbeat();

    // Enter pre-operational after boot-up
    g_canopen.state = CANOPEN_STATE_PREOP;
    g_canopen.last_heartbeat_tick = HAL_GetTick();
    g_canopen.last_tpdo1_tick = HAL_GetTick();
}

static void CANOpen_ObjectDictionaryInit(void)
{
    memset(&g_canopen_od, 0, sizeof(g_canopen_od));

    g_canopen_od.device_type = 0x00040192UL;
    g_canopen_od.vendor_id = 0x00000331UL;
    g_canopen_od.product_code = 0x00008100UL;
    g_canopen_od.revision_number = 0x00000100UL;
    g_canopen_od.serial_number = 0x00000000UL;

    g_canopen_od.motor_resolution = CANOPEN_RESOLUTION_MIN;
    g_canopen_od.enable_valid_level = 1U;
    g_canopen_od.controlword = 0x0000U;
    g_canopen_od.statusword = 0x0040U;
    g_canopen_od.modes_of_operation = 3;
    g_canopen_od.modes_of_operation_display = 3;
    g_canopen_od.target_position = 0;
    g_canopen_od.target_velocity = 0;
    g_canopen_od.profile_velocity = 300U;
    g_canopen_od.profile_acceleration = 3000U;
    g_canopen_od.profile_deceleration = 3000U;
    g_canopen_od.node_baudrate = 2U;
    g_canopen.control.value32 = 0;
}

void CANOpen_ObjectDictionarySync(void)
{
    g_canopen_od.error_register = (motor_fault_code != 0U) ? 0x01U : 0x00U;
    g_canopen_od.modes_of_operation_display = g_canopen_od.modes_of_operation;
    g_canopen_od.position_actual_value = (int32_t)(enc.pos_rev * (float)g_canopen_od.motor_resolution);
    g_canopen_od.velocity_actual_value = (int32_t)(enc.vel_rpm_f);
    CANOpen_StatuswordSync();
}

static void CANOpen_StatuswordSync(void)
{
    uint16_t sw = 0U;
    uint8_t operation_enabled = 0U;

    if ((g_canopen_od.controlword & 0x000FU) == 0x000FU)
    {
        operation_enabled = 1U;
    }

    if (motor_fault_code != 0U)
    {
        sw |= (1U << 3);   // Fault
    }
    else
    {
        sw |= (1U << 0);   // Ready to switch on
        sw |= (1U << 1);   // Switched on
        if (operation_enabled && (g_canopen.state == CANOPEN_STATE_OPERATIONAL))
        {
            sw |= (1U << 2);   // Operation enabled
        }
        else
        {
            sw |= (1U << 6);   // Switch on disabled / not fully enabled
        }
    }

    sw |= (1U << 5);           // Quick stop bit set (not in quick stop)
    sw |= (1U << 9);           // Remote

    if (!MotionTrap2_IsBusy(&g_motion_trap2) && !motor_aligning)
    {
        sw |= (1U << 10);      // Target reached
    }

    if (motor_aligning)
    {
        sw &= (uint16_t)~(1U << 10);
    }

    g_canopen_od.statusword = sw;
}

static void CANOpen_ApplyControlword(void)
{
    g_canopen_od.modes_of_operation_display = g_canopen_od.modes_of_operation;

    if ((g_canopen_od.controlword & 0x0080U) != 0U)
    {
        g_canopen.control.command = CANOPEN_CMD_CLEAR_FAULT;
        g_canopen.control.mode = CANOPEN_MODE_IDLE;
        g_canopen.control.target_speed_rpm = 0;
        g_canopen.control.value1 = 0;
        g_canopen.control.value2 = 0U;
        g_canopen.control.value3 = 0U;
        g_canopen.control.command_pending = 1U;
        return;
    }

    if (((g_canopen_od.controlword & 0x0004U) == 0U) ||
        ((g_canopen_od.controlword & 0x000FU) != 0x000FU))
    {
        g_canopen.control.command = CANOPEN_CMD_STOP;
        g_canopen.control.mode = CANOPEN_MODE_IDLE;
        g_canopen.control.target_speed_rpm = 0;
        g_canopen.control.value1 = 0;
        g_canopen.control.value2 = 0U;
        g_canopen.control.value3 = 0U;
        g_canopen.control.command_pending = 1U;
        return;
    }

    if ((g_canopen_od.controlword & 0x000FU) == 0x000FU)
    {
        if (g_canopen_od.modes_of_operation == 3)
        {
            g_canopen.control.command = CANOPEN_CMD_START;
            g_canopen.control.mode = CANOPEN_MODE_SPEED;
            g_canopen.control.target_speed_rpm = (int16_t)g_canopen_od.target_velocity;
            g_canopen.control.value1 = (int16_t)g_canopen_od.target_velocity;
            g_canopen.control.value2 = 0U;
            g_canopen.control.value3 = 0U;
            g_canopen.control.command_pending = 1U;
        }
        else if (g_canopen_od.modes_of_operation == 1)
        {
            int32_t centi_turns = 0;
            uint16_t position_cmd = (uint16_t)(g_canopen_od.controlword & 0x00F0U);
            uint8_t position_start =
                ((g_canopen_od.controlword & 0x0010U) != 0U) ||
                (position_cmd == 0x0020U) ||
                (position_cmd == 0x0030U) ||
                (position_cmd == 0x0040U) ||
                (position_cmd == 0x0050U) ||
                ((g_canopen_od.controlword & 0x103FU) == 0x103FU);

            if (!position_start)
            {
                return;
            }

            if (g_canopen_od.motor_resolution != 0U)
            {
                centi_turns = (g_canopen_od.target_position * 100) /
                              (int32_t)g_canopen_od.motor_resolution;
            }

            g_canopen.control.command = CANOPEN_CMD_START;
            g_canopen.control.mode = CANOPEN_MODE_POS_PULSES;
            g_canopen.control.value1 = CANOpen_ClampI16(centi_turns);
            g_canopen.control.value2 = (uint16_t)g_canopen_od.profile_velocity;
            g_canopen.control.value3 = (uint16_t)g_canopen_od.profile_acceleration;
            g_canopen.control.value32 = g_canopen_od.target_position;
            g_canopen.control.target_speed_rpm = 0;
            g_canopen.control.command_pending = 1U;
        }
    }
}

void CANOpen_Process(void)
{
    canopen_rx_frame_t frame;
    uint32_t now = HAL_GetTick();

    CANOpen_ObjectDictionarySync();

    while (CANOpen_RxQueuePop(&frame))
    {
        CANOpen_HandleFrame(&frame);
    }

    if ((now - g_canopen.last_heartbeat_tick) >= CANOPEN_HEARTBEAT_PERIOD_MS)
    {
        g_canopen.last_heartbeat_tick = now;
        CANOpen_SendHeartbeat();
    }

    if ((now - g_canopen.last_tpdo1_tick) >= CANOPEN_TPDO1_PERIOD_MS)
    {
        g_canopen.last_tpdo1_tick = now;
        CANOpen_SendTPDO1();
        CANOpen_SendTPDO2();
        CANOpen_SendTPDO4();
    }
}

HAL_StatusTypeDef CANOpen_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;
    uint8_t tx_buf[8] = {0};
    HAL_StatusTypeDef ret;

    if ((g_canopen.hcan == NULL) || (len > 8U))
    {
        g_canopen.tx_err_count++;
        return HAL_ERROR;
    }

    tx_header.StdId = std_id & 0x7FFU;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    if ((data != NULL) && (len > 0U))
    {
        memcpy(tx_buf, data, len);
    }

    ret = HAL_CAN_AddTxMessage(g_canopen.hcan, &tx_header, tx_buf, &mailbox);
    if (ret == HAL_OK)
    {
        g_canopen.tx_ok_count++;
    }
    else
    {
        g_canopen.tx_err_count++;
    }

    return ret;
}

HAL_StatusTypeDef CANOpen_SendHeartbeat(void)
{
    uint8_t data[1];
    HAL_StatusTypeDef ret;

    data[0] = (uint8_t)g_canopen.state;

    ret = CANOpen_SendStd((uint16_t)(0x700U + g_canopen.node_id), data, 1U);
    if (ret == HAL_OK)
    {
        g_canopen.heartbeat_tx_count++;
    }

    return ret;
}

HAL_StatusTypeDef CANOpen_SendTPDO1(void)
{
    uint8_t data[2] = {0};
    HAL_StatusTypeDef ret;

    CANOpen_ObjectDictionarySync();
    CANOpen_WriteU16(&data[0], g_canopen_od.statusword);

    ret = CANOpen_SendStd((uint16_t)(CANOPEN_TPDO1_BASE_ID + g_canopen.node_id), data, 2U);
    if (ret == HAL_OK)
    {
        g_canopen.tpdo1_tx_count++;
    }

    return ret;
}

static HAL_StatusTypeDef CANOpen_SendTPDO2(void)
{
    uint8_t data[6] = {0};

    CANOpen_ObjectDictionarySync();
    CANOpen_WriteU16(&data[0], g_canopen_od.statusword);
    CANOpen_WriteI32(&data[2], g_canopen_od.position_actual_value);

    return CANOpen_SendStd((uint16_t)(CANOPEN_TPDO2_BASE_ID + g_canopen.node_id), data, 6U);
}

static HAL_StatusTypeDef CANOpen_SendTPDO3(void)
{
    uint8_t data[6] = {0};

    CANOpen_ObjectDictionarySync();
    CANOpen_WriteU16(&data[0], g_canopen_od.statusword);
    CANOpen_WriteI32(&data[2], g_canopen_od.position_actual_value);

    return CANOpen_SendStd((uint16_t)(CANOPEN_TPDO3_BASE_ID + g_canopen.node_id), data, 6U);
}

static HAL_StatusTypeDef CANOpen_SendTPDO4(void)
{
    uint8_t data[6] = {0};

    CANOpen_ObjectDictionarySync();
    CANOpen_WriteU16(&data[0], g_canopen_od.statusword);
    CANOpen_WriteI32(&data[2], g_canopen_od.velocity_actual_value);

    return CANOpen_SendStd((uint16_t)(CANOPEN_TPDO4_BASE_ID + g_canopen.node_id), data, 6U);
}

void CANOpen_OnRxMessage(const CAN_RxHeaderTypeDef *header, const uint8_t *data)
{
    if ((header == NULL) || (data == NULL))
    {
        return;
    }

    if (!CANOpen_RxQueuePush(header, data))
    {
        g_canopen.rx_overflow_count++;
    }
}

uint8_t CANOpen_GetCommand(canopen_control_t *out_cmd)
{
    if ((out_cmd == NULL) || (g_canopen.control.command_pending == 0U))
    {
        return 0U;
    }

    *out_cmd = g_canopen.control;
    g_canopen.control.command_pending = 0U;

    return 1U;
}

static void CANOpen_FilterInit(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(g_canopen.hcan, &filter) != HAL_OK)
    {
        Error_Handler();
    }
}

static uint8_t CANOpen_RxQueuePush(const CAN_RxHeaderTypeDef *header, const uint8_t *data)
{
    uint8_t idx;
    uint8_t len;

    if (g_canopen.rx_count >= CANOPEN_RX_QUEUE_SIZE)
    {
        return 0U;
    }

    idx = g_canopen.rx_head;
    g_canopen.rx_queue[idx].header = *header;

    len = header->DLC;
    if (len > 8U)
    {
        len = 8U;
    }

    memcpy(g_canopen.rx_queue[idx].data, data, len);

    g_canopen.rx_head = (uint8_t)((g_canopen.rx_head + 1U) % CANOPEN_RX_QUEUE_SIZE);
    g_canopen.rx_count++;
    g_canopen.rx_ok_count++;

    return 1U;
}

static uint8_t CANOpen_RxQueuePop(canopen_rx_frame_t *frame)
{
    uint8_t idx;

    if ((frame == NULL) || (g_canopen.rx_count == 0U))
    {
        return 0U;
    }

    idx = g_canopen.rx_tail;
    *frame = g_canopen.rx_queue[idx];

    g_canopen.rx_tail = (uint8_t)((g_canopen.rx_tail + 1U) % CANOPEN_RX_QUEUE_SIZE);
    g_canopen.rx_count--;

    return 1U;
}

static void CANOpen_HandleRPDO1(const canopen_rx_frame_t *frame)
{
    if (frame->header.DLC < 2U)
    {
        return;
    }

    g_canopen_od.controlword = CANOpen_ReadU16(&frame->data[0]);
    CANOpen_ApplyControlword();
}

static void CANOpen_HandleRPDO2(const canopen_rx_frame_t *frame)
{
    if (frame->header.DLC < 6U)
    {
        return;
    }

    g_canopen_od.controlword = CANOpen_ReadU16(&frame->data[0]);
    g_canopen_od.target_position = CANOpen_ReadI32(&frame->data[2]);
    CANOpen_ApplyControlword();
}

static void CANOpen_HandleRPDO3(const canopen_rx_frame_t *frame)
{
    if (frame->header.DLC < 6U)
    {
        return;
    }

    g_canopen_od.controlword = CANOpen_ReadU16(&frame->data[0]);
    g_canopen_od.target_position = CANOpen_ReadI32(&frame->data[2]);
    CANOpen_ApplyControlword();
}

static void CANOpen_HandleRPDO4(const canopen_rx_frame_t *frame)
{
    if (frame->header.DLC < 6U)
    {
        return;
    }

    g_canopen_od.controlword = CANOpen_ReadU16(&frame->data[0]);
    g_canopen_od.target_velocity = CANOpen_ReadI32(&frame->data[2]);
    CANOpen_ApplyControlword();
}

static HAL_StatusTypeDef CANOpen_SendSDOAbort(uint16_t index, uint8_t subindex, uint32_t abort_code)
{
    uint8_t data[8];

    data[0] = 0x80U;
    data[1] = (uint8_t)(index & 0xFFU);
    data[2] = (uint8_t)((index >> 8) & 0xFFU);
    data[3] = subindex;
    data[4] = (uint8_t)(abort_code & 0xFFU);
    data[5] = (uint8_t)((abort_code >> 8) & 0xFFU);
    data[6] = (uint8_t)((abort_code >> 16) & 0xFFU);
    data[7] = (uint8_t)((abort_code >> 24) & 0xFFU);

    return CANOpen_SendStd((uint16_t)(CANOPEN_SDO_TX_BASE_ID + g_canopen.node_id), data, 8U);
}

static void CANOpen_HandleSDO(const canopen_rx_frame_t *frame)
{
    uint8_t cmd;
    uint16_t index;
    uint8_t subindex;
    canopen_od_entry_t *entry;
    uint8_t resp[8] = {0};
    uint8_t size_indicated;

    if (frame->header.DLC < 8U)
    {
        return;
    }

    CANOpen_ObjectDictionarySync();

    cmd = frame->data[0];
    index = (uint16_t)((uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8));
    subindex = frame->data[3];
    entry = CANOpen_FindOdEntry(index, subindex);

    if ((cmd == 0x40U) && (index == 0x1000U) && (subindex == 0x00U))
    {
        resp[0] = 0x43U;
        resp[1] = 0x00U;
        resp[2] = 0x10U;
        resp[3] = 0x00U;
        resp[4] = 0x92U;
        resp[5] = 0x01U;
        resp[6] = 0x04U;
        resp[7] = 0x00U;
        (void)CANOpen_SendStd((uint16_t)(CANOPEN_SDO_TX_BASE_ID + g_canopen.node_id), resp, 8U);
        return;
    }

    if ((cmd & 0xE0U) == 0x40U)
    {
        if (entry == NULL)
        {
            (void)CANOpen_SendSDOAbort(index, subindex, CANOPEN_ABORT_NOT_EXISTS);
            return;
        }

        resp[0] = (uint8_t)(0x43U | ((4U - entry->size) << 2));
        resp[1] = frame->data[1];
        resp[2] = frame->data[2];
        resp[3] = subindex;
        memcpy(&resp[4], entry->data, entry->size);
        (void)CANOpen_SendStd((uint16_t)(CANOPEN_SDO_TX_BASE_ID + g_canopen.node_id), resp, 8U);
        return;
    }

    if ((cmd & 0xE0U) == 0x20U)
    {
        if (entry == NULL)
        {
            (void)CANOpen_SendSDOAbort(index, subindex, CANOPEN_ABORT_NOT_EXISTS);
            return;
        }

        if (!entry->writable)
        {
            (void)CANOpen_SendSDOAbort(index, subindex, CANOPEN_ABORT_RO);
            return;
        }

        if ((cmd & 0x02U) == 0U)
        {
            (void)CANOpen_SendSDOAbort(index, subindex, CANOPEN_ABORT_CMD);
            return;
        }

        size_indicated = (uint8_t)(4U - ((cmd >> 2) & 0x03U));
        if (size_indicated != entry->size)
        {
            (void)CANOpen_SendSDOAbort(index, subindex, CANOPEN_ABORT_TYPE_MISMATCH);
            return;
        }

        if ((index == 0x6060U) && (subindex == 0x00U))
        {
            int8_t mode = (int8_t)frame->data[4];
            if ((mode != 1) &&
                (mode != 3) &&
                (mode != 6))
            {
                (void)CANOpen_SendSDOAbort(index, subindex, CANOPEN_ABORT_VALUE_RANGE);
                return;
            }
        }

        memcpy(entry->data, &frame->data[4], entry->size);

        if (((index == 0x6040U) ||
             (index == 0x6060U) ||
             (index == 0x607AU) ||
             (index == 0x6081U) ||
             (index == 0x6083U) ||
             (index == 0x6084U) ||
             (index == 0x60FFU)) &&
            (subindex == 0x00U))
        {
            CANOpen_ApplyControlword();
        }

        resp[0] = 0x60U;
        resp[1] = frame->data[1];
        resp[2] = frame->data[2];
        resp[3] = subindex;
        (void)CANOpen_SendStd((uint16_t)(CANOPEN_SDO_TX_BASE_ID + g_canopen.node_id), resp, 8U);
        return;
    }

    (void)CANOpen_SendSDOAbort(index, subindex, CANOPEN_ABORT_CMD);
}

static void CANOpen_HandleFrame(const canopen_rx_frame_t *frame)
{
    uint16_t cob_id;
    uint16_t rpdo1_id;
    uint16_t rpdo2_id;
    uint16_t rpdo3_id;
    uint16_t rpdo4_id;
    uint16_t sdo_rx_id;

    cob_id = frame->header.StdId;
    rpdo1_id = (uint16_t)(CANOPEN_RPDO1_BASE_ID + g_canopen.node_id);
    rpdo2_id = (uint16_t)(CANOPEN_RPDO2_BASE_ID + g_canopen.node_id);
    rpdo3_id = (uint16_t)(CANOPEN_RPDO3_BASE_ID + g_canopen.node_id);
    rpdo4_id = (uint16_t)(CANOPEN_RPDO4_BASE_ID + g_canopen.node_id);
    sdo_rx_id = (uint16_t)(CANOPEN_SDO_RX_BASE_ID + g_canopen.node_id);

    if (cob_id == 0x000U)
    {
        if ((frame->data[1] == 0U) || (frame->data[1] == g_canopen.node_id))
        {
            switch (frame->data[0])
            {
            case 0x01:
                g_canopen.state = CANOPEN_STATE_OPERATIONAL;
                break;

            case 0x02:
                g_canopen.state = CANOPEN_STATE_STOPPED;
                MotionTrap2_Abort(&g_motion_trap2);
                break;

            case 0x80:
                g_canopen.state = CANOPEN_STATE_PREOP;
                MotionTrap2_Abort(&g_motion_trap2);
                break;

            case 0x81:
                MotionTrap2_Abort(&g_motion_trap2);
                g_canopen.state = CANOPEN_STATE_BOOTUP;
                CANOpen_SendHeartbeat();
                g_canopen.state = CANOPEN_STATE_PREOP;
                break;

            default:
                break;
            }
        }

        return;
    }

    if (cob_id == rpdo1_id)
    {
        CANOpen_HandleRPDO1(frame);
        return;
    }

    if (cob_id == rpdo2_id)
    {
        CANOpen_HandleRPDO2(frame);
        return;
    }

    if (cob_id == rpdo3_id)
    {
        CANOpen_HandleRPDO3(frame);
        return;
    }

    if (cob_id == rpdo4_id)
    {
        CANOpen_HandleRPDO4(frame);
        return;
    }

    if (cob_id == sdo_rx_id)
    {
        CANOpen_HandleSDO(frame);
        return;
    }
}

static uint8_t CANOpen_NormalizeNodeId(uint8_t node_id)
{
    return (uint8_t)(node_id & 0x07U);
}

void CANOpen_SetMotorResolution(uint32_t pulses_per_rev)
{
    if (pulses_per_rev < CANOPEN_RESOLUTION_MIN)
    {
        pulses_per_rev = CANOPEN_RESOLUTION_MIN;
    }
    else if (pulses_per_rev > CANOPEN_RESOLUTION_MAX)
    {
        pulses_per_rev = CANOPEN_RESOLUTION_MAX;
    }

    g_canopen_od.motor_resolution = (uint16_t)pulses_per_rev;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan->Instance != CAN1)
    {
        return;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        CANOpen_OnRxMessage(&rx_header, rx_data);
    }
}
