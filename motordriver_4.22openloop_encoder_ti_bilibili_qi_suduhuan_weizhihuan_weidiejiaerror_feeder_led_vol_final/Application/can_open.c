#include "can_open.h"
#include "main.h"
#include "motion_trap2.h"
#include "encoder.h"
#include "feeder.h"
#include <string.h>

extern encoder_t enc;
extern motion_trap2_t g_motion_trap2;
extern feeder_ctrl_t g_feeder;

canopen_handle_t g_canopen;

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

void CANOpen_Init(CAN_HandleTypeDef *hcan, uint8_t node_id)
{
    memset(&g_canopen, 0, sizeof(g_canopen));

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

void CANOpen_Process(void)
{
    canopen_rx_frame_t frame;
    uint32_t now = HAL_GetTick();

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
    uint8_t data[8] = {0};
    HAL_StatusTypeDef ret;
    int16_t speed_rpm;
    uint16_t feed_index;
    uint16_t error_code = 0U;

    // status_word
    // bit0: ready
    // bit1: operational
    // bit2: fault
    // bit3: motion busy
    // bit4: motion done
    data[0] = 0x01U;

    if (g_canopen.state == CANOPEN_STATE_OPERATIONAL)
    {
        data[0] |= 0x02U;
    }

    if (MotionTrap2_IsBusy(&g_motion_trap2))
    {
        data[0] |= 0x08U;
    }

    if (MotionTrap2_IsDone(&g_motion_trap2))
    {
        data[0] |= 0x10U;
    }

    data[1] = (uint8_t)g_canopen.state;

    speed_rpm = (int16_t)(enc.vel_rpm_f);
    data[2] = (uint8_t)(speed_rpm & 0xFFU);
    data[3] = (uint8_t)((speed_rpm >> 8) & 0xFFU);

    feed_index = (uint16_t)g_feeder.feed_index;
    data[4] = (uint8_t)(feed_index & 0xFFU);
    data[5] = (uint8_t)((feed_index >> 8) & 0xFFU);

    data[6] = (uint8_t)(error_code & 0xFFU);
    data[7] = (uint8_t)((error_code >> 8) & 0xFFU);

    ret = CANOpen_SendStd((uint16_t)(CANOPEN_TPDO1_BASE_ID + g_canopen.node_id), data, 8U);
    if (ret == HAL_OK)
    {
        g_canopen.tpdo1_tx_count++;
    }

    return ret;
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
    int16_t speed_rpm;

    if (frame->header.DLC < 4U)
    {
        return;
    }

    speed_rpm = (int16_t)((uint16_t)frame->data[2] |
                         ((uint16_t)frame->data[3] << 8));

    g_canopen.control.command = frame->data[0];
    g_canopen.control.mode = frame->data[1];
    g_canopen.control.target_speed_rpm = speed_rpm;
    g_canopen.control.command_pending = 1U;
}

static void CANOpen_HandleFrame(const canopen_rx_frame_t *frame)
{
    uint16_t cob_id;
    uint16_t rpdo1_id;

    cob_id = frame->header.StdId;
    rpdo1_id = (uint16_t)(CANOPEN_RPDO1_BASE_ID + g_canopen.node_id);

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
}

static uint8_t CANOpen_NormalizeNodeId(uint8_t node_id)
{
    return (uint8_t)(node_id & 0x07U);
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
