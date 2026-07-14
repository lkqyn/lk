#ifndef __CAN_OPEN_H__
#define __CAN_OPEN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"
#include <stdint.h>

#define CANOPEN_RX_QUEUE_SIZE  8U
#define CANOPEN_HEARTBEAT_PERIOD_MS  1000U    //心跳周期1s
#define CANOPEN_RPDO1_BASE_ID       0x200U
#define CANOPEN_RPDO2_BASE_ID       0x300U
#define CANOPEN_RPDO3_BASE_ID       0x400U
#define CANOPEN_RPDO4_BASE_ID       0x500U
#define CANOPEN_TPDO1_BASE_ID       0x180U
#define CANOPEN_TPDO2_BASE_ID       0x280U
#define CANOPEN_TPDO3_BASE_ID       0x380U
#define CANOPEN_TPDO4_BASE_ID       0x480U
#define CANOPEN_SDO_RX_BASE_ID      0x600U
#define CANOPEN_SDO_TX_BASE_ID      0x580U
#define CANOPEN_TPDO1_PERIOD_MS     100U //每100ms发送一次
typedef enum
{
    CANOPEN_STATE_BOOTUP         = 0x00,
    CANOPEN_STATE_STOPPED        = 0x04,//停止
    CANOPEN_STATE_OPERATIONAL    = 0x05,//可操作
    CANOPEN_STATE_PREOP          = 0x7F,//预操作
    CANOPEN_STATE_RESET          = 0x80
} canopen_state_t;

typedef enum
{
    CANOPEN_CMD_STOP            = 0x00,
    CANOPEN_CMD_START           = 0x01,
    CANOPEN_CMD_ZERO            = 0x02,
    CANOPEN_CMD_FEED_ONCE       = 0x03,
    CANOPEN_CMD_AUTO_FEED_START = 0x04,
    CANOPEN_CMD_AUTO_FEED_STOP  = 0x05,
    CANOPEN_CMD_CLEAR_FAULT     = 0x06,
    CANOPEN_CMD_ALIGN           = 0x07
} canopen_command_t;

typedef enum
{
    CANOPEN_MODE_IDLE       = 0x00,
    CANOPEN_MODE_SPEED      = 0x01,
    CANOPEN_MODE_POS_TURNS  = 0x02,
    CANOPEN_MODE_TIME_TURNS = 0x03,
    CANOPEN_MODE_FEEDER     = 0x04,
    CANOPEN_MODE_POS_PULSES = 0x05
} canopen_mode_t;

typedef struct
{
    volatile uint8_t command_pending;
    volatile uint8_t command;
    volatile uint8_t mode;
    volatile int16_t target_speed_rpm;
    volatile int16_t value1;
    volatile uint16_t value2;
    volatile uint16_t value3;
    volatile int32_t value32;
} canopen_control_t;

typedef struct
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} canopen_rx_frame_t;

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint8_t node_id;
    canopen_state_t state;

    uint32_t last_heartbeat_tick;
    volatile uint32_t heartbeat_tx_count;

    uint32_t last_tpdo1_tick;
    volatile uint32_t tpdo1_tx_count;

    canopen_control_t control;

    canopen_rx_frame_t rx_queue[CANOPEN_RX_QUEUE_SIZE];
    volatile uint8_t rx_head;
    volatile uint8_t rx_tail;
    volatile uint8_t rx_count;

    volatile uint32_t rx_overflow_count;
    volatile uint32_t tx_ok_count;
    volatile uint32_t tx_err_count;
    volatile uint32_t rx_ok_count;
} canopen_handle_t;

typedef struct
{
    uint32_t device_type;                  // 0x1000
    uint8_t error_register;                // 0x1001

    uint32_t vendor_id;                    // 0x1018:1
    uint32_t product_code;                 // 0x1018:2
    uint32_t revision_number;              // 0x1018:3
    uint32_t serial_number;                // 0x1018:4

    uint16_t motor_resolution;             // 0x2001, pulses/rev
    uint16_t enable_valid_level;           // 0x2006
    uint16_t controlword;                  // 0x6040
    uint16_t statusword;                   // 0x6041
    int8_t modes_of_operation;             // 0x6060
    int8_t modes_of_operation_display;     // 0x6061
    int32_t position_actual_value;         // 0x6064
    int32_t velocity_actual_value;         // 0x606C
    int32_t target_position;               // 0x607A
    int32_t target_velocity;               // 0x60FF
    uint32_t profile_velocity;             // 0x6081
    uint32_t profile_acceleration;         // 0x6083
    uint32_t profile_deceleration;         // 0x6084
    uint16_t node_baudrate;                // 0x2151, 2 = 500K
} canopen_object_dictionary_t;

extern canopen_handle_t g_canopen;
extern canopen_object_dictionary_t g_canopen_od;
/**
 * @brief 初始化 CANopen 运行时对象并启动底层 CAN 外设。
 * @param hcan    指向底层 CAN 句柄的指针，通常传入 &hcan1
 * @param node_id 当前节点 ID，CANopen 常用范围为 1~127
 */
void CANOpen_Init(CAN_HandleTypeDef *hcan, uint8_t node_id);
void CANOpen_SetMotorResolution(uint32_t pulses_per_rev);
/**
 * @brief CANopen 后台处理函数，建议在主循环中周期调用。
 */
void CANOpen_Process(void);
/**
 * @brief 发送一帧标准格式 CAN 数据帧。

 * @param std_id 标准帧 ID，11-bit
 * @param data   指向待发送数据缓冲区的指针，可为 NULL
 * @param len    数据长度，范围 0~8
 *
 * @retval HAL_OK    发送请求成功进入发送邮箱
 * @retval HAL_ERROR 参数非法或发送失败
 * @retval 其他      HAL 底层返回的状态码
 */
HAL_StatusTypeDef CANOpen_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len);
/**
 * @brief 将一帧新收到的 CAN 报文交给 CANopen 模块。

 * @param header 指向接收帧头的指针
 * @param data   指向接收数据区的指针，长度由 header->DLC 指定
 */
HAL_StatusTypeDef CANOpen_SendHeartbeat(void);
void CANOpen_OnRxMessage(const CAN_RxHeaderTypeDef *header, const uint8_t *data);

HAL_StatusTypeDef CANOpen_SendTPDO1(void);
uint8_t CANOpen_GetCommand(canopen_control_t *out_cmd);
void CANOpen_ObjectDictionarySync(void);
#ifdef __cplusplus
}
#endif

#endif
