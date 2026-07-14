#ifndef __DIP_SWITCH_H__
#define __DIP_SWITCH_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum
{
    MICROSTEP_1   = 0,   // 000
    MICROSTEP_2   = 1,   // 001
    MICROSTEP_4   = 2,   // 010
    MICROSTEP_8   = 3,   // 011
    MICROSTEP_16  = 4,   // 100
    MICROSTEP_32  = 5,   // 101
    MICROSTEP_64  = 6,   // 110
    MICROSTEP_128 = 7    // 111
} MicroStep_TypeDef;

typedef enum
{
    DIR_FORWARD = 0,
    DIR_REVERSE = 1
} MotorDir_TypeDef;

typedef enum
{
    CTRL_MODE_CLOSED_LOOP = 0,
    CTRL_MODE_OPEN_LOOP   = 1
} CtrlMode_TypeDef;

typedef enum
{
    MOTOR_TYPE_STEP = 0,
    MOTOR_TYPE_BLDC = 1
} MotorType_TypeDef;

typedef enum
{
    TERM_RES_DISABLE = 0,
    TERM_RES_ENABLE  = 1
} TermRes_TypeDef;

typedef struct
{
    MicroStep_TypeDef microstep;   // 细分设置
    MotorDir_TypeDef  direction;   // 电机方向
    CtrlMode_TypeDef  ctrl_mode;   // 开环/闭环
    MotorType_TypeDef motor_type;  // 步进/无刷
    uint8_t           comm_addr;   // 通讯地址，当前硬件为 3bit: 0~7
    TermRes_TypeDef   term_res;    // 终端电阻使能
} DipSwitch_TypeDef;

/**
 * @brief 初始化拨码开关模块。
 *
 * 当前版本 GPIO 由 CubeMX 完成初始化，因此该函数通常无需额外配置。
 * 保留该接口是为了后续若需要加入软初始化、状态缓存或自检逻辑时，
 * 不必改动上层调用接口。
 */
void DipSwitch_Init(void);

/**
 * @brief 读取当前拨码开关状态并解析为结构化配置。
 *
 * 函数内部包含简单去抖处理，并将每个拨码位转换为业务含义：
 * - 细分
 * - 方向
 * - 控制模式
 * - 电机类型
 * - 通讯地址
 * - 终端电阻
 *
 * @param sw 指向拨码状态结构体的指针
 */
void DipSwitch_Read(DipSwitch_TypeDef *sw);

/**
 * @brief 通过 printf 打印当前拨码解析结果。
 *
 * 建议在上电调试阶段调用，用于确认硬件拨码和软件解析结果一致。
 *
 * @param sw 指向拨码状态结构体的指针
 */
void DipSwitch_Print(const DipSwitch_TypeDef *sw);

#endif /* __DIP_SWITCH_H__ */
