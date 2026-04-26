#ifndef __DIP_SWITCH_H
#define __DIP_SWITCH_H

#include "stm32f4xx_hal.h"

// 细分模式（0-7 对应 1/2/4/8/16/32/64/128细分，匹配拨码3位）
typedef enum
{
    MICROSTEP_1     = 0,    // 全步（1细分）
    MICROSTEP_2     = 1,    // 2细分
    MICROSTEP_4     = 2,    // 4细分
    MICROSTEP_8     = 3,    // 8细分（默认）
    MICROSTEP_16    = 4,    // 16细分
    MICROSTEP_32    = 5,    // 32细分
    MICROSTEP_64    = 6,    // 64细分
    MICROSTEP_128   = 7     // 128细分
} MicroStep_TypeDef;
// 旋转方向
typedef enum
{
    DIR_FORWARD     = 0,    // 正向
    DIR_REVERSE     = 1     // 反向
} MotorDir_TypeDef;

// 控制模式
typedef enum
{
    CTRL_MODE_CLOSED_LOOP = 0,  // 闭环
    CTRL_MODE_OPEN_LOOP   = 1   // 开环
} CtrlMode_TypeDef;

// 电机类型
typedef enum
{
    MOTOR_TYPE_STEP   = 0,  // 步进电机
    MOTOR_TYPE_BLDC   = 1   // 无刷电机
} MotorType_TypeDef;

// 终端电阻120欧
typedef enum
{
    TERM_RES_DISABLE = 0,  // 禁用
    TERM_RES_ENABLE  = 1   // 启用
} TermRes_TypeDef;

//拨码开关结构体
typedef struct
{
    MicroStep_TypeDef   microstep;    // 细分（枚举类型，仅能取0-7）
    MotorDir_TypeDef    direction;    // 旋转方向（枚举类型，仅能取0/1）
    CtrlMode_TypeDef    ctrl_mode;    // 控制模式（枚举类型，仅能取0/1）
    MotorType_TypeDef   motor_type;   // 电机类型（枚举类型，仅能取0/1）
    uint8_t             comm_addr;    // 通讯地址（数值型，0-15，无枚举）
    TermRes_TypeDef     term_res;     // 终端电阻（枚举类型，仅能取0/1）
} DipSwitch_TypeDef;

// 函数声明
void DipSwitch_Init(void);
void DipSwitch_Read(DipSwitch_TypeDef *sw);
void DipSwitch_Print(DipSwitch_TypeDef *sw);
#endif
