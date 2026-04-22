#include "dip_switch.h"
#include "main.h"
#include <stdio.h>

/**
 * @brief 读取单个 GPIO 输入并做一次简单去抖。
 *
 * 先读取一次电平，延时 10ms 后再次读取；
 * 若两次结果一致，则认为本次采样有效。
 *
 * @param GPIOx    GPIO 端口
 * @param GPIO_Pin GPIO 引脚
 *
 * @retval 稳定后的引脚电平
 */
static uint8_t GPIO_ReadPin_Debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint8_t val1 = (uint8_t)HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
    HAL_Delay(10);
    uint8_t val2 = (uint8_t)HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

    return (val1 == val2) ? val1 : val2;
}

void DipSwitch_Init(void)
{
    // GPIO 已由 CubeMX 初始化，这里暂不需要额外处理
}

void DipSwitch_Read(DipSwitch_TypeDef *sw)
{
    uint8_t sw_val = 0;

    if (sw == NULL)
    {
        return;
    }

    // -------------------------------------------------
    // 细分控制
    // SW_MICROSTEP_0 -> bit0 -> 最低位 -> PB11
    // SW_MICROSTEP_1 -> bit1 -> 次高位 -> PE7
    // SW_MICROSTEP_2 -> bit2 -> 最高位 -> PE8
    // -------------------------------------------------
    sw_val = 0;
    if (GPIO_ReadPin_Debounce(SW_MICROSTEP_0_GPIO_Port, SW_MICROSTEP_0_Pin) == GPIO_PIN_RESET) sw_val |= 0x01;
    if (GPIO_ReadPin_Debounce(SW_MICROSTEP_1_GPIO_Port, SW_MICROSTEP_1_Pin) == GPIO_PIN_RESET) sw_val |= 0x02;
    if (GPIO_ReadPin_Debounce(SW_MICROSTEP_2_GPIO_Port, SW_MICROSTEP_2_Pin) == GPIO_PIN_RESET) sw_val |= 0x04;
    sw->microstep = (MicroStep_TypeDef)(sw_val & 0x07U);

    // 旋转方向
    sw->direction =
        (GPIO_ReadPin_Debounce(SW_DIR_GPIO_Port, SW_DIR_Pin) == GPIO_PIN_RESET) ?
        DIR_REVERSE : DIR_FORWARD;

    // 控制模式
    sw->ctrl_mode =
        (GPIO_ReadPin_Debounce(SW_CTRL_MODE_GPIO_Port, SW_CTRL_MODE_Pin) == GPIO_PIN_RESET) ?
        CTRL_MODE_OPEN_LOOP : CTRL_MODE_CLOSED_LOOP;

    // 电机类型
    sw->motor_type =
        (GPIO_ReadPin_Debounce(SW_MOTOR_TYPE_GPIO_Port, SW_MOTOR_TYPE_Pin) == GPIO_PIN_RESET) ?
        MOTOR_TYPE_BLDC : MOTOR_TYPE_STEP;

    // -------------------------------------------------
    // 通讯地址
    // SW_ADDR_0 -> bit0 -> PE12
    // SW_ADDR_1 -> bit1 -> PE13
    // SW_ADDR_2 -> bit2 -> PE14
    // 当前硬件地址范围：0~7
    // -------------------------------------------------
    sw_val = 0;
    if (GPIO_ReadPin_Debounce(SW_ADDR_0_GPIO_Port, SW_ADDR_0_Pin) == GPIO_PIN_RESET) sw_val |= 0x01;
    if (GPIO_ReadPin_Debounce(SW_ADDR_1_GPIO_Port, SW_ADDR_1_Pin) == GPIO_PIN_RESET) sw_val |= 0x02;
    if (GPIO_ReadPin_Debounce(SW_ADDR_2_GPIO_Port, SW_ADDR_2_Pin) == GPIO_PIN_RESET) sw_val |= 0x04;
    sw->comm_addr = (uint8_t)(sw_val & 0x07U);

    // 终端电阻
    sw->term_res =
        (GPIO_ReadPin_Debounce(SW_TERM_RES_GPIO_Port, SW_TERM_RES_Pin) == GPIO_PIN_RESET) ?
        TERM_RES_ENABLE : TERM_RES_DISABLE;
}

void DipSwitch_Print(const DipSwitch_TypeDef *sw)
{
    static const char *microstep_str[] =
    {
        "1", "2", "4", "8", "16", "32", "64", "128"
    };

    static const char *dir_str[] =
    {
        "Forward", "Reverse"
    };

    static const char *mode_str[] =
    {
        "Closed-Loop", "Open-Loop"
    };

    static const char *motor_str[] =
    {
        "Stepper", "BLDC"
    };

    static const char *res_str[] =
    {
        "Disable", "Enable"
    };

    if (sw == NULL)
    {
        return;
    }

    printf("================ Dip Switch Status ================\r\n");
    printf("Microstep        : %s\r\n", microstep_str[sw->microstep]);
    printf("Direction        : %s\r\n", dir_str[sw->direction]);
    printf("Control Mode     : %s\r\n", mode_str[sw->ctrl_mode]);
    printf("Motor Type       : %s\r\n", motor_str[sw->motor_type]);
    printf("Comm Address     : %u\r\n", sw->comm_addr);
    printf("Terminal Resistor: %s\r\n", res_str[sw->term_res]);
    printf("===================================================\r\n");
}
