#include "dip_switch.h"
#include "gpio.h"
#include "usb_device.h"
#include <stdio.h>

// 去抖函数
static uint8_t GPIO_ReadPin_Debounce(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint8_t val1 = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
    HAL_Delay(10);
    uint8_t val2 = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
    return (val1 == val2) ? val1 : val2;
}
// 读取拨码开关状态
void DipSwitch_Read(DipSwitch_TypeDef *sw)
{
	uint8_t sw_val = 0;
  // 细分控制（拨码1-3，PE6/PE7/PE8）
  if(GPIO_ReadPin_Debounce(GPIOE, SW_MICROSTEP_0_Pin) == GPIO_PIN_RESET) sw_val |= 0x01;
  if(GPIO_ReadPin_Debounce(GPIOE, SW_MICROSTEP_1_Pin) == GPIO_PIN_RESET) sw_val |= 0x02;
  if(GPIO_ReadPin_Debounce(GPIOE, SW_MICROSTEP_2_Pin) == GPIO_PIN_RESET) sw_val |= 0x04;
	sw->microstep = (MicroStep_TypeDef)sw_val;
	
  // 旋转方向（拨码4，PE9）
  sw->direction = (GPIO_ReadPin_Debounce(GPIOE, SW_DIR_Pin) == GPIO_PIN_RESET) ? DIR_REVERSE : DIR_FORWARD;

  // 控制模式（拨码5，PE10）
  sw->ctrl_mode = (GPIO_ReadPin_Debounce(GPIOE, SW_CTRL_MODE_Pin) == GPIO_PIN_RESET) ? CTRL_MODE_OPEN_LOOP : CTRL_MODE_CLOSED_LOOP;

  // 电机类型（拨码6，PE11）
  sw->motor_type = (GPIO_ReadPin_Debounce(GPIOE, SW_MOTOR_TYPE_Pin) == GPIO_PIN_RESET) ? MOTOR_TYPE_BLDC : MOTOR_TYPE_STEP;

  // 通讯地址（拨码7-9，PE12/PE13/PE14）
  sw_val = 0;
  if(GPIO_ReadPin_Debounce(GPIOE, SW_ADDR_0_Pin) == GPIO_PIN_RESET) sw_val |= 0x01;
  if(GPIO_ReadPin_Debounce(GPIOE, SW_ADDR_1_Pin) == GPIO_PIN_RESET) sw_val |= 0x02;
  if(GPIO_ReadPin_Debounce(GPIOE, SW_ADDR_2_Pin) == GPIO_PIN_RESET) sw_val |= 0x04;
	sw->comm_addr = sw_val;
	
  // 终端电阻选择（拨码10，PE15）
  sw->term_res = (GPIO_ReadPin_Debounce(GPIOE, SW_TERM_RES_Pin) == GPIO_PIN_RESET) ? TERM_RES_ENABLE : TERM_RES_DISABLE;
}

// 初始化（可选，这里主要是确保引脚已初始化）
void DipSwitch_Init(void)
{
  // CubeMX 已经生成了 GPIO 初始化代码，这里可以留空或添加额外配置
}



void DipSwitch_Print(DipSwitch_TypeDef *sw)
{
    // 全部改用英文，彻底解决中文编码问题
    const char* microstep_str[] = {"Full","2","4","8","16","32","64","128"};
    const char* dir_str[] = {"Forward","Reverse"};
    const char* mode_str[] = {"Closed-Loop","Open-Loop"};
    const char* motor_str[] = {"Stepper","BLDC"};  // 修复motor_str未定义问题
    const char* res_str[] = {"Disable","Enable"};

    // 英文打印，避免乱码
    printf("================ Dip Switch Status ================\r\n");
    printf("Microstep: %s Step\r\n", microstep_str[sw->microstep]);
    printf("Direction: %s\r\n", dir_str[sw->direction]);
    printf("Control Mode: %s\r\n", mode_str[sw->ctrl_mode]);
    printf("Motor Type: %s\r\n", motor_str[sw->motor_type]);
    printf("Comm Address: %d\r\n", sw->comm_addr);
    printf("Terminal Resistor: %s\r\n", res_str[sw->term_res]);
    printf("==============================================\r\n");
}

