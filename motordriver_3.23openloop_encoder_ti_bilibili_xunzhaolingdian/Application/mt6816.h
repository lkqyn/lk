#ifndef __MT6816_H
#define __MT6816_H

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "spi.h"
#include "main.h"

#define MT6816_SPI_CS_H()         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define MT6816_SPI_CS_L()         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define MT6816_SPI_Get_HSPI       (hspi3)

extern uint16_t init_angle;
extern int DIR;
extern uint8_t mt6816_flag;

typedef struct
{
    uint16_t sample_data;     // 原始16位数据
    uint16_t angle;           // 14位角度值 0~16383
    bool parity_ok;           // 奇偶校验是否通过
    bool no_magnet_flag;      // 磁场不足标志
} MT6816_SPI_Data_Typedef;

extern MT6816_SPI_Data_Typedef mt6816_spi_data;

// 读取一次角度原始数据
bool RINE_mt6816_spi_data_Get_AngleData(void);

// 带重试读取角度，最多尝试10次
float REIN_MT6816_GetAngleData(void);

// 初始化
void REIN_mt6816_spi_data_Signal_Init(void);

// 检测编码器是否存在
bool check_mt6816(void);

#endif
