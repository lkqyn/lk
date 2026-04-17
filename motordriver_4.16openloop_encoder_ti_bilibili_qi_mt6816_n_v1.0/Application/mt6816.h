#ifndef __MT6816_H
#define __MT6816_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "spi.h"
/* SPI句柄：按你当前硬件是 SPI3 */
#define MT6816_SPI_Get_HSPI      (hspi3)

/* CS脚：按你当前硬件是 PA4 */
#define MT6816_SPI_CS_PORT       GPIOA
#define MT6816_SPI_CS_PIN        GPIO_PIN_4

#define MT6816_SPI_CS_L()        HAL_GPIO_WritePin(MT6816_SPI_CS_PORT, MT6816_SPI_CS_PIN, GPIO_PIN_RESET)
#define MT6816_SPI_CS_H()        HAL_GPIO_WritePin(MT6816_SPI_CS_PORT, MT6816_SPI_CS_PIN, GPIO_PIN_SET)

typedef struct
{
    uint16_t sample_data;
    uint16_t angle;
    bool parity_ok;
    bool no_magnet_flag;
} MT6816_SPI_Data_Typedef;

extern int DIR;
extern uint16_t dir_desirde[2];
extern MT6816_SPI_Data_Typedef mt6816_spi_data;
extern uint16_t init_angle;
extern uint8_t mt6816_flag;
extern uint16_t tetemp1[200];
extern uint16_t tetemp2[200];
extern uint8_t encoder_adjust_flag;
extern uint16_t last_data;

void delay(void);
void REIN_mt6816_spi_data_Signal_Init(void);
bool RINE_mt6816_spi_data_Get_AngleData(void);
float REIN_MT6816_GetAngleData(void);
bool check_mt6816(void);

#ifdef __cplusplus
}
#endif

#endif