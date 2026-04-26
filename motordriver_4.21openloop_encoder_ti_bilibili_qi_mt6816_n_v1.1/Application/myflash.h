#ifndef __MYFLASH_H
#define __MYFLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * STM32F407VET6 Flash 512KB
 * Sector 7: 0x08060000 ~ 0x0807FFFF (128KB)
 * 这里把最后一个扇区拿来存用户数据
 */
#define FLASH_USER_START_ADDR   ((uint32_t)0x08060000U)

/* 参考代码中方向值存放位置 */
#define DIR_AD                  200

int Flash_HAL_Write_Data(uint32_t addr, uint32_t data[2]);
int Flash_HAL_Write_N_Data(uint32_t addr, uint16_t *data, uint16_t num);
void Flash_HAL_Read_N_Data(uint32_t addr, uint16_t *data, uint32_t num);
void Flash_HAL_Read_N_Byte(uint32_t addr, uint8_t *data, uint32_t num);

void Flash_Read_Angle(uint16_t buf[200]);
uint16_t flash_read_dir(int addr);

#ifdef __cplusplus
}
#endif

#endif