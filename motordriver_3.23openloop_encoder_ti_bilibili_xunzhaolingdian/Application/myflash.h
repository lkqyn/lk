#ifndef __MYFLASH_H
#define __MYFLASH_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * STM32F407VET6:
 * Flash总容量 512KB
 * 最后一个Sector7: 0x08060000 ~ 0x0807FFFF (128KB)
 * 这里先把Sector7作为用户参数存储区
 */
#define FLASH_USER_SECTOR        FLASH_SECTOR_7
#define FLASH_USER_START_ADDR    0x08060000U
#define DIR_AD 200
int Flash_HAL_Write_Data(uint32_t addr, uint32_t data[2]);
int Flash_HAL_Write_N_Data(uint32_t addr, uint16_t *data, uint16_t num);

void Flash_HAL_Read_N_Data(uint32_t addr, uint16_t *data, uint32_t num);
void Flash_HAL_Read_N_Byte(uint32_t addr, uint8_t *data, uint32_t num);

void Flash_Read_Angle(uint16_t buf[200]);
uint16_t flash_read_dir(int addr);

#endif