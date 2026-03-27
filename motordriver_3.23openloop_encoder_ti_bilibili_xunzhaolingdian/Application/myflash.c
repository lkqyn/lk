#include "myflash.h"

/**
  * @brief  HAL库版写一个uint32_t类型的数据
  * @param  addr: 存储数据的地址
  * @param  data: 写入的数据数组
  * @retval 成功返回0，失败返回-1
  */
int Flash_HAL_Write_Data(uint32_t addr, uint32_t data[2])
{
    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    |
                           FLASH_FLAG_OPERR  |
                           FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR |
                           FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = FLASH_USER_SECTOR;
    EraseInitStruct.NbSectors    = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    // 这里写两个uint32_t
    for (uint16_t i = 0; i < 2; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return -1;
        }
        addr += sizeof(uint32_t);
    }

    HAL_FLASH_Lock();
    return 0;
}


/**
  * @brief  HAL库版写N个uint16_t类型的数据
  * @param  addr: 存储数据的地址
  * @param  data: 数据数组
  * @param  num: 数据的个数
  * @retval 成功返回0，失败返回-1
  */
int Flash_HAL_Write_N_Data(uint32_t addr, uint16_t *data, uint16_t num)
{
    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    |
                           FLASH_FLAG_OPERR  |
                           FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR |
                           FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = FLASH_USER_SECTOR;
    EraseInitStruct.NbSectors    = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    
    // 虽然传入的是uint16_t，但每个数据占4字节地址
    for (uint16_t i = 0; i < num; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint32_t)data[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return -1;
        }
        addr += sizeof(uint32_t);
    }

    HAL_FLASH_Lock();
    return 0;
}


/**
  * @brief  HAL库版读取N个uint16_t类型的数据
  * @param  addr: 读取数据的地址
  * @param  data: 数据数组
  * @param  num: 数据的个数
  * @retval NONE
  */
void Flash_HAL_Read_N_Data(uint32_t addr, uint16_t *data, uint32_t num)
{
    for (uint32_t i = 0; i < num; i++)
    {
        data[i] = *(volatile uint16_t *)addr;
        addr += sizeof(uint32_t);   // 保持和写入方式一致，每个数据间隔4字节
    }
}


/**
  * @brief  HAL库版读取N个uint8_t类型的数据
  * @param  addr: 读取数据的地址
  * @param  data: 数据数组
  * @param  num: 数据的个数
  * @retval NONE
  */
void Flash_HAL_Read_N_Byte(uint32_t addr, uint8_t *data, uint32_t num)
{
    for (uint32_t i = 0; i < num; i++)
    {
        data[i] = *(volatile uint8_t *)addr;
        addr += sizeof(uint8_t);
    }
}


/**
  * @brief  读取200个角度补偿点
  * @param  buf: 长度为200的数组
  * @retval NONE
  */
void Flash_Read_Angle(uint16_t buf[200])
{
    Flash_HAL_Read_N_Data(FLASH_USER_START_ADDR, buf, 200);
}


/**
  * @brief  读取方向标志
  * @param  addr: 第几个数据位置，例如读取temp[200]则传200
  * @retval 方向标志值
  */
uint16_t flash_read_dir(int addr)
{
    return *(volatile uint16_t *)(FLASH_USER_START_ADDR + addr * 4);
}