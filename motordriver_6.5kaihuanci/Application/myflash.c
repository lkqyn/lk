#include "myflash.h"

/**
  * @brief  获取地址所在的Flash扇区
  * @param  addr: Flash地址
  * @retval 扇区号
  */
static uint32_t GetSector(uint32_t addr)
{
    if(addr < 0x08004000) return FLASH_SECTOR_0;
    else if(addr < 0x08008000) return FLASH_SECTOR_1;
    else if(addr < 0x0800C000) return FLASH_SECTOR_2;
    else if(addr < 0x08010000) return FLASH_SECTOR_3;
    else if(addr < 0x08020000) return FLASH_SECTOR_4;
    else if(addr < 0x08040000) return FLASH_SECTOR_5;
    else if(addr < 0x08060000) return FLASH_SECTOR_6;
    else return FLASH_SECTOR_7;
}

/**
  * @brief  HAL库版写一个uint32_t类型的数据
  * @param  addr: 存储数据的地址
  * @param  data: 写入的数据
  * @retval 成功返回0，失败返回-1
  */
int Flash_HAL_Write_Data(uint32_t addr, uint32_t data[2])
{
    uint32_t buf[514] = {0};

    // 先把整个用户区备份出来，避免F407整扇区擦除把别的数据擦掉
    for(uint16_t i = 0; i < 514; i++)
    {
        buf[i] = *(volatile uint32_t *)(FLASH_USER_START_ADDR + i * 4);
    }

    // 将目标数据覆盖到备份区对应位置
    buf[(addr - FLASH_USER_START_ADDR) / 4] = data[0];
    buf[(addr - FLASH_USER_START_ADDR) / 4 + 1] = data[1];

    // 1、FLASH解锁
    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    |
                           FLASH_FLAG_OPERR  |
                           FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR |
                           FLASH_FLAG_PGSERR);

    // 2、FLASH擦除
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = GetSector(FLASH_USER_START_ADDR);
    EraseInitStruct.NbSectors    = 1;

    if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    // 3、FLASH写入
    for(uint16_t i = 0; i < 514; i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                             FLASH_USER_START_ADDR + i * 4,
                             buf[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return -1;
        }
    }

    // 4、FLASH上锁
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
    uint32_t buf[514] = {0};

    // 先把整个用户区备份出来，避免Sector擦除丢数据
    for(uint16_t i = 0; i < 514; i++)
    {
        buf[i] = *(volatile uint32_t *)(FLASH_USER_START_ADDR + i * 4);
    }

    // 按参考代码的习惯：uint16_t数据按4字节步进存
    for(uint16_t i = 0; i < num; i++)
    {
        buf[(addr - FLASH_USER_START_ADDR) / 4 + i] = (uint32_t)data[i];
    }

    // 1、FLASH解锁
    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    |
                           FLASH_FLAG_OPERR  |
                           FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR |
                           FLASH_FLAG_PGSERR);

    // 2、FLASH擦除
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = GetSector(FLASH_USER_START_ADDR);
    EraseInitStruct.NbSectors    = 1;

    if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    // 3、FLASH写入
    for(uint16_t i = 0; i < 514; i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                             FLASH_USER_START_ADDR + i * 4,
                             buf[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return -1;
        }
    }

    // 4、FLASH上锁
    HAL_FLASH_Lock();
    return 0;
}

/**
  * @brief  HAL库版读取N个uint16_t类型的数据
  * @param  addr: 读取数据的地址（用户空间的地址）
  * @param  data: 数据数组
  * @param  num: 数据的个数
  * @retval NONE
  */
void Flash_HAL_Read_N_Data(uint32_t addr, uint16_t *data, uint32_t num)
{
    for(uint32_t i = 0; i < num; i++)
    {
        data[i] = *(volatile uint16_t*)addr;
        addr += sizeof(uint32_t);
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
    for(uint32_t i = 0; i < num; i++)
    {
        data[i] = *(volatile uint8_t*)addr;
        addr += sizeof(uint8_t);
    }
}

void Flash_Read_Angle(uint16_t buf[200])
{
    Flash_HAL_Read_N_Data(FLASH_USER_START_ADDR, buf, 200);
}

uint16_t flash_read_dir(int addr)
{
    return *(__IO uint32_t *)(FLASH_USER_START_ADDR + addr * 4);
}