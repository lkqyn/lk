#include "mt6816.h"
#include "FOC.h"
#include "myflash.h"
#include <stdlib.h>
int DIR = 1;                    // 默认方向
uint16_t init_angle = 0;        // 上电初始角度
uint8_t mt6816_flag = 0;        // 编码器存在标志
uint16_t dir_desirde[2]={0};//用于判断DIR的值
MT6816_SPI_Data_Typedef mt6816_spi_data = {0};
uint16_t tetemp1[200] = {0};
uint16_t tetemp2[200] = {0};
extern float vol;
static uint16_t last_data = 0;  // 上一次正确角度
uint8_t encoder_adjust_flag=1;//定义编码器校准的标记 // 1:正常运行 0:校准中

uint16_t aglc[4]={0,256,512,768};
uint16_t aglc1[4]={768,512,256,0};
//环形平均
static uint16_t mt6816_circular_mean(uint16_t a, uint16_t b)
{
    int32_t diff = (int32_t)a - (int32_t)b;

    // 处理0~16383跨零点情况
    if (abs(diff) > 8192)
    {
        if (a > b)
        {
            b += 16384;
        }
        else
        {
            a += 16384;
        }
    }

    uint16_t mean = (uint16_t)(((uint32_t)a + (uint32_t)b) / 2U);

    if (mean >= 16384)
    {
        mean -= 16384;
    }

    return mean;
}
void REIN_mt6816_spi_data_Signal_Init(void)   // 编码器校准，识别编码器方向，非线性校准等
{
    mt6816_spi_data.sample_data = 0;
    mt6816_spi_data.angle = 0;
    encoder_adjust_flag = 0;      // 编码器校准中

    uint16_t temp[201] = {0};

    // 1. 初始化FOC
    FOC_BaseInit();

    // 2. 设置一个合适的开环电压
    // 参考你原来的写法：bbb = 48 / vol
    uint8_t bbb = (uint8_t)(48.0f / vol);

    // 防止过大或过小
    if (bbb < 2)  bbb = 2;
    if (bbb > 12) bbb = 12;

    set_uduq(&m1_foc, 0, bbb);

    // 3. 先锁到一个初始位置
    foc_open(&m1_foc, 0);
    HAL_Delay(200);

    REIN_MT6816_GetAngleData();

    // 4. 判断编码器方向
    foc_open(&m1_foc, 256);
    HAL_Delay(30);
    dir_desirde[0] = (uint16_t)REIN_MT6816_GetAngleData();

    foc_open(&m1_foc, 512);
    HAL_Delay(30);
    REIN_MT6816_GetAngleData();

    foc_open(&m1_foc, 768);
    HAL_Delay(30);
    REIN_MT6816_GetAngleData();

    foc_open(&m1_foc, 0);
    HAL_Delay(1000);
    dir_desirde[1] = (uint16_t)REIN_MT6816_GetAngleData();

    {
        int16_t delta = (int16_t)(dir_desirde[1] - dir_desirde[0]);

        // 防止跨零点导致判断错误
        if (delta > 8192)
            delta -= 16384;
        else if (delta < -8192)
            delta += 16384;

        if ((delta > 0) && (abs(delta) < 4000))
            DIR = 1;
        else
            DIR = -1;
    }

    // 5. 正转采样一圈
    for (uint16_t i = 0; i < 200; i++)
    {
        tetemp1[i] = (uint16_t)REIN_MT6816_GetAngleData();
        foc_open(&m1_foc, aglc[(i + 1) % 4]);
        HAL_Delay(10);
    }

    foc_open(&m1_foc, 0);
    HAL_Delay(500);

    // 6. 反转采样一圈
    for (uint16_t i = 0; i < 200; i++)
    {
        tetemp2[i] = (uint16_t)REIN_MT6816_GetAngleData();
        foc_open(&m1_foc, aglc1[i % 4]);
        HAL_Delay(10);
    }

    // 7. 正反采样取平均，生成补偿表
    temp[0] = mt6816_circular_mean(tetemp1[0], tetemp2[0]);

    for (uint16_t j = 1; j < 200; j++)
    {
        temp[j] = mt6816_circular_mean(tetemp1[j], tetemp2[200 - j]);
    }

    // 8. 关闭电机输出
    set_uduq(&m1_foc, 0, 0);
    foc_open(&m1_foc, 0);

    // 9. 保存方向标志
    if (DIR == 1)
        temp[200] = 11;
    else
        temp[200] = 22;

    // 10. 写入Flash

    Flash_HAL_Write_N_Data(FLASH_USER_START_ADDR, temp, 201);

    HAL_Delay(1000);


    encoder_adjust_flag = 1;   // 校准完成

    // 11. 软件复位，重新加载参数
    __disable_irq();
    HAL_NVIC_SystemReset();
}
/**
 * @brief  读取一次 MT6816 的原始角度数据
 * @retval true  读取成功
 * @retval false 读取失败
 */
bool RINE_mt6816_spi_data_Get_AngleData(void)
{
    uint16_t data_t[2];
    uint16_t data_r[2];
    uint8_t h_count = 0;

    // 读寄存器 0x03 和 0x04
    data_t[0] = ((0x80 | 0x03) << 8);
    data_t[1] = ((0x80 | 0x04) << 8);

    mt6816_spi_data.parity_ok = false;
    mt6816_spi_data.no_magnet_flag = false;

    for(uint8_t i = 0; i < 3; i++)
    {
        // 读取高8位
        MT6816_SPI_CS_L();
        if(HAL_SPI_TransmitReceive(&MT6816_SPI_Get_HSPI,
                                   (uint8_t *)&data_t[0],
                                   (uint8_t *)&data_r[0],
                                   1,
                                   100) != HAL_OK)
        {
            MT6816_SPI_CS_H();
            continue;
        }
        MT6816_SPI_CS_H();

        // 读取低8位
        MT6816_SPI_CS_L();
        if(HAL_SPI_TransmitReceive(&MT6816_SPI_Get_HSPI,
                                   (uint8_t *)&data_t[1],
                                   (uint8_t *)&data_r[1],
                                   1,
                                   100) != HAL_OK)
        {
            MT6816_SPI_CS_H();
            continue;
        }
        MT6816_SPI_CS_H();

        // 拼接16位数据
        mt6816_spi_data.sample_data = ((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF);

        // 偶校验
        h_count = 0;
        for(uint8_t j = 0; j < 16; j++)
        {
            if(mt6816_spi_data.sample_data & (1U << j))
            {
                h_count++;
            }
        }

        if((h_count & 0x01) == 0)
        {
            mt6816_spi_data.parity_ok = true;
            break;
        }
    }

    if(mt6816_spi_data.parity_ok)
    {
        // 右移2位得到14bit角度
        mt6816_spi_data.angle = (mt6816_spi_data.sample_data >> 2) & 0x3FFF;

        // bit1 为磁场不足标志
        mt6816_spi_data.no_magnet_flag = (bool)(mt6816_spi_data.sample_data & (1U << 1));

        if(mt6816_spi_data.no_magnet_flag)
        {
            return false;
        }

        return true;
    }

    return false;
}

/**
 * @brief  带重试读取角度
 * @retval 当前角度值，失败时返回上一次成功读取的值
 */
float REIN_MT6816_GetAngleData(void)
{
    for(uint8_t retry = 0; retry < 10; retry++)
    {
        if(RINE_mt6816_spi_data_Get_AngleData())
        {
            last_data = mt6816_spi_data.angle;
            return (float)mt6816_spi_data.angle;
        }
    }

    return (float)last_data;
}


/**
 * @brief  检查编码器是否存在
 * @retval true 存在
 * @retval false 不存在
 */
bool check_mt6816(void)
{
    uint8_t ok_cnt = 0;

    for(uint8_t i = 0; i < 10; i++)
    {
        if(RINE_mt6816_spi_data_Get_AngleData())
        {
            if(mt6816_spi_data.angle <= 16383)
            {
                ok_cnt++;
            }
        }
        HAL_Delay(1);
    }

    if(ok_cnt >= 8)
    {
        return true;
    }
    else
    {
        return false;
    }
}