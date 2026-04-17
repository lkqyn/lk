#include "mt6816.h"
#include <stdlib.h>
#include "myflash.h"
#include "FOC.h"
//#include "float_char.h"
//#include "iwdg.h"
#include "math.h"

int  DIR = -1;                       // 编码器方向+-1
uint16_t dir_desirde[2] = {0};       // 用于判断DIR的值
MT6816_SPI_Data_Typedef mt6816_spi_data;
uint16_t init_angle = 0;
uint8_t mt6816_flag = 0;             // 是否存在编码器
uint16_t tetemp1[200] = {0};
uint16_t tetemp2[200] = {0};
extern float vol;
uint8_t encoder_adjust_flag = 1;     // 定义编码器校准的标记

uint16_t aglc[4]  = {0, 256, 512, 768};
uint16_t aglc1[4] = {768, 512, 256, 0};

uint16_t last_data = 0;

void delay(void)
{
    uint8_t i = 170;
    uint16_t j = 1000;
    while(i--)
        for(; j > 0; j--);
}

void REIN_mt6816_spi_data_Signal_Init(void)   // 编码器校准，识别编码器方向，非线性校准等
{
    mt6816_spi_data.sample_data = 0;
    mt6816_spi_data.angle = 0;
    mt6816_spi_data.parity_ok = false;
    mt6816_spi_data.no_magnet_flag = false;

    encoder_adjust_flag = 0;   // 编码器校准中
    uint16_t temp[201] = {0};
    m1_foc.Udc      = 24.0f;
    m1_foc.outmax   = 20.0f;
    //b_foc_init();

    uint8_t bbb = 48 / vol;
    set_uduq(&m1_foc, 0, bbb);
    foc_open(&m1_foc, 0);
    HAL_Delay(200);

    REIN_MT6816_GetAngleData();

    foc_open(&m1_foc, 256);
    HAL_Delay(30);
    dir_desirde[0] = REIN_MT6816_GetAngleData();

    foc_open(&m1_foc, 512);
    HAL_Delay(30);
    REIN_MT6816_GetAngleData();

    foc_open(&m1_foc, 768);
    HAL_Delay(30);

    foc_open(&m1_foc, 0);
    HAL_Delay(1000);
    dir_desirde[1] = REIN_MT6816_GetAngleData();

    //HAL_IWDG_Refresh(&hiwdg);   // 看门狗清零

    if(((int16_t)(dir_desirde[1] - dir_desirde[0]) > 0) &&
       (abs((int16_t)(dir_desirde[1] - dir_desirde[0])) < 4000))   // 判断编码器方向
        DIR = 1;
    else
        DIR = -1;

    ///////// 正转一圈
    for(uint16_t i = 0; i < 200; i++)
    {
        tetemp1[i] = REIN_MT6816_GetAngleData();
        foc_open(&m1_foc, aglc[(i + 1) % 4]);
        HAL_Delay(10);
        //HAL_IWDG_Refresh(&hiwdg);   // 看门狗清零
    }

    foc_open(&m1_foc, 0);
    HAL_Delay(500);

    ///////////// 反转一圈
    for(uint16_t i = 0; i < 200; i++)
    {
        tetemp2[i] = REIN_MT6816_GetAngleData();
        foc_open(&m1_foc, aglc1[i % 4]);
        HAL_Delay(10);
        //HAL_IWDG_Refresh(&hiwdg);   // 看门狗清零
    }

    for(uint8_t jj = 0; jj < 200; jj++)   // 将正转反转得到的值取平均
    {
        if(jj == 0)
        {
            if(abs((int16_t)(tetemp1[0] - tetemp2[0])) > 200)
            {
                if(tetemp1[0] > tetemp2[0])
                {
                    if((16384 - tetemp1[0]) > tetemp2[0])   // 在大的数这一边
                        temp[jj] = (uint16_t)(0.5f * (tetemp1[0] + tetemp2[0] + 16384));
                    else                                     // 在小的数那一边
                        temp[jj] = (uint16_t)(0.5f * (tetemp1[0] + tetemp2[0] - 16384));
                }
                else
                {
                    if((16384 - tetemp2[0]) > tetemp1[0])   // 在大的数这一边
                        temp[jj] = (uint16_t)(0.5f * (tetemp1[0] + tetemp2[0] + 16384));
                    else                                     // 在小的数那一边
                        temp[jj] = (uint16_t)(0.5f * (tetemp1[0] + tetemp2[0] - 16384));
                }
            }
            else
            {
                temp[jj] = (uint16_t)(0.5f * (tetemp1[0] + tetemp2[0]));
            }
        }
        else
        {
            if(abs((int16_t)(tetemp1[jj] - tetemp2[200 - jj])) > 200)
            {
                if(tetemp1[jj] > tetemp2[200 - jj])
                {
                    if((16384 - tetemp1[jj]) > tetemp2[200 - jj])   // 在大的数这一边
                        temp[jj] = (uint16_t)(0.5f * (tetemp1[jj] + tetemp2[200 - jj] + 16384));
                    else                                             // 在小的数那一边
                        temp[jj] = (uint16_t)(0.5f * (tetemp1[jj] + tetemp2[200 - jj] - 16384));
                }
                else
                {
                    if((16384 - tetemp2[200 - jj]) > tetemp1[jj])   // 在大的数这一边
                        temp[jj] = (uint16_t)(0.5f * (tetemp1[jj] + tetemp2[200 - jj] + 16384));
                    else                                             // 在小的数那一边
                        temp[jj] = (uint16_t)(0.5f * (tetemp1[jj] + tetemp2[200 - jj] - 16384));
                }
            }
            else
            {
                temp[jj] = (uint16_t)(0.5f * (tetemp1[jj] + tetemp2[200 - jj]));
            }
        }
    }

    set_uduq(&m1_foc, 0, 0);
    foc_open(&m1_foc, 0);

    if(DIR == 1)      // 保存编码器方向值
        temp[200] = 11;
    else
        temp[200] = 22;

    Flash_HAL_Write_N_Data(FLASH_USER_START_ADDR, temp, 201);

    HAL_Delay(1000);
    encoder_adjust_flag = 1;   // 校准完成

    __disable_irq();           // 关闭所有中断
    HAL_NVIC_SystemReset();    // 执行复位
}

bool RINE_mt6816_spi_data_Get_AngleData(void)
{
    uint16_t data_t[2];
    uint16_t data_r[2];
    uint8_t h_count;

    data_t[0] = (uint16_t)((0x80 | 0x03) << 8);
    data_t[1] = (uint16_t)((0x80 | 0x04) << 8);

    for(uint8_t i = 0; i < 3; i++)
    {
        // 读取SPI数据
        MT6816_SPI_CS_L();
        HAL_SPI_TransmitReceive(&MT6816_SPI_Get_HSPI, (uint8_t*)&data_t[0], (uint8_t*)&data_r[0], 1, HAL_MAX_DELAY);
        MT6816_SPI_CS_H();

        MT6816_SPI_CS_L();
        HAL_SPI_TransmitReceive(&MT6816_SPI_Get_HSPI, (uint8_t*)&data_t[1], (uint8_t*)&data_r[1], 1, HAL_MAX_DELAY);
        MT6816_SPI_CS_H();

        mt6816_spi_data.sample_data = (uint16_t)(((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF));

        // 奇偶校验
        h_count = 0;
        for(uint8_t j = 0; j < 16; j++)
        {
            if(mt6816_spi_data.sample_data & (0x0001 << j))
                h_count++;
        }

        if(h_count & 0x01)
        {
            mt6816_spi_data.parity_ok = false;
        }
        else
        {
            mt6816_spi_data.parity_ok = true;
            break;
        }
    }

    if(mt6816_spi_data.parity_ok)
    {
        mt6816_spi_data.angle = mt6816_spi_data.sample_data >> 2;
        mt6816_spi_data.no_magnet_flag = (bool)(mt6816_spi_data.sample_data & (0x0001 << 1));

        if(mt6816_spi_data.no_magnet_flag)   // 磁通不足
            return false;
        else
            return true;
    }
    else
    {
        return false;   // 奇偶校验不通过
    }
}

// 封装函数：带重试读取角度，最多尝试10次
float REIN_MT6816_GetAngleData(void)
{
    uint8_t retry = 0;

    for(retry = 0; retry < 10; retry++)
    {
        if(RINE_mt6816_spi_data_Get_AngleData())
        {
            last_data = mt6816_spi_data.angle;
            return mt6816_spi_data.angle;
        }
    }

    // 读取失败，返回上一次正确的数据
    return last_data;
}

bool check_mt6816(void)   // 检测编码器是否存在
{
    uint8_t n = 0;

    for(uint8_t i = 0; i < 10; i++)
    {
        if(RINE_mt6816_spi_data_Get_AngleData() && mt6816_spi_data.angle < 16385)
            n++;
        HAL_Delay(1);
    }

    if(n >= 9)   // 9次检测到数据为真
        return true;
    else
        return false;
}