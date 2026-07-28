#include "mcp4728/mcp4728.h"
#include "mcp4728/iic_4728.h"

#define MCP4728_ADDRESS     0xc0

#define MCP4728_SINGALDAC   0X58

#define IS_RDY()       HAL_GPIO_ReadPin(MCP4728_RDYBSY_GPIOx,MCP4728_RDYBSY_PIN)

void MCP4728_config(void)
{
    GPIO_InitTypeDef gpio;
    
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pin = MCP4728_LDAC_PIN;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    
    HAL_GPIO_Init(MCP4728_LDAC_GPIOx,&gpio);
    
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pin = MCP4728_RDYBSY_PIN;
    gpio.Pull = GPIO_PULLUP;
    
    HAL_GPIO_Init(MCP4728_RDYBSY_GPIOx,&gpio);
    
    IIC4728_Init();
}

/**
 @brief 
 @param vref

*/
void MCP4728_vrefConfig(uint8_t vref)
{
    IIC4728_Start();
    
    IIC4728_Send_Byte(MCP4728_ADDRESS);
    
    IIC4728_Wait_Ack();
}

void MCP4728_setOutput(uint8_t channel,uint16_t output)
{
    uint8_t s;
    uint8_t temp;
    
    while(!IS_RDY());
    
    s = MCP4728_SINGALDAC + (channel<<1);
    
    IIC4728_Start();
    
    IIC4728_Send_Byte(MCP4728_ADDRESS);
    
    IIC4728_Wait_Ack();
    
    IIC4728_Send_Byte(s);
    
    IIC4728_Wait_Ack();
    
    temp = (output >> 8) & 0xf;
    
    IIC4728_Send_Byte(temp);
    
    IIC4728_Wait_Ack();
    
    temp = output & 0xff;
    
    IIC4728_Send_Byte(temp);
    
    IIC4728_Wait_Ack();
    
    IIC4728_Stop();
}

void MCP4728_writeOutputSeq(uint16_t out1,uint16_t out2,uint16_t out3,uint16_t out4)
{
    uint8_t temp;
    
    IIC4728_Start();
    
    IIC4728_Send_Byte(MCP4728_ADDRESS);
    
    IIC4728_Wait_Ack();
    
    IIC4728_Send_Byte(0x50);
    
    IIC4728_Wait_Ack();
    
    temp = (out1 >> 8) & 0xf;
    
    IIC4728_Send_Byte(temp);
    
    IIC4728_Wait_Ack();
    
    temp = out1 & 0xff;
    
    IIC4728_Send_Byte(temp);
    
    IIC4728_Wait_Ack();
    
    temp = (out2 >> 8) & 0xf;
    
    IIC4728_Send_Byte(temp);
    
    IIC4728_Wait_Ack();
    
    temp = out2 & 0xff;
    
    IIC4728_Send_Byte(temp);
    
    IIC4728_Wait_Ack();
    
    temp = (out3 >> 8) & 0xf;
    
    IIC4728_Send_Byte(temp);
    
    IIC4728_Wait_Ack();
    
    temp = out3 & 0xff;
    
    IIC4728_Send_Byte(temp);
    
    IIC4728_Wait_Ack();
    
    temp = (out4 >> 8) & 0xf;
    
    IIC4728_Send_Byte(temp);
    
    IIC4728_Wait_Ack();
    
    temp = out4 & 0xff;
    
    IIC4728_Send_Byte(temp);
    
    IIC4728_Wait_Ack();
    
    IIC4728_Stop();
}
