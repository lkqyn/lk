#include <exio/exio.h>
#include "lv8731v/lv8731v.h"
#include "delay/delay.h"
#include "sysconfig/sysconfig.h"

const GpioPin exIO[] = {
        // INPUT_1 ~ INPUT_4
        {GPIOD,GPIO_PIN_3},{GPIOD,GPIO_PIN_2},
        {GPIOD,GPIO_PIN_1},{GPIOD,GPIO_PIN_0},
		{GPIOA,GPIO_PIN_7},{GPIOA,GPIO_PIN_6},
		{GPIOE,GPIO_PIN_0},{GPIOE,GPIO_PIN_1},
        // OUTPUT_1 ~ OUTPUT_8
        {GPIOD,GPIO_PIN_10},{GPIOD,GPIO_PIN_9},
        {GPIOD,GPIO_PIN_8},{GPIOB,GPIO_PIN_15},
        {GPIOE,GPIO_PIN_6},{GPIOC,GPIO_PIN_13},
		{GPIOC,GPIO_PIN_15},{GPIOD,GPIO_PIN_15},
};

void EXIO_interruptConfig(void);

void EXIO_config(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    
    for(i=8;i<16;i++)
    {
        HAL_GPIO_WritePin(exIO[i].GPIOx,exIO[i].pin,GPIO_PIN_SET);
    }



    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	for(i=8;i<16;i++)
	{
		gpio.Pin = exIO[i].pin;
		HAL_GPIO_Init(exIO[i].GPIOx, &gpio);
	}

	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_PULLUP;
	for (i = 0; i < 8; i++)
	{
		gpio.Pin = exIO[i].pin;
		HAL_GPIO_Init(exIO[i].GPIOx, &gpio);
	}



    //EXIO_interruptConfig();

    //EXIO_enableEXI0Interrupt();
}

/**
 @brief ����EXIO�����ƽ
 @param -no- ��Ҫ�޸ĵ����ͨ�� 1~4.
 @param -level- �����ƽ. 0-�͵�ƽ; 1-�ߵ�ƽ
*/
void EXIO_setOutput(uint8_t no,uint8_t level)
{
    switch(level)
    {
        case 1:
            HAL_GPIO_WritePin(exIO[no+8].GPIOx,exIO[no+8].pin,GPIO_PIN_SET);
            break;
        
        case 0:
            HAL_GPIO_WritePin(exIO[no+8].GPIOx,exIO[no+8].pin,GPIO_PIN_RESET);
            break;
    }
}

/**
 @brief ���EXIO�����ƽ
 @param -no- ��Ҫ��ȡ������ͨ�� 1~4
 @return 0-�͵�ƽ; 1-�ߵ�ƽ
*/
uint8_t EXIO_getInput(uint8_t no)
{
    uint8_t rtl=1;
    
//    if(HAL_GPIO_ReadPin(exIO[no].GPIOx,exIO[no].pin) == GPIO_PIN_RESET)
//    {
//#ifdef CONF_SOFTWARE_WAIT_INPUT_STABLE
//    	delay_ms(CONF_SOFTWARE_WAIT_INPUT_TIME);
//#endif
//        if(HAL_GPIO_ReadPin(exIO[no].GPIOx,exIO[no].pin) == GPIO_PIN_RESET)
//        {
//            rtl = 0;
//        }
//    }
    if(HAL_GPIO_ReadPin(exIO[no].GPIOx,exIO[no].pin) == GPIO_PIN_RESET)
   {
	   rtl = 0;
   }
    return rtl;
}

uint8_t EXIO_getOutput(uint8_t no)
{
    uint8_t rtl=1;

    if(HAL_GPIO_ReadPin(exIO[no+8].GPIOx,exIO[no+8].pin) == GPIO_PIN_RESET)
    {
#ifdef CONF_SOFTWARE_WAIT_INPUT_STABLE
    	delay_ms(CONF_SOFTWARE_WAIT_INPUT_TIME);
#endif
        if(HAL_GPIO_ReadPin(exIO[no+8].GPIOx,exIO[no+8].pin) == GPIO_PIN_RESET)
        {
            rtl = 0;
        }
    }
    return rtl;
}
/**
 �ж�����. �����˿�, �жϷ�ʽ����, ����λ��������Ҫʹ��ʱ���ú�����. 
*/
void EXIO_interruptConfig(void)
{
	HAL_NVIC_DisableIRQ (EXTI3_IRQn);
	HAL_NVIC_DisableIRQ (EXTI2_IRQn);
	HAL_NVIC_DisableIRQ (EXTI1_IRQn);
	HAL_NVIC_DisableIRQ (EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI3_IRQn,0,0);
    HAL_NVIC_SetPriority(EXTI2_IRQn,0,0);
    HAL_NVIC_SetPriority(EXTI1_IRQn,0,0);
    HAL_NVIC_SetPriority(EXTI0_IRQn,0,0);
    
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    
    AFIO->EXTICR[0] |= (AFIO_EXTICR1_EXTI3_PD | AFIO_EXTICR1_EXTI2_PD | AFIO_EXTICR1_EXTI1_PD | AFIO_EXTICR1_EXTI0_PD );
    EXTI->FTSR |= (EXTI_FTSR_TR3 | EXTI_FTSR_TR2 | EXTI_FTSR_TR1 | EXTI_FTSR_TR0);
    EXTI->IMR |= (EXTI_IMR_MR3 | EXTI_FTSR_TR2 | EXTI_FTSR_TR1 | EXTI_FTSR_TR0);
    
    HAL_NVIC_EnableIRQ (EXTI3_IRQn);
    HAL_NVIC_EnableIRQ (EXTI2_IRQn);
    HAL_NVIC_EnableIRQ (EXTI1_IRQn);
    HAL_NVIC_EnableIRQ (EXTI0_IRQn);
}

