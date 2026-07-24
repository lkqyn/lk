#include "pwm2/pwm2.h"

TIM_HandleTypeDef htim2;

volatile uint32_t PWM2_pulseCount=0;

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 359;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 9999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        //Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        //Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        //Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        //Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 9;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        //Error_Handler();
    }
    //HAL_TIM_MspPostInit(&htim2);
}

void PWM2_config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    /**TIM2 GPIO Configuration    
    PA0-WKUP     ------> TIM2_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    MX_TIM2_Init();
    
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->DIER |= TIM_DIER_UIE;
    
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief ����PWM2
 */
void PWM2_start(void)
{
    PWM2_pulseCount = 0;
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    
}

/**
 * @brief ֹͣPWM2
 */
void PWM2_stop(void)
{
    HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
}


inline void PWM2_setFrequent(uint32_t f)
{
    if(f < 100)                         // 0~99Hz
    {
        TIM2->PSC = 7199;               // ��ʱ��ʱ��10kHz
        TIM2->ARR = 10000 / f - 1;
//        if((1000 % f) != 0) TIM2->ARR += 1;
        TIM2->CCR1 = (TIM2->ARR >> 1);
    }
    else if(f <= 500000)                // 100Hz ~ 500kHz
    {
        TIM2->PSC = 71;                 // ��ʱ��ʱ��1MHz
        TIM2->ARR = 1000000 / f - 1;          // t(ms) = 1.0 / f * 1000000;
//        if((1000000 % f) != 0) TIM2->ARR += 1;
        TIM2->CCR1 = (TIM2->ARR >> 1);
    }
}


