#include "pwm4/pwm4.h"

TIM_HandleTypeDef htim4;
volatile uint32_t PWM4_pulseCount = 0;

/* TIM4 init function */
static void MX_TIM4_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 36-1;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 999;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
//        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
//        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 499;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
//        Error_Handler();
    }
}

void PWM4_config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    /**TIM4 GPIO Configuration    
    PB6     ------> TIM4_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    MX_TIM4_Init();
    
    TIM4->SR &= ~TIM_SR_UIF;
    TIM4->DIER |= TIM_DIER_UIE;
    
    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 2, 2);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/**
 * @brief ����PWM4
 */
void PWM4_start(void)
{
    PWM4_pulseCount = 0;
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
    
}

/**
 * @brief ֹͣPWM4
 */
void PWM4_stop(void)
{
    HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
}

inline void PWM4_setFrequent(uint32_t f)
{
    if(f < 100)                         // 0~99Hz
    {
        TIM4->PSC = 7199;               // ��ʱ��ʱ��10kHz
        TIM4->ARR = 10000 / f - 1;
//        if((1000 % f) != 0) TIM2->ARR += 1;
        TIM4->CCR1 = (TIM4->ARR >> 1);
    }
    else if(f <= 500000)                // 100Hz ~ 500kHz
    {
        TIM4->PSC = 71;                 // ��ʱ��ʱ��1MHz
        TIM4->ARR = 1000000 / f - 1;          // t(ms) = 1.0 / f * 1000000;
//        if((1000000 % f) != 0) TIM2->ARR += 1;
        TIM4->CCR1 = (TIM4->ARR >> 1);
    }
}


