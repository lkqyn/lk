#include "pwm3/pwm3.h"

TIM_HandleTypeDef htim3;
volatile uint32_t PWM3_pulseCount = 0;

static void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 36-1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 9999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        //Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        //Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 4999;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        //Error_Handler();
    }
}

void PWM3_config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**TIM3 GPIO Configuration    
    PB0     ------> TIM3_CH3 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    MX_TIM3_Init();
    
//    TIM3->SR &= ~TIM_SR_UIF;
//    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->SR &= ~TIM_SR_UIF;
    TIM3->DIER |= TIM_DIER_UIE;
    
    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
 * @brief ����PWM3
 */
void PWM3_start(void)
{
    PWM3_pulseCount = 0;
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
    
}

/**
 * @brief ֹͣPWM3
 */
void PWM3_stop(void)
{
    HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
}

inline void PWM3_setFrequent(uint32_t f)
{
    if(f == 0)
    {
        return;
    }
    if(f < 100)                         // 0~99Hz
    {
        TIM3->PSC = 7199;               // ��ʱ��ʱ��10kHz
        TIM3->ARR = 10000 / f - 1;
//        if((1000 % f) != 0) TIM2->ARR += 1;
        TIM3->CCR3 = (TIM3->ARR >> 1);
    }
    else if(f <= 500000)                // 100Hz ~ 500kHz
    {
        TIM3->PSC = 71;                 // ��ʱ��ʱ��1MHz
        TIM3->ARR = 1000000 / f - 1;          // t(ms) = 1.0 / f * 1000000;
//        if((1000000 % f) != 0) TIM2->ARR += 1;
        TIM3->CCR3 = (TIM3->ARR >> 1);
    }
}

