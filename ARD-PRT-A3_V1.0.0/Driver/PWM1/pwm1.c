/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-08     ylj       the first version
 */

#include "PWM1/pwm1.h"

TIM_HandleTypeDef htim1;

volatile uint32_t PWM1_pulseCount = 0;

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 359;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 9999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
//        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
//        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
//        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
//        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 4999;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
//        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
//        Error_Handler();
    }

}

/**
 * @brief PWM1 ����
 */
void PWM1_config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    
    MX_TIM1_Init();
    
//    TIM1->SR &= ~TIM_SR_UIF; //����жϱ�־λ
//    TIM1->DIER |= TIM_DIER_UIE; // ʹ���ж�
    
    TIM1->SR &= ~TIM_SR_CC1IF;
    TIM1->DIER |= TIM_DIER_CC1IE;
    
//    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 1, 0);
//    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief ����PWM1
 */
void PWM1_start(void)
{
    PWM1_pulseCount = 0;
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    
}

/**
 * @brief ֹͣPWM1
 */
void PWM1_stop(void)
{
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
}

/**
 * @brief ����PWM1Ƶ��
 * @param f- �趨Ƶ�� ��λHz
 * @warning ʵ��Ƶ�ʻ������
 * @note �ú���ִ��ʱ�侭��ʱ���ⶨ������1us, ����תʱ��. 
 */
inline void PWM1_setFrequent(uint32_t f)
{
    if(f < 100)                         // 0~99Hz
    {
        TIM1->PSC = 7199;               // ��ʱ��ʱ��10kHz
        TIM1->ARR = 10000 / f - 1;
        TIM1->CCR1 = (TIM1->ARR >> 1);
    }
    else if(f <= 500000)                // 100Hz ~ 500kHz
    {
        TIM1->PSC = 71;                 // ��ʱ��ʱ��1MHz
        TIM1->ARR = 1000000 / f - 1;          // t(ms) = 1.0 / f * 1000000;
        TIM1->CCR1 = (TIM1->ARR >> 1);
    }
}

/**
 * @brief �������������
 */
void PWM1_clearCount(void)
{
    PWM1_pulseCount = 0;
}


