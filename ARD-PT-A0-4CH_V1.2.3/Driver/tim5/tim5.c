#include "tim5/tim5.h"

TIM_HandleTypeDef htim5;

static void MX_TIM5_Init(void)
{

    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 7199;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 4999;	//4999
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
//        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
    {
//        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
//        Error_Handler();
    }
    /* USER CODE BEGIN TIM5_Init 2 */

    /* USER CODE END TIM5_Init 2 */

}

void TIM5_config(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();
    
    MX_TIM5_Init();

    TIM5->SR &= ~TIM_SR_UIF;
    TIM5->DIER |= TIM_DIER_UIE; //ʹ��TIM5 UIE�ж�
    
    /* TIM5 interrupt Init */
    HAL_NVIC_SetPriority(TIM5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

void TIM5_start(void)
{
    HAL_TIM_Base_Start(&htim5);
}

void TIM5_stop(void)
{
    HAL_TIM_Base_Stop(&htim5);
}


