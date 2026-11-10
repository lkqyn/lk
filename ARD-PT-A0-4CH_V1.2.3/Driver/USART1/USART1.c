/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-01     ylj       the first version
 */

#include <usart1/USART1.h>
#include "string.h"

char USART1_DMATX_buffer[30];

//void DMA_Channel_7_config(void)
//{
//    __HAL_RCC_DMA1_CLK_ENABLE();
//
//    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 1, 0);
//    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
//
//    USART2->CR3 |= USART_CR3_DMAT;
//
//    DMA1_Channel7->CPAR = (uint32_t)&(USART2->DR);
//    DMA1_Channel7->CMAR = (uint32_t)USART2_DMATX_buffer;
//    DMA1_Channel7->CCR |= DMA_CCR_MINC | DMA_CCR_DIR /*| DMA_CCR_TCIE*/;
//}

void USART1_config(void)
{
    UART_HandleTypeDef huart1;
    GPIO_InitTypeDef gpio;

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pin = GPIO_PIN_9;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Mode = GPIO_MODE_AF_INPUT;
    gpio.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    USART1->SR &= ~USART_SR_RXNE;
    __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);

    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn,1,3);

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {

    }
    
//    DMA_Channel_7_config();
}

void USART1_sendBuf(char *p,uint8_t len)
{
    for(;len>0;len--)
    {
        USART1->DR = *p;
        p++;
        while((USART1->SR & UART_FLAG_TXE) != UART_FLAG_TXE);
    }
}

//void USART2_sendBufDMA(char *p,uint8_t len)
//{
//    memcpy(USART2_DMATX_buffer,p,len);
//
//    DMA1_Channel7->CCR &= ~DMA_CCR_EN;
//    USART2->SR &= ~USART_SR_TC;
//    DMA1_Channel7->CNDTR = len;
//    DMA1_Channel7->CCR |= DMA_CCR_EN;
//
//    while((DMA1->ISR & DMA_ISR_TCIF7) != DMA_ISR_TCIF7);
//}

//void DMA1_Channel7_IRQHandler(void)
//{
//    DMA1->IFCR |= DMA_IFCR_CGIF7;
//}


