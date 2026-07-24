/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-10     ylj       the first version
 */
#ifndef DRIVERS_EXIO_INBOARDIO_H_
#define DRIVERS_EXIO_INBOARDIO_H_

#include "stm32f1xx_hal.h"

void EXIO_config(void);
void EXIO_setOutput(uint8_t no,uint8_t level);
uint8_t EXIO_getOutput(uint8_t no);
uint8_t EXIO_getInput(uint8_t no);

#define EXIO_enableEXI2Interrupt()          do{\
                EXTI->PR |= EXTI_PR_PR1;\
                EXTI->IMR |= EXTI_IMR_MR1;\
            }while(0)

#define EXIO_disableEXI2Interrupt()          EXTI->IMR &= ~EXTI_IMR_MR1

#define EXIO_enableEXI1Interrupt()          do{\
                EXTI->PR |= EXTI_PR_PR2;\
                EXTI->IMR |= EXTI_IMR_MR2;\
            }while(0)

#define EXIO_disableEXI1Interrupt()          EXTI->IMR &= ~EXTI_IMR_MR2

#define EXIO_enableEXI0Interrupt()          do{\
                EXTI->PR |= EXTI_PR_PR3;\
                EXTI->IMR |= EXTI_IMR_MR3;\
            }while(0)

#define EXIO_disableEXI0Interrupt()          EXTI->IMR &= ~EXTI_IMR_MR3

#endif /* DRIVERS_EXIO_INBOARDIO_H_ */
