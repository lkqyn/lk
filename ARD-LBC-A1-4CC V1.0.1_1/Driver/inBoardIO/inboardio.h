/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-10     ylj       the first version
 */
#ifndef DRIVERS_INBOARDIO_SENSORGPIO_H_
#define DRIVERS_INBOARDIO_SENSORGPIO_H_

#include "stm32f1xx_hal.h"

void IBIO_config(void);
uint8_t IBIO_getInput(uint8_t no);
uint8_t IBIO_getOutput(uint8_t no);
void IBIO_setOutput(uint8_t no,uint8_t level);
void IBIO_configEXTI(void);

#define IBIO_NOEXTI         0
#define IBIO_INPUT2_EXTI    1
#define IBIO_INPUT3_EXTI    2

#define IBIO_INPUT1_EXTI_ENABLE()   \
			do{\
				EXTI->PR |= EXTI_PR_PR5;\
				EXTI->IMR |= EXTI_IMR_MR5;\
			}while(0)

#define IBIO_INPUT2_EXTI_ENABLE()   \
            do{\
                EXTI->PR |= EXTI_PR_PR4;\
                EXTI->IMR |= EXTI_IMR_MR4;\
            }while(0)
            
#define IBIO_INPUT3_EXTI_ENABLE()   \
            do{\
                EXTI->PR |= EXTI_PR_PR10;\
                EXTI->IMR |= EXTI_IMR_MR10;\
            }while(0)
#define IBIO_INPUT4_EXTI_ENABLE()   \
            do{\
                EXTI->PR |= EXTI_PR_PR15;\
                EXTI->IMR |= EXTI_IMR_MR15;\
            }while(0)
#define IBIO_INPUT2_3_EXTI_ENABLE()   \
            do{\
                EXTI->PR |= EXTI_PR_PR10 | EXTI_PR_PR4;\
                EXTI->IMR |= EXTI_IMR_MR10 | EXTI_IMR_MR4;\
            }while(0)

#define IBIO_INPUT5_EXTI_ENABLE()   \
            do{\
                EXTI->PR |= EXTI_PR_PR14;\
                EXTI->IMR |= EXTI_IMR_MR14;\
            }while(0)

#define IBIO_INPUT6_EXTI_ENABLE()   \
            do{\
                EXTI->PR |= EXTI_PR_PR13;\
                EXTI->IMR |= EXTI_IMR_MR13;\
            }while(0)

#define IBIO_INPUT7_EXTI_ENABLE()   \
            do{\
                EXTI->PR |= EXTI_PR_PR12;\
                EXTI->IMR |= EXTI_IMR_MR12;\
            }while(0)

#define IBIO_INPUT8_EXTI_ENABLE()   \
            do{\
                EXTI->PR |= EXTI_PR_PR11;\
                EXTI->IMR |= EXTI_IMR_MR11;\
            }while(0)

#define IBIO_INPUT1_EXTI_DISABLE()   EXTI->IMR &= ~EXTI_IMR_MR5
#define IBIO_INPUT2_EXTI_DISABLE()   EXTI->IMR &= ~EXTI_IMR_MR4
#define IBIO_INPUT3_EXTI_DISABLE()   EXTI->IMR &= ~EXTI_IMR_MR10
#define IBIO_INPUT4_EXTI_DISABLE()   EXTI->IMR &= ~EXTI_IMR_MR15
#define IBIO_INPUT2_3_EXTI_DISABLE()   EXTI->IMR &= ~(EXTI_IMR_MR4 | EXTI_IMR_MR10)
#define IBIO_INPUT5_EXTI_DISABLE()   EXTI->IMR &= ~EXTI_IMR_MR14
#define IBIO_INPUT6_EXTI_DISABLE()   EXTI->IMR &= ~EXTI_IMR_MR13
#define IBIO_INPUT7_EXTI_DISABLE()   EXTI->IMR &= ~EXTI_IMR_MR12
#define IBIO_INPUT8_EXTI_DISABLE()   EXTI->IMR &= ~EXTI_IMR_MR11
           
            
#endif /* DRIVERS_INBOARDIO_SENSORGPIO_H_ */
