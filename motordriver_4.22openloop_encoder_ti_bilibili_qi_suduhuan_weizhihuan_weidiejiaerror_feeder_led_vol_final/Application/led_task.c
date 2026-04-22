#include "led_task.h"

#define LED_ON(port, pin)   HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)
#define LED_OFF(port, pin)  HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)

void LED_Task_Init(void)
{
    LED_SetNormal();
}

void LED_SetNormal(void)
{
    /* Õý³££ºÂÌµÆÁÁ£¬ºìµÆÃð */
    LED_OFF(LED_R_GPIO_Port, LED_R_Pin);
    LED_ON(LED_G_GPIO_Port, LED_G_Pin);
}

void LED_SetFault(void)
{
    /* ¹ÊÕÏ£ººìµÆÁÁ£¬ÂÌµÆÃð */
    LED_ON(LED_R_GPIO_Port, LED_R_Pin);
    LED_OFF(LED_G_GPIO_Port, LED_G_Pin);
}

void LED_SetAllOff(void)
{
    LED_OFF(LED_R_GPIO_Port, LED_R_Pin);
    LED_OFF(LED_G_GPIO_Port, LED_G_Pin);
}