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
void LED_ShowFaultCode(uint8_t code, uint32_t now_ms)
{
    static uint32_t last_tick = 0;
    static uint8_t blink_count = 0;
    static uint8_t led_is_on = 0;
    static uint8_t pause_state = 0;
    const uint32_t blink_ms = 300U;
    const uint32_t pause_ms = 1500U;

    if (code == 0U)
    {
        LED_SetNormal();
        blink_count = 0;
        led_is_on = 0;
        pause_state = 0;
        last_tick = now_ms;
        return;
    }

    LED_OFF(LED_G_GPIO_Port, LED_G_Pin);

    if (pause_state)
    {
        LED_OFF(LED_R_GPIO_Port, LED_R_Pin);
        if ((now_ms - last_tick) >= pause_ms)
        {
            pause_state = 0;
            blink_count = 0;
            led_is_on = 0;
            last_tick = now_ms;
        }
        return;
    }

    if ((now_ms - last_tick) < blink_ms)
    {
        return;
    }

    last_tick = now_ms;

    if (led_is_on)
    {
        LED_OFF(LED_R_GPIO_Port, LED_R_Pin);
        led_is_on = 0;
        blink_count++;

        if (blink_count >= code)
        {
            pause_state = 1;
            last_tick = now_ms;
        }
    }
    else
    {
        LED_ON(LED_R_GPIO_Port, LED_R_Pin);
        led_is_on = 1;
    }
}