#include "LED.h"

void LED_ON(LED_STRUCT *led)
{
    HAL_GPIO_WritePin(led->gpio, led->pin, GPIO_PIN_RESET);
}

void LED_OFF(LED_STRUCT *led)
{
    HAL_GPIO_WritePin(led->gpio, led->pin, GPIO_PIN_SET);
}

void LED_Toggle(LED_STRUCT *led)
{
    HAL_GPIO_TogglePin(led->gpio, led->pin);
}
