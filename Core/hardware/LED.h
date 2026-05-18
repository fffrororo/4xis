#ifndef __LED_H__
#define __LED_H__ 

#include "main.h"


typedef struct{
    GPIO_TypeDef *gpio;
    uint16_t pin;
}LED_STRUCT;

void LED_ON(LED_STRUCT *led);
void LED_OFF(LED_STRUCT *led);
void LED_Toggle(LED_STRUCT *led);

#endif