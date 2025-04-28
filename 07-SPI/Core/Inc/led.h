#ifndef LED_H
#define LED_H

#include "stm32f4xx.h"

// Initialize a LED pin
// port: GPIOA, GPIOB, ...
// pin: the pin number (e.g., 5 for PA5, 10 for PB10)
void LED_Init(GPIO_TypeDef *port, uint8_t pin);

// Turn LED on
void LED_On(GPIO_TypeDef *port, uint8_t pin);

// Turn LED off
void LED_Off(GPIO_TypeDef *port, uint8_t pin);

#endif