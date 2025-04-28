#include "led.h"
#include <stdint.h>

void LED_Init(GPIO_TypeDef *port, uint8_t pin) {
    // Enable clock for the given port using switch-case on the port address
    switch ((uint32_t)port) {
        case (uint32_t)GPIOA:
            RCC->AHB1ENR |= (1 << 0);
            break;
        case (uint32_t)GPIOB:
            RCC->AHB1ENR |= (1 << 1);
            break;
        case (uint32_t)GPIOC:
            RCC->AHB1ENR |= (1 << 2);
            break;
        case (uint32_t)GPIOD:
            RCC->AHB1ENR |= (1 << 3);
            break;
        case (uint32_t)GPIOE:
            RCC->AHB1ENR |= (1 << 4);
            break;
        default:
            // Handle unsupported ports if needed
            break;
    }

    // Set the pin to output mode: clear mode bits then set to '01'
    port->MODER &= ~(3 << (pin * 2));
    port->MODER |=  (1 << (pin * 2));
}

void LED_On(GPIO_TypeDef *port, uint8_t pin) {
    port->ODR |= (1 << pin);
}

void LED_Off(GPIO_TypeDef *port, uint8_t pin) {
    port->ODR &= ~(1 << pin);
}