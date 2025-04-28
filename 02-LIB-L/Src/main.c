// ...existing code...
#include "stm32f4xx.h"

int main(){
    // Enable GPIOA clock
    RCC->AHB1ENR |= (1 << 0);
    
    // Set PA5 as output
    GPIOA->MODER &= ~(3 << (8 * 2));
    GPIOA->MODER |=  (1 << (8 * 2));

    while(1){
        GPIOA->ODR ^= (1 << 8); // Toggle PA5
        for(volatile int i = 0; i < 100000; i++); // Crude delay
    }
}
