#include "adc.h"

void ADC1_Init(void) {
    // Enable clocks for GPIOA and ADC1
    RCC->AHB1ENR |= (1 << 0);      // GPIOA clock enable
    RCC->APB2ENR |= (1 << 8);      // ADC1 clock enable
    
    // Configure PA0 as analog mode
    GPIOA->MODER &= ~(3 << (0 * 2));
    GPIOA->MODER |= (3 << (0 * 2));  // Set PA0 to analog
    
    // Set sample time for ADC1 channel 0 (using maximum cycles for better accuracy)
    ADC1->SMPR2 &= ~(7 << 0);       // Clear sample time selection bits for channel 0
    ADC1->SMPR2 |= (7 << 0);        // Set sample time to 480 cycles
    
    // Turn on ADC1 (set ADON bit in CR2)
    ADC1->CR2 |= (1 << 0);
    
    // Give ADC some time to settle
    for(volatile int i = 0; i < 1000; i++);
}

uint16_t ADC1_Read(void) {
    // Start conversion by setting SWSTART bit (bit 30 in CR2)
    ADC1->CR2 |= (1 << 30);
    
    // Wait until conversion is complete (EOC flag in SR, bit 1)
    while (!(ADC1->SR & (1 << 1)));;
    
    // Return the conversion result from DR
    return ADC1->DR;
}