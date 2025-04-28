#include "stm32f4xx.h"
#include "usart.h"
#include "led.h"
#include "adc.h"
#include <string.h>
#include <stdio.h>

char buffer[30];

int main(void) {
    USART2_TXRX_Init();
    ADC1_Init();
    
    USART2_SendString("Voltage: \r\n");
    
    while(1) {
        uint16_t adcValue = ADC1_Read();
        
        // Chuyển đổi thành millivolt (không cần float)
        uint32_t millivolts = (adcValue * 3300) / 4096;
        uint16_t whole = millivolts / 1000;
        uint16_t frac = (millivolts % 1000) / 10; // Lấy 2 số thập phân
        
        // Format thủ công: "x.yy V"
        sprintf(buffer, "ADC Value: %u raw, %u.%02u V\r\n", adcValue, whole, frac);
        USART2_SendString(buffer);
        
        for(volatile int i = 0; i < 100000; i++);
    }
    
    return 0;
}