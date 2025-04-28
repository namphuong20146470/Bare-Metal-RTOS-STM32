#include "stm32f4xx.h"
#include "usart.h"
#include "led.h"
#include <stdlib.h>
#include <string.h>

#define PA5 5    // LED1 on PA5
#define PA8 8   // LED2 on PB10
#define PA9 9   // LED2 on PB10
#define PA10 10   // LED2 on PB10
int main(void){
    // Initialize USART2
    USART2_TXRX_Init();
    
    // Initialize LED1 on PA5 and LED2 on PB10
    LED_Init(GPIOA, PA5);
    LED_Init(GPIOA, PA8);
    LED_Init(GPIOA, PA9);
    LED_Init(GPIOA, PA10);
    
    // Allocate memory for 100 characters dynamically
    char *key = malloc(100 * sizeof(char));
    if(key == NULL){
        // Handle allocation error
        while(1);
    }

    while(1){
        // Get string from UART
        USART2_SendString("Nhap onRed, onGreen, onBlue, off: ");
        USART2_GetString(key, 100);
        USART2_SendString(key);
        USART2_SendChar('\n');
        // USART2_SendChar("Nhap onRed, onGreen, onBlue, off\n");
        // If received string is "on", turn both LEDs on.
        // If "off", turn both LEDs off.
        if(strcmp(key, "onRed") == 0) {
            LED_On(GPIOA, PA5);
            LED_On(GPIOA, PA8);
            LED_Off(GPIOA, PA9);
            LED_Off(GPIOA, PA10);
        }
        else if(strcmp(key, "onGreen") == 0) {
            LED_On(GPIOA, PA5);
            LED_On(GPIOA, PA9);
            LED_Off(GPIOA, PA8);
            LED_Off(GPIOA, PA10);
        }
        else if(strcmp(key, "onBlue") == 0) {
            LED_On(GPIOA, PA5);
            LED_Off(GPIOA, PA8);
            LED_Off(GPIOA, PA9);
            LED_On(GPIOA, PA10);
        }

        else if (strcmp(key, "off") == 0)
        {
            LED_Off(GPIOA, PA5);
            LED_Off(GPIOA, PA8);
            LED_Off(GPIOA, PA9);
            LED_Off(GPIOA, PA10);
        }
        
    }
    
    // Normally unreachable in an embedded infinite loop, but added for completeness
    free(key);
    return 0;
}