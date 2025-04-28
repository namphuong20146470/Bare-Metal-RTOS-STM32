#include "stm32f4xx.h"
#include "usart.h"
#include "led.h"
#include <string.h>

#define PA5 5    // LED1 on PA5
#define PA8 8    // LED2 on PB10 (e.g., Red)
#define PA9 9    // LED3 on PA10 (e.g., Green) – adjust as needed
#define PA10 10  // LED4 on PB10 (e.g., Blue) – adjust as needed

int main(void){
    // Initialize USART2 and LEDs
    USART2_TXRX_Init();
    LED_Init(GPIOA, PA5);
    LED_Init(GPIOA, PA8);
    LED_Init(GPIOA, PA9);
    LED_Init(GPIOA, PA10);

    // Prompt the user
    USART2_SendString("Nhap onRed, onGreen, onBlue, off: \r\n");
    
    while(1){
        // Check if a complete command has been received by the UART interrupt
        if(commandReady){
            // Echo the received command
            USART2_SendString(uartBuffer);
            USART2_SendChar('\n');
            
            // Process command and control LEDs accordingly
            if(strcmp((char *)uartBuffer, "onRed") == 0) {
                LED_On(GPIOA, PA5);
                LED_On(GPIOA, PA8);
                LED_Off(GPIOA, PA9);
                LED_Off(GPIOA, PA10);
            }
            else if(strcmp((char *)uartBuffer, "onGreen") == 0) {
                LED_On(GPIOA, PA5);
                LED_On(GPIOA, PA9);
                LED_Off(GPIOA, PA8);
                LED_Off(GPIOA, PA10);
            }
            else if(strcmp((char *)uartBuffer, "onBlue") == 0) {
                LED_On(GPIOA, PA5);
                LED_Off(GPIOA, PA8);
                LED_Off(GPIOA, PA9);
                LED_On(GPIOA, PA10);
            }
            else if(strcmp((char *)uartBuffer, "off") == 0) {
                LED_Off(GPIOA, PA5);
                LED_Off(GPIOA, PA8);
                LED_Off(GPIOA, PA9);
                LED_Off(GPIOA, PA10);
            }
            // Reset command flag and prompt again
            commandReady = 0;
            USART2_SendString("\r\nNhap onRed, onGreen, onBlue, off: \r\n");
        }
    }
    
    return 0;
}