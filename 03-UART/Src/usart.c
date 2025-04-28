#include "usart.h"

void USART2_TXRX_Init(void){
    // 1) Enable GPIOA clock (for PA2, PA3)
    RCC->AHB1ENR |= (1 << 0);

    // 2) Enable USART2 clock
    RCC->APB1ENR |= (1 << 17);

    // 3) Configure PA2, PA3 as Alternate Function AF7 (USART2)
    GPIOA->MODER  &= ~((3 << (2*2)) | (3 << (3*2)));
    GPIOA->MODER  |=  ((2 << (2*2)) | (2 << (3*2)));
    GPIOA->AFR[0] &= ~((0xF << (2*4)) | (0xF << (3*4)));
    GPIOA->AFR[0] |=  ((7 << (2*4)) | (7 << (3*4)));

    // 4) Configure baud rate (APB1 @ 16 MHz -> ~9600 baud)
    USART2->BRR = 0x683;

    // 5) Enable USART2, TX, RX
    USART2->CR1 |= (1 << 2) | (1 << 3) | (1 << 13);
}

void USART2_SendChar(char ch){
    // Wait for TXE (Transmit Data Register Empty)
    while(!(USART2->SR & (1 << 7)));
    USART2->DR = (uint8_t)ch;
}

// Use a loop to send each character in "key"
void USART2_SendString(const char *s){
    while(*s){
        USART2_SendChar(*s++);
    }
}
char USART2_GetChar(void){
    // Wait for RXNE (Received Data Ready to be Read)
    while(!(USART2->SR & (1 << 5)));
    return (char)USART2->DR;
}

// ...existing code...
char USART2_GetString(char *buffer, int maxLen){
    int i = 0;
    while(1){
        char ch = USART2_GetChar();
        if(ch == '\r' || ch == '\n'){
            buffer[i] = '\0';
            break;
        }
        if(i < (maxLen - 1)){
            buffer[i++] = ch;
        } else {
            buffer[i] = '\0';
            break;
        }
    }
    return i; // Number of chars read
}
