#include "usart.h"

volatile char uartBuffer[100];
volatile int uartIndex = 0;
volatile uint8_t commandReady = 0;

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

    // 5) Enable RXNE (Receive not empty) interrupt
    USART2->CR1 |= (1 << 5);

    // 6) Enable USART2 transmitter, receiver, and USART peripheral
    USART2->CR1 |= (1 << 2) | (1 << 3) | (1 << 13);

    // 7) Enable USART2 interrupt in NVIC
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_SendChar(char ch){
    // Wait for TXE (Transmit Data Register Empty)
    while(!(USART2->SR & (1 << 7)));
    USART2->DR = (uint8_t)ch;
}

void USART2_SendString(const char *s){
    while(*s){
        USART2_SendChar(*s++);
    }
}

// USART2 Interrupt Handler: stores received characters into uartBuffer
void USART2_IRQHandler(void) {
    if(USART2->SR & (1 << 5)) { // RXNE flag is set
        char ch = (char)USART2->DR;  // Reading DR clears the RXNE flag
        if(ch == '\r' || ch == '\n'){
            uartBuffer[uartIndex] = '\0';
            uartIndex = 0;
            commandReady = 1;
        } else {
            if(uartIndex < 99){ // Prevent buffer overflow
                uartBuffer[uartIndex++] = ch;
            }
        }
    }
}