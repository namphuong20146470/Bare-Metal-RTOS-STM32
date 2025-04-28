#include "usart.h"

volatile char uartBuffer[100];
volatile int uartIndex = 0;
volatile uint8_t commandReady = 0;


void USART2_SetBaudrate(uint32_t baudrate) {
    // Calculate BRR value based on APB1 clock (42 MHz)
    uint32_t apb1_clock = 42000000; // 42 MHz
    
    // Calculate the exact divider value
    float exact_div = (float)apb1_clock / (16.0f * baudrate);
    
    // Calculate mantissa and fraction parts
    uint16_t mantissa = (uint16_t)exact_div;
    uint8_t fraction = (uint8_t)((exact_div - mantissa) * 16.0f + 0.5f); // Round properly
    
    // Check if fraction overflows
    if (fraction >= 16) {
        mantissa++;
        fraction = 0;
    }
    
    // Combine mantissa and fraction into BRR value
    uint16_t brr_value = (mantissa << 4) | (fraction & 0xF);
    
    // Set BRR register
    USART2->BRR = brr_value;
}
void USART2_TXRX_Init(void) {
    // 1) Enable GPIOA clock (for PA2, PA3)
    RCC->AHB1ENR |= (1 << 0);

    // 2) Enable USART2 clock
    RCC->APB1ENR |= (1 << 17);

    // 3) Configure PA2, PA3 as Alternate Function AF7 (USART2)
    GPIOA->MODER  &= ~((3 << (2*2)) | (3 << (3*2)));
    GPIOA->MODER  |=  ((2 << (2*2)) | (2 << (3*2)));
    GPIOA->AFR[0] &= ~((0xF << (2*4)) | (0xF << (3*4)));
    GPIOA->AFR[0] |=  ((7 << (2*4)) | (7 << (3*4)));

    // 4) Cấu hình baud rate - đây có thể là vấn đề!
    // Kiểm tra đồng hồ APB1 và giá trị BRR
    // Với APB1 @ 42 MHz, cho baud 115200, BRR = 22.786 → 0x16E
    USART2_SetBaudrate(115200); 

    // 5) Kích hoạt USART2
    USART2->CR1 |= (1 << 2) | (1 << 3) | (1 << 13);
}

void USART2_SendChar(char ch) {
    // Đợi cho đến khi TXE flag set (Transmit Data Register Empty)
    while (!(USART2->SR & (1 << 7)));
    USART2->DR = (uint8_t)ch;
}

void USART2_SendString(const char *s) {
    // Thêm debug để kiểm tra từng ký tự
    while (*s) {
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