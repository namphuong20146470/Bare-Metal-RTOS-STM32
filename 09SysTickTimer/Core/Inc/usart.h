#ifndef USART_H
#define USART_H

#include "stm32f4xx.h"
#define F (42*10^6)
void USART2_TXRX_Init(void);
void USART2_SendChar(char ch);
void USART2_SendString(const char *s);
void USART2_SetBaudrate(uint32_t baudrate);
extern volatile char uartBuffer[100];  // Buffer for received string
extern volatile uint8_t commandReady;  // Flag set when a complete command is received

#endif