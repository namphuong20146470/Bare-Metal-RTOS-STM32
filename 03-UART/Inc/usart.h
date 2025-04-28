#ifndef USART_H
#define USART_H

#include "stm32f4xx.h"

void USART2_TXRX_Init(void);
void USART2_SendChar(char ch);
char USART2_GetChar(void);
void USART2_SendString(const char *s);
char USART2_GetString(char *buffer, int maxLen);

#endif