/*
 * spi.c
 *
 *  Created on: Apr 28, 2025
 *      Author: phuong
 *      Implementation: Register-level SPI for SX1278
 */

 #include "spi.h"
 #include "main.h"
 #include "usart.h"
 #include <string.h>
 #include <stdio.h>
 /* Add these headers at the top of the file */
#include "FreeRTOS.h"
#include "task.h"
 /* Private function prototypes */
 static void SX1278_Reset(void);
 static void SX1278_CS_Low(void);
 static void SX1278_CS_High(void);
 
 /* Delay function implementation using SysTick */
 void Delay_ms(uint32_t ms)
 {
     /* Configure SysTick with millisecond resolution */
     SysTick->LOAD = (SystemCoreClock / 1000) - 1;
     SysTick->VAL = 0;
     SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
 
     for (uint32_t i = 0; i < ms; i++)
     {
         /* Wait until the COUNT flag is set */
         while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
     }
     
     /* Disable SysTick when done */
     SysTick->CTRL = 0;
 }
 
 /* Initialize the SPI peripheral for SX1278 communication using registers */
 void SPI1_Init(void)
 {
     /* Enable GPIOA clock */
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
     
     /* Enable SPI1 clock */
     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
     
     /* Configure SCK (PA5), MISO (PA6), MOSI (PA7) pins */
     /* Set mode to Alternate Function (AF) */
     GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
     GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
     
     /* Set output type to push-pull */
     GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
     
     /* Set output speed to high speed */
     GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR7_1);
     
     /* No pull-up, no pull-down */
     GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
     
     /* Configure alternate function AF5 for SPI1 pins */
     GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7);
     GPIOA->AFR[0] |= (5U << GPIO_AFRL_AFSEL5_Pos) | (5U << GPIO_AFRL_AFSEL6_Pos) | 
                      (5U << GPIO_AFRL_AFSEL7_Pos);
     
  /* At the beginning of SPI1_Init function, after configuring SPI pins */

/* Configure CS pin (PA9) as output and set it high initially */
GPIOA->MODER &= ~GPIO_MODER_MODER9;
GPIOA->MODER |= GPIO_MODER_MODER9_0;  /* Output mode */
GPIOA->OTYPER &= ~GPIO_OTYPER_OT9;    /* Push-pull */
GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9; /* High speed */
GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR9;
GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0;  /* Pull-up */
GPIOA->BSRR = LORA_CS_PIN;  /* Set CS high initially */

/* Configure RESET pin (PA10) as output and set it high initially */
GPIOA->MODER &= ~GPIO_MODER_MODER10;
GPIOA->MODER |= GPIO_MODER_MODER10_0; /* Output mode */
GPIOA->OTYPER &= ~GPIO_OTYPER_OT10;   /* Push-pull */
GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR10; /* Low speed */
GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR10;
GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0; /* Pull-up */
GPIOA->BSRR = LORA_RST_PIN;  /* Set RESET high initially */
     
     /* Configure DIO0 pin (PA8) as input */
     GPIOA->MODER &= ~GPIO_MODER_MODER8;  /* Input mode */
     GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR8;
     GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0; /* Pull-up */
     
     /* Set CS high (inactive) by default */
     GPIOA->BSRR = LORA_CS_PIN;
     
     /* Configure SPI1 */
     /* Disable SPI before configuration */
     SPI1->CR1 &= ~SPI_CR1_SPE;
     
     /* Reset SPI */
     RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
     RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
     
     /* Configure SPI1 */
/* Configure SPI1 */
SPI1->CR1 = 0;
SPI1->CR1 |= (
    SPI_CR1_MSTR |           /* Master mode */
    SPI_CR1_SSM |            /* Software slave management enabled */
    SPI_CR1_SSI |            /* Internal slave select */
    (4U << SPI_CR1_BR_Pos)   /* Baud rate: fPCLK/32 */
    /* Clock polarity low (CPOL=0) - omit SPI_CR1_CPOL */
    /* Clock phase first edge (CPHA=0) - omit SPI_CR1_CPHA */
);
     
     /* Enable SPI1 */
     SPI1->CR1 |= SPI_CR1_SPE;
 }
 
 /* SPI transmit and receive one byte using registers */
 uint8_t SPI_TransmitReceive(uint8_t data)
 {
     /* Wait until TXE = 1 (transmit buffer empty) */
     while (!(SPI1->SR & SPI_SR_TXE));
     
     /* Send data */
     *((__IO uint8_t *)&SPI1->DR) = data;
     
     /* Wait until RXNE = 1 (receive buffer not empty) */
     while (!(SPI1->SR & SPI_SR_RXNE));
     
     /* Return received data */
     return (uint8_t)SPI1->DR;
 }
 
 /* Read a register from SX1278 using register-level SPI */
 uint8_t SX1278_ReadRegister(uint8_t reg)
 {
     uint8_t value;
     
     SX1278_CS_Low();
     
     /* Send address with MSB=0 to indicate read operation */
     SPI_TransmitReceive(reg & 0x7F);
     
     /* Read the data (send dummy byte to receive) */
     value = SPI_TransmitReceive(0x00);
     
     SX1278_CS_High();
     
     return value;
 }
 
 /* Write to a register on SX1278 using register-level SPI */
 void SX1278_WriteRegister(uint8_t reg, uint8_t value)
 {
     SX1278_CS_Low();
     
     /* Send address with MSB=1 to indicate write operation */
     SPI_TransmitReceive(reg | 0x80);
     
     /* Write the data */
     SPI_TransmitReceive(value);
     
     SX1278_CS_High();
 }
 
 /* Set the SX1278 to a specific mode */
 void SX1278_SetMode(uint8_t mode)
 {
     SX1278_WriteRegister(REG_OP_MODE, mode);
 }
 
 /* Initialize SX1278 with default configuration for LoRa communication */
 /* Update the SX1278_Init function with more consistent settings */
uint8_t SX1278_Init(void)
{
    char buffer[50];
    
    SPI1_Init();
    
    // Reset the SX1278 module
    SX1278_Reset();
    
    // Allow time for reset to complete
    Delay_ms(10);
    
    /* Check if SX1278 is present by reading version register */
    uint8_t version = SX1278_ReadRegister(REG_VERSION);
    sprintf(buffer, "SX1278 version: 0x%02X\r\n", version);
    USART2_SendString(buffer);
    
    if (version != 0x12) {
        USART2_SendString("SX1278 not found! Version mismatch\r\n");
        return 1;  /* Error: module not detected */
    }
    
    /* Set sleep mode to configure LoRa mode */
    SX1278_SetMode(MODE_SLEEP);
    Delay_ms(10);
    
    /* Set LoRa mode (instead of FSK) */
    SX1278_WriteRegister(REG_OP_MODE, MODE_SLEEP | MODE_LONG_RANGE_MODE);
    Delay_ms(10);
    
    /* Set frequency to 433 MHz - MAKE SURE ESP32 MATCHES THIS */
    /* 433MHz = 433000000 Hz / 61.035 Hz = 7094272 = 0x6C8000 */
    SX1278_WriteRegister(REG_FRF_MSB, 0x6C);
    SX1278_WriteRegister(REG_FRF_MID, 0x80);
    SX1278_WriteRegister(REG_FRF_LSB, 0x00);
    USART2_SendString("Frequency set to 433 MHz\r\n");
    
    /* Set output power to 17 dBm using PA_BOOST */
    SX1278_WriteRegister(REG_PA_CONFIG, PA_BOOST | 0x0F); 
    USART2_SendString("Power output set to 17 dBm\r\n");
    
    /* Improve PA performance for +20 dBm operation */
    SX1278_WriteRegister(REG_PA_DAC, 0x87);
    
    /* Configure modulation settings */
    /* BW=125kHz, CR=4/5, Explicit header mode */
    SX1278_WriteRegister(REG_MODEM_CONFIG1, 0x72);
    
    /* SF=7, normal mode, CRC enabled, RX timeout MSB */
    SX1278_WriteRegister(REG_MODEM_CONFIG2, 0x74);
    USART2_SendString("Bandwidth: 125kHz, SF=7, CR=4/5\r\n");
    
    /* Low Data Rate Optimize enabled, LNA gain set by AGC loop */
    SX1278_WriteRegister(REG_MODEM_CONFIG3, 0x04);
    
    /* Set preamble length to 8 */
    SX1278_WriteRegister(REG_PREAMBLE_MSB, 0x00);
    SX1278_WriteRegister(REG_PREAMBLE_LSB, 0x08);
    USART2_SendString("Preamble length: 8\r\n");
    
    /* Set sync word - CRITICAL for devices to communicate */
    SX1278_WriteRegister(REG_SYNC_WORD, 0x12);
    USART2_SendString("Sync word: 0x12\r\n");
    
    /* Set FIFO pointers */
    SX1278_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
    SX1278_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
    
    /* Set device to standby mode */
    SX1278_SetMode(MODE_STDBY);
    
    USART2_SendString("SX1278 initialized successfully\r\n");
    return 0;  /* Success */
}
 
 /* Send data using SX1278 in LoRa mode */
/* Send data using SX1278 in LoRa mode */
/* Send data using SX1278 in LoRa mode */
void SX1278_Send(uint8_t *data, uint8_t size)
{
    uint8_t i;
    char debug[50];
    uint32_t timeout_counter;
    
    USART2_SendString("===== STARTING LORA TRANSMISSION =====\r\n");
    
    /* Set device to standby mode first */
    SX1278_SetMode(MODE_STDBY);
    Delay_ms(10); // Short delay for mode change
    USART2_SendString("SX1278 in standby mode\r\n");
    
    /* Set FIFO address pointer to beginning of TX part */
    SX1278_WriteRegister(REG_FIFO_ADDR_PTR, 0x00);
    USART2_SendString("FIFO pointer set\r\n");
    
    /* Write data to FIFO */
    SX1278_CS_Low();
    
    /* Send the FIFO address with write bit set */
    SPI_TransmitReceive(REG_FIFO | 0x80);
    
    /* Send all data bytes and print them */
    USART2_SendString("Data bytes: ");
    for (i = 0; i < size; i++) {
        SPI_TransmitReceive(data[i]);
        sprintf(debug, "%02X ", data[i]);
        USART2_SendString(debug);
    }
    USART2_SendString("\r\n");
    
    SX1278_CS_High();
    USART2_SendString("Data written to FIFO\r\n");
    
    /* Set payload length */
    SX1278_WriteRegister(REG_PAYLOAD_LENGTH, size);
    sprintf(debug, "Payload length set to %d\r\n", size);
    USART2_SendString(debug);
    
    /* Set TX mode to start transmission */
    SX1278_SetMode(MODE_TX);
    USART2_SendString("SX1278 in TX mode - sending packet...\r\n");
    
    /* Wait for TX to complete with timeout using a simple counter */
    uint8_t irqFlags;
    timeout_counter = 0;
    uint32_t max_timeout = 20; // Short timeout for testing
    
    USART2_SendString("Waiting for TX to complete...\r\n");
    do {
        irqFlags = SX1278_ReadRegister(REG_IRQ_FLAGS);
        sprintf(debug, "IRQ flags: 0x%02X\r\n", irqFlags);
        USART2_SendString(debug);
        
        // Display TX Done bit specifically
        if (irqFlags & 0x08) {
            USART2_SendString("TX Done flag detected!\r\n");
            break;
        }
        
        Delay_ms(50);
        
        timeout_counter++;
        if(timeout_counter > max_timeout) {
            USART2_SendString("TX timeout - forcing continue\r\n");
            break;
        }
    } while (1); // Continue until TX Done flag or timeout
    
    /* Clear IRQ flags */
    SX1278_WriteRegister(REG_IRQ_FLAGS, 0xFF);
    USART2_SendString("IRQ flags cleared\r\n");
    
    /* Set device back to standby mode */
    SX1278_SetMode(MODE_STDBY);
    USART2_SendString("SX1278 back to standby mode\r\n");
    USART2_SendString("===== TRANSMISSION COMPLETED =====\r\n");
}
/* Reset the SX1278 module */
static void SX1278_Reset(void)
{
    USART2_SendString("Resetting SX1278...\r\n");
    
    /* Reset pin low */
    GPIOA->BSRR = (LORA_RST_PIN << 16); /* Set bit in the high part of BSRR to reset */
    Delay_ms(1);
    
    /* Reset pin high */
    GPIOA->BSRR = LORA_RST_PIN;
    Delay_ms(5);
    
    USART2_SendString("SX1278 reset complete\r\n");
}
 /* Set CS pin low (active) */
 static void SX1278_CS_Low(void)
 {
     GPIOA->BSRR = (LORA_CS_PIN << 16); /* Set bit in the high part of BSRR to reset */
 }
 
 /* Set CS pin high (inactive) */
 static void SX1278_CS_High(void)
 {
     GPIOA->BSRR = LORA_CS_PIN;
 }