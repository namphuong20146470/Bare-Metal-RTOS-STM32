/*
 * spi.h
 *
 *  Created on: Apr 28, 2025
 *      Author: phuong
 */

 #ifndef INC_SPI_H_
 #define INC_SPI_H_
 
 #include "stm32f4xx.h"
 
 /* SX1278 registers */
 #define REG_FIFO                 0x00
 #define REG_OP_MODE              0x01
 #define REG_FRF_MSB              0x06
 #define REG_FRF_MID              0x07
 #define REG_FRF_LSB              0x08
 #define REG_PA_CONFIG            0x09
 #define REG_LNA                  0x0C
 #define REG_FIFO_ADDR_PTR        0x0D
 #define REG_FIFO_TX_BASE_ADDR    0x0E
 #define REG_FIFO_RX_BASE_ADDR    0x0F
 #define REG_IRQ_FLAGS            0x12
 #define REG_MODEM_CONFIG1        0x1D
 #define REG_MODEM_CONFIG2        0x1E
 #define REG_PAYLOAD_LENGTH       0x22
 #define REG_PREAMBLE_MSB         0x20
 #define REG_PREAMBLE_LSB         0x21
 #define REG_SYNC_WORD            0x39
 #define REG_MODEM_CONFIG3        0x26
 #define REG_DIO_MAPPING1         0x40
 #define REG_VERSION              0x42
 #define REG_PA_DAC               0x4D
 
 /* SX1278 operation modes */
 #define MODE_SLEEP               0x00
 #define MODE_STDBY               0x01
 #define MODE_TX                  0x03
 #define MODE_RX_CONTINUOUS       0x05
 #define MODE_RX_SINGLE           0x06
 #define MODE_LONG_RANGE_MODE     0x80
 
 /* SX1278 PA config */
 #define PA_BOOST                 0x80
 
 /* Pin definitions */
 #define LORA_SCK_PIN             (1UL << 5)   // PA5
 #define LORA_MISO_PIN            (1UL << 6)   // PA6
 #define LORA_MOSI_PIN            (1UL << 7)   // PA7
 #define LORA_CS_PIN              (1UL << 9)   // PA9
 #define LORA_RST_PIN             (1UL << 10)  // PA10
 #define LORA_DIO0_PIN            (1UL << 8)   // PA8
 
 /* Initialize SPI for SX1278 communication */
 void SPI1_Init(void);
 
 /* Initialize SX1278 module */
 uint8_t SX1278_Init(void);
 
 /* Send data using SX1278 */
 void SX1278_Send(uint8_t *data, uint8_t size);
 
 /* SX1278 helper functions */
 void SX1278_SetMode(uint8_t mode);
 uint8_t SX1278_ReadRegister(uint8_t reg);
 void SX1278_WriteRegister(uint8_t reg, uint8_t value);
 
 /* Delay function using registers */
 void Delay_ms(uint32_t ms);
 
 #endif /* INC_SPI_H_ */