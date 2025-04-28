/*
 * spi.h
 *
 *  Created on: Apr 26, 2025
 *      Author: phuong
 */

 #ifndef SPI_H_
 #define SPI_H_
 
 #include "stm32f4xx.h"
 #include <stdint.h>
 
 // SX1278 register addresses
 #define REG_FIFO                 0x00
 #define REG_OP_MODE              0x01
 #define REG_FRF_MSB              0x06
 #define REG_FRF_MID              0x07
 #define REG_FRF_LSB              0x08
 #define REG_PA_CONFIG            0x09
 #define REG_PA_RAMP              0x0A
 #define REG_OCP                  0x0B
 #define REG_LNA                  0x0C
 #define REG_FIFO_ADDR_PTR        0x0D
 #define REG_FIFO_TX_BASE_ADDR    0x0E
 #define REG_FIFO_RX_BASE_ADDR    0x0F
 #define REG_FIFO_RX_CURRENT_ADDR 0x10
 #define REG_IRQ_FLAGS_MASK       0x11
 #define REG_IRQ_FLAGS            0x12
 #define REG_RX_NB_BYTES          0x13
 #define REG_MODEM_CONFIG_1       0x1D
 #define REG_MODEM_CONFIG_2       0x1E
 #define REG_MODEM_CONFIG_3       0x26
 #define REG_PREAMBLE_MSB         0x20
 #define REG_PREAMBLE_LSB         0x21
 #define REG_PAYLOAD_LENGTH       0x22
 #define REG_MAX_PAYLOAD_LENGTH   0x23
 #define REG_HOP_PERIOD           0x24
 #define REG_SYNC_WORD            0x39
 #define REG_DIO_MAPPING_1        0x40
 #define REG_VERSION              0x42
 
 // LoRa IRQ Flags
 #define IRQ_TX_DONE_MASK           0x08
 #define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
 #define IRQ_RX_DONE_MASK           0x40
 
 // SX1278 modes
 #define MODE_SLEEP               0x00
 #define MODE_STDBY               0x01
 #define MODE_TX                  0x03
 #define MODE_RX_CONTINUOUS       0x05
 
 // SPI interface functions
 void SPI1_Init(void);
 void SPI1_CS_High(void);
 void SPI1_CS_Low(void);
 uint8_t SPI1_Transfer(uint8_t data);
 void SPI1_DirectGPIO_Init(void);
 uint8_t SPI1_DirectGPIO_Transfer(uint8_t data);
 void SX1278_HardwareTest(void);
 
 // SX1278 LoRa functions
 void SX1278_Init(void);
 void SX1278_Reset(void);
 void SX1278_ForceWriteReg(uint8_t reg, uint8_t value);
 void SX1278_WriteReg(uint8_t reg, uint8_t value);
 uint8_t SX1278_ReadReg(uint8_t reg);
 void SX1278_SetMode(uint8_t mode);
 void SX1278_SetFrequency(float freq);
 void SX1278_Send(uint8_t* data, uint8_t size);
 uint8_t SX1278_Receive(uint8_t* data, uint8_t size);
 uint8_t SX1278_Available(void);
 void SX1278_DiagnosticTest(void);
 
 // Utility function
 void Delay_ms(uint32_t ms);
 
 #endif /* SPI_H_ */