/*
 * spi.c
 *
 *  Created on: Apr 26, 2025
 *      Author: phuong
 */

 #include "spi.h"
 #include "stm32f4xx.h"
 #include <stdio.h>
 #include "usart.h"
 #include <string.h>
 
 char debug_buffer[100];
 
 // Simple delay function
 void Delay_ms(uint32_t ms) {
     // Assuming 84MHz system clock, rough delay
     ms *= 21000; // Adjust based on actual clock
     for(uint32_t i = 0; i < ms; i++) {
         __NOP(); // No operation
     }
 }
 
 // Initialize direct GPIO pins for manual bit-banging
 void SPI1_DirectGPIO_Init(void) {
     USART2_SendString("Initializing direct GPIO pins for SPI...\r\n");
     
     // Enable clock for GPIOA
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
     
     // Configure pin modes for GPIO control
     // PA5 (SCK), PA7 (MOSI) as outputs
     GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk);
     GPIOA->MODER |= GPIO_MODER_MODER5_0 | GPIO_MODER_MODER7_0; // Output mode (01)
     
     // PA6 (MISO) as input
     GPIOA->MODER &= ~GPIO_MODER_MODER6_Msk; // Input mode (00)
     GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD6_Msk;
     GPIOA->PUPDR |= GPIO_PUPDR_PUPD6_0; // Pull-up (01)
     
     // CS pin (PA9) as output
     GPIOA->MODER &= ~GPIO_MODER_MODER9_Msk;
     GPIOA->MODER |= GPIO_MODER_MODER9_0; // Output mode (01)
     
     // RST pin (PA10) as output
     GPIOA->MODER &= ~GPIO_MODER_MODER10_Msk;
     GPIOA->MODER |= GPIO_MODER_MODER10_0; // Output mode (01)
     
     // Set initial pin states
     GPIOA->BSRR = GPIO_BSRR_BS9;  // CS high
     GPIOA->BSRR = GPIO_BSRR_BS5;  // SCK high
     GPIOA->BSRR = GPIO_BSRR_BR7;  // MOSI low
     
     USART2_SendString("Direct GPIO SPI pins initialized\r\n");
 }
 
 // Direct GPIO bit-banging SPI transfer (much slower but more reliable)
 uint8_t SPI1_DirectGPIO_Transfer(uint8_t data) {
     uint8_t received = 0;
     
     // Transfer 8 bits
     for (int i = 7; i >= 0; i--) {
         // Set MOSI (PA7)
         if (data & (1 << i)) {
             GPIOA->BSRR = GPIO_BSRR_BS7; // Set bit
         } else {
             GPIOA->BSRR = GPIO_BSRR_BR7; // Reset bit
         }
         
         // Delay
         for (int j = 0; j < 20; j++) __NOP();
         
         // SCK low (PA5)
         GPIOA->BSRR = GPIO_BSRR_BR5;
         
         // Delay
         for (int j = 0; j < 20; j++) __NOP();
         
         // SCK high (PA5)
         GPIOA->BSRR = GPIO_BSRR_BS5;
         
         // Read MISO (PA6)
         if (GPIOA->IDR & GPIO_IDR_ID6) {
             received |= (1 << i);
         }
         
         // Delay
         for (int j = 0; j < 20; j++) __NOP();
     }
     
     return received;
 }
 
 // Initialize SPI for SX1278 with very slow clock
 void SPI1_Init(void) {
     USART2_SendString("Initializing SPI1 interface...\r\n");
     
     // Initialize direct GPIO mode first (as backup)
     SPI1_DirectGPIO_Init();
     
     // Enable clock for GPIOA and SPI1
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
     
     // First ensure CS is high
     GPIOA->BSRR = GPIO_BSRR_BS9;
     Delay_ms(10);
     
     // Configure SPI pins
     // SCK (PA5), MOSI (PA7) as Alternate Function
     GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk);
     GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1); // AF mode (10)
     
     // MISO (PA6) as Alternate Function
     GPIOA->MODER &= ~GPIO_MODER_MODER6_Msk;
     GPIOA->MODER |= GPIO_MODER_MODER6_1; // AF mode (10)
     
     // Set Alternate Function 5 (SPI1) for PA5, PA6, PA7
     GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5_Msk | GPIO_AFRL_AFSEL6_Msk | GPIO_AFRL_AFSEL7_Msk);
     GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL5_Pos) | 
                      (5 << GPIO_AFRL_AFSEL6_Pos) | 
                      (5 << GPIO_AFRL_AFSEL7_Pos);
     
     // Configure SPI1
     // Reset SPI first
     SPI1->CR1 = 0;
     Delay_ms(10);
     
     // Use EXTREMELY slow SPI clock for reliability
     SPI1->CR1 = SPI_CR1_MSTR |     // Master mode
                 SPI_CR1_SSM |      // Software slave management 
                 SPI_CR1_SSI |      // Internal slave select
                 (7 << SPI_CR1_BR_Pos); // Prescaler = 256 (slowest possible)
     
     Delay_ms(10);
     
     // Enable SPI1
     SPI1->CR1 |= SPI_CR1_SPE;
     
     USART2_SendString("SPI1 interface initialized at very low speed\r\n");
 }
 
 // Set CS pin high (inactive) with increased delay
 void SPI1_CS_High(void) {
     GPIOA->BSRR = GPIO_BSRR_BS9;  // Set bit
     Delay_ms(5); // Longer delay for stability
 }
 
 // Set CS pin low (active) with increased delay
 void SPI1_CS_Low(void) {
     GPIOA->BSRR = GPIO_BSRR_BR9;  // Reset bit
     Delay_ms(5); // Longer delay for stability
 }
 
 // Transfer a single byte over SPI with verification and diagnostics
 uint8_t SPI1_Transfer(uint8_t data) {
     // Wait until TXE flag is set (Transmit buffer empty)
     uint32_t timeout = 100000;
     while(!(SPI1->SR & SPI_SR_TXE)) {
         if(--timeout == 0) {
             USART2_SendString("SPI TX timeout!\r\n");
             return 0xFF; // Error handling
         }
     }
     
     // Send data
     SPI1->DR = data;
     
     // Wait until RXNE flag is set (Receive buffer not empty)
     timeout = 100000;
     while(!(SPI1->SR & SPI_SR_RXNE)) {
         if(--timeout == 0) {
             USART2_SendString("SPI RX timeout!\r\n");
             return 0xFF; // Error handling
         }
     }
     
     // Return received data
     return SPI1->DR;
 }
 
 // Reset the SX1278 module with improved timing and multiple attempts
 void SX1278_Reset(void) {
     USART2_SendString("Performing multiple hard resets on SX1278 module...\r\n");
     
     for (int i = 0; i < 3; i++) {
         // Reset low
         GPIOA->BSRR = GPIO_BSRR_BR10;
         Delay_ms(100);  // Much longer delay
         
         // Reset high
         GPIOA->BSRR = GPIO_BSRR_BS10;
         Delay_ms(200);  // Much longer delay
         
         sprintf(debug_buffer, "Reset cycle %d complete\r\n", i+1);
         USART2_SendString(debug_buffer);
     }
 }
 
 // Force-write to an SX1278 register using direct GPIO bit-banging
 void SX1278_ForceWriteReg(uint8_t reg, uint8_t value) {
     // Try normal SPI first
     SPI1_CS_Low();
     SPI1_Transfer(reg | 0x80); // Set MSB for write operation
     SPI1_Transfer(value);
     SPI1_CS_High();
     
     // Verify write
     uint8_t readback = SX1278_ReadReg(reg);
     if (readback != value) {
         sprintf(debug_buffer, "Force write required for reg 0x%02X. Trying bit-bang...\r\n", reg);
         USART2_SendString(debug_buffer);
         
         // Try bit-bang SPI as fallback
         SPI1_CS_Low();
         SPI1_DirectGPIO_Transfer(reg | 0x80);
         SPI1_DirectGPIO_Transfer(value);
         SPI1_CS_High();
         
         // Verify again
         readback = SX1278_ReadReg(reg);
         sprintf(debug_buffer, "After bit-bang: Wrote 0x%02X, Read 0x%02X\r\n", value, readback);
         USART2_SendString(debug_buffer);
     }
 }
 
 // Write to an SX1278 register with verification
 void SX1278_WriteReg(uint8_t reg, uint8_t value) {
     uint8_t readback;
     uint8_t attempt = 0;
     const uint8_t max_attempts = 3;
     
     do {
         // Write register
         SPI1_CS_Low();
         SPI1_Transfer(reg | 0x80); // Set MSB for write operation
         SPI1_Transfer(value);
         SPI1_CS_High();
         
         // Add delay before verification
         Delay_ms(5);
         
         // Verify write
         readback = SX1278_ReadReg(reg);
         if (readback == value) {
             break; // Successfully written
         }
         
         sprintf(debug_buffer, "Register 0x%02X write failed. Wrote 0x%02X, read 0x%02X. Attempt %d\r\n", 
                 reg, value, readback, attempt+1);
         USART2_SendString(debug_buffer);
         
         attempt++;
         Delay_ms(10);
     } while (attempt < max_attempts);
 }
 
 // Read from an SX1278 register with retry
 uint8_t SX1278_ReadReg(uint8_t reg) {
     uint8_t value;
     
     SPI1_CS_Low();
     SPI1_Transfer(reg & 0x7F); // Clear MSB for read operation
     value = SPI1_Transfer(0x00);
     SPI1_CS_High();
     
     return value;
 }
 
 // Comprehensive hardware test
 void SX1278_HardwareTest(void) {
     USART2_SendString("\r\n=== SX1278 HARDWARE TEST ===\r\n");
     
     // 1. Check power connections by reading version register
     USART2_SendString("Test 1: Power and basic communication\r\n");
     SX1278_Reset();
     Delay_ms(100);
     
     for (int i = 0; i < 3; i++) {
         uint8_t version = SX1278_ReadReg(REG_VERSION);
         sprintf(debug_buffer, "  Version register read: 0x%02X (expect 0x12)\r\n", version);
         USART2_SendString(debug_buffer);
         Delay_ms(50);
         
         if (version != 0x12) {
             // Try with direct GPIO
             SPI1_CS_Low();
             SPI1_DirectGPIO_Transfer(REG_VERSION & 0x7F);
             version = SPI1_DirectGPIO_Transfer(0x00);
             SPI1_CS_High();
             
             sprintf(debug_buffer, "  Direct GPIO version read: 0x%02X\r\n", version);
             USART2_SendString(debug_buffer);
         }
     }
     
     // 2. Test SCK, MOSI, MISO continuity via register read/write
     USART2_SendString("Test 2: SPI signal integrity check\r\n");
     
     // Try to change a non-critical register (sync word)
     uint8_t origSync = SX1278_ReadReg(REG_SYNC_WORD);
     uint8_t testValue = (origSync == 0x12) ? 0x34 : 0x12;
     
     USART2_SendString("  Writing test value to SYNC register...\r\n");
     SX1278_WriteReg(REG_SYNC_WORD, testValue);
     uint8_t readback = SX1278_ReadReg(REG_SYNC_WORD);
     
     sprintf(debug_buffer, "  SYNC write test: Wrote 0x%02X, Read 0x%02X %s\r\n", 
             testValue, readback, (testValue == readback) ? "SUCCESS" : "FAILED");
     USART2_SendString(debug_buffer);
     
     // Also try with direct GPIO method
     USART2_SendString("  Trying direct GPIO write...\r\n");
     SPI1_CS_Low();
     SPI1_DirectGPIO_Transfer(REG_SYNC_WORD | 0x80);
     SPI1_DirectGPIO_Transfer(~testValue); // Invert the value
     SPI1_CS_High();
     Delay_ms(10);
     
     SPI1_CS_Low();
     SPI1_DirectGPIO_Transfer(REG_SYNC_WORD & 0x7F);
     readback = SPI1_DirectGPIO_Transfer(0x00);
     SPI1_CS_High();
     
     sprintf(debug_buffer, "  Direct GPIO SYNC write: Wrote 0x%02X, Read 0x%02X %s\r\n", 
             ~testValue, readback, ((~testValue) == readback) ? "SUCCESS" : "FAILED");
     USART2_SendString(debug_buffer);
     
     // 3. Test mode register - critical register
     USART2_SendString("Test 3: Operation mode register\r\n");
     
     // Try writing sleep mode
     SPI1_CS_Low();
     SPI1_Transfer(REG_OP_MODE | 0x80);
     SPI1_Transfer(0x00); // Sleep mode
     SPI1_CS_High();
     Delay_ms(20);
     
     uint8_t opMode = SX1278_ReadReg(REG_OP_MODE);
     sprintf(debug_buffer, "  Mode register write test: Wrote 0x00, Read 0x%02X %s\r\n", 
             opMode, (opMode == 0x00) ? "SUCCESS" : "FAILED");
     USART2_SendString(debug_buffer);
     
     // Try with direct GPIO
     USART2_SendString("  Trying direct GPIO mode write...\r\n");
     SPI1_CS_Low();
     SPI1_DirectGPIO_Transfer(REG_OP_MODE | 0x80);
     SPI1_DirectGPIO_Transfer(0x01); // Standby mode
     SPI1_CS_High();
     Delay_ms(20);
     
     SPI1_CS_Low();
     SPI1_DirectGPIO_Transfer(REG_OP_MODE & 0x7F);
     opMode = SPI1_DirectGPIO_Transfer(0x00);
     SPI1_CS_High();
     
     sprintf(debug_buffer, "  Direct GPIO mode write: Wrote 0x01, Read 0x%02X %s\r\n", 
             opMode, (opMode == 0x01) ? "SUCCESS" : "FAILED");
     USART2_SendString(debug_buffer);
     
     // 4. Test CS pin function
     USART2_SendString("Test 4: Chip Select (CS) pin test\r\n");
     
     // Write with CS properly managed
     SPI1_CS_Low();
     SPI1_Transfer(REG_SYNC_WORD | 0x80);
     SPI1_Transfer(0x55);
     SPI1_CS_High();
     Delay_ms(10);
     
     // Read back
     uint8_t value1 = SX1278_ReadReg(REG_SYNC_WORD);
     
     // Now send data without CS - should be ignored
     SPI1_Transfer(REG_SYNC_WORD | 0x80);
     SPI1_Transfer(0xAA);
     Delay_ms(10);
     
     // Read back - should still have previous value
     uint8_t value2 = SX1278_ReadReg(REG_SYNC_WORD);
     
     sprintf(debug_buffer, "  CS test: First read 0x%02X, Second read 0x%02X %s\r\n", 
             value1, value2, (value1 == value2 && value1 == 0x55) ? "CS WORKING" : "CS PROBLEM");
     USART2_SendString(debug_buffer);
     
     // 5. Reset test
     USART2_SendString("Test 5: Reset pin test\r\n");
     
     // Set a test value
     SX1278_WriteReg(REG_SYNC_WORD, 0x99);
     Delay_ms(10);
     
     // Read before reset
     uint8_t beforeReset = SX1278_ReadReg(REG_SYNC_WORD);
     
     // Reset the module
     GPIOA->BSRR = GPIO_BSRR_BR10; // Reset low
     Delay_ms(100);
     GPIOA->BSRR = GPIO_BSRR_BS10; // Reset high
     Delay_ms(200);
     
     // Read after reset - should be back to default
     uint8_t afterReset = SX1278_ReadReg(REG_SYNC_WORD);
     
     sprintf(debug_buffer, "  Reset test: Before 0x%02X, After 0x%02X %s\r\n", 
             beforeReset, afterReset, 
             (beforeReset == 0x99 && afterReset != 0x99) ? "RESET WORKING" : "RESET PROBLEM");
     USART2_SendString(debug_buffer);
     
     USART2_SendString("=== HARDWARE TEST COMPLETE ===\r\n\r\n");
     
     // Summary of test results
     USART2_SendString("HARDWARE TEST SUMMARY:\r\n");
     if (SX1278_ReadReg(REG_VERSION) != 0x12) {
         USART2_SendString("❌ VERSION REG TEST FAILED - Check power and wiring\r\n");
     } else {
         USART2_SendString("✓ Version register read correctly\r\n");
     }
     
     // Add more summary points here
     
     USART2_SendString("\r\nRECOMMENDATIONS:\r\n");
     USART2_SendString("1. Check all wiring connections\r\n");
     USART2_SendString("2. Verify 3.3V power supply is stable\r\n");
     USART2_SendString("3. Try shorter wires between STM32 and SX1278\r\n");
     USART2_SendString("4. Add pull-up resistors to SPI lines\r\n");
     USART2_SendString("5. If possible, try a different SX1278 module\r\n");
 }
 
 // Initialize the SX1278 LoRa module (simplified for diagnostics)
 void SX1278_Init(void) {
     USART2_SendString("\r\n=== INITIALIZING SX1278 LORA MODULE ===\r\n");
     
     // Initialize SPI
     SPI1_Init();
     Delay_ms(100);
     
     // Perform hardware reset
     SX1278_Reset();
     Delay_ms(100);
     
     // Check if module is responding
     uint8_t version = SX1278_ReadReg(REG_VERSION);
     sprintf(debug_buffer, "SX1278 Version: 0x%02X (should be 0x12)\r\n", version);
     USART2_SendString(debug_buffer);
     
     if (version != 0x12) {
         USART2_SendString("WARNING: SX1278 not detected correctly! Hardware problem likely.\r\n");
         USART2_SendString("Running hardware diagnostic tests...\r\n");
         SX1278_HardwareTest();
         return;
     }
     
     // Basic simplified configuration focused on reliability
     USART2_SendString("Device detected. Attempting basic configuration...\r\n");
     
     // 1. Sleep mode first
     SPI1_CS_Low();
     SPI1_Transfer(REG_OP_MODE | 0x80);
     SPI1_Transfer(MODE_SLEEP);
     SPI1_CS_High();
     Delay_ms(50);
     
     // 2. LoRa mode
     SPI1_CS_Low();
     SPI1_Transfer(REG_OP_MODE | 0x80);
     SPI1_Transfer(0x80); // Sleep + LoRa
     SPI1_CS_High();
     Delay_ms(100);
     
     // 3. Standby mode
     SPI1_CS_Low();
     SPI1_Transfer(REG_OP_MODE | 0x80);
     SPI1_Transfer(0x81); // Standby + LoRa
     SPI1_CS_High();
     Delay_ms(50);
     
     // 4. Set sync word
     SPI1_CS_Low();
     SPI1_Transfer(REG_SYNC_WORD | 0x80);
     SPI1_Transfer(0x12);
     SPI1_CS_High();
     Delay_ms(10);
     
     // Verify configuration
     uint8_t opMode = SX1278_ReadReg(REG_OP_MODE);
     uint8_t syncWord = SX1278_ReadReg(REG_SYNC_WORD);
     
     sprintf(debug_buffer, "Configuration check: MODE=0x%02X, SYNC=0x%02X\r\n", 
             opMode, syncWord);
     USART2_SendString(debug_buffer);
     
     // Report status
     if (opMode == 0x81 && syncWord == 0x12) {
         USART2_SendString("Basic configuration successful\r\n");
     } else {
         USART2_SendString("Configuration failed - hardware problem likely\r\n");
     }
     
     USART2_SendString("=== INITIALIZATION COMPLETE ===\r\n\r\n");
 }
 
 // Send data - simplified emergency version
 void SX1278_Send(uint8_t* data, uint8_t size) {
     USART2_SendString("Attempting LoRa transmission (emergency mode)...\r\n");
     
     // 1. Try to set standby mode
     SPI1_CS_Low();
     SPI1_Transfer(REG_OP_MODE | 0x80);
     SPI1_Transfer(0x81); // LoRa + Standby
     SPI1_CS_High();
     Delay_ms(20);
     
     // 2. Clear flags
     SPI1_CS_Low();
     SPI1_Transfer(REG_IRQ_FLAGS | 0x80);
     SPI1_Transfer(0xFF);
     SPI1_CS_High();
     
     // 3. Set FIFO pointer
     SPI1_CS_Low();
     SPI1_Transfer(REG_FIFO_ADDR_PTR | 0x80);
     SPI1_Transfer(0x00);
     SPI1_CS_High();
     
     // 4. Write data to FIFO
     for (uint8_t i = 0; i < size; i++) {
         SPI1_CS_Low();
         SPI1_Transfer(REG_FIFO | 0x80);
         SPI1_Transfer(data[i]);
         SPI1_CS_High();
     }
     
     // 5. Set payload length
     SPI1_CS_Low();
     SPI1_Transfer(REG_PAYLOAD_LENGTH | 0x80);
     SPI1_Transfer(size);
     SPI1_CS_High();
     
     // 6. Check state before TX
     uint8_t mode = SX1278_ReadReg(REG_OP_MODE);
     uint8_t flags = SX1278_ReadReg(REG_IRQ_FLAGS);
     sprintf(debug_buffer, "Before TX - Mode: 0x%02X, Flags: 0x%02X\r\n", mode, flags);
     USART2_SendString(debug_buffer);
     
     // 7. Enter TX mode - using both normal and direct methods
     // First normal SPI
     SPI1_CS_Low();
     SPI1_Transfer(REG_OP_MODE | 0x80);
     SPI1_Transfer(0x83); // LoRa + TX
     SPI1_CS_High();
     Delay_ms(10);
     
     // Then direct GPIO in case normal failed
     SPI1_CS_Low();
     SPI1_DirectGPIO_Transfer(REG_OP_MODE | 0x80);
     SPI1_DirectGPIO_Transfer(0x83); // LoRa + TX
     SPI1_CS_High();
     Delay_ms(20);
     
     // Check mode after setting
     mode = SX1278_ReadReg(REG_OP_MODE);
     sprintf(debug_buffer, "TX mode set: 0x%02X (should be 0x83)\r\n", mode);
     USART2_SendString(debug_buffer);
     
     // Wait for TX complete or timeout
     uint32_t startTime = HAL_GetTick();
     uint32_t lastCheck = startTime;
     while ((HAL_GetTick() - startTime) < 3000) {
         if ((HAL_GetTick() - lastCheck) >= 500) {
             lastCheck = HAL_GetTick();
             flags = SX1278_ReadReg(REG_IRQ_FLAGS);
             mode = SX1278_ReadReg(REG_OP_MODE);
             sprintf(debug_buffer, "TX wait - Mode: 0x%02X, Flags: 0x%02X, Time: %lu ms\r\n", 
                     mode, flags, HAL_GetTick() - startTime);
             USART2_SendString(debug_buffer);
             
             if (flags & IRQ_TX_DONE_MASK) {
                 USART2_SendString("TX complete!\r\n");
                 break;
             }
         }
         Delay_ms(10);
     }
     
     // Clear flags and return to standby
     SPI1_CS_Low();
     SPI1_Transfer(REG_IRQ_FLAGS | 0x80);
     SPI1_Transfer(0xFF);
     SPI1_CS_High();
     
     SPI1_CS_Low();
     SPI1_Transfer(REG_OP_MODE | 0x80);
     SPI1_Transfer(0x81); // LoRa + Standby
     SPI1_CS_High();
     
     USART2_SendString("Transmission attempt completed\r\n");
 }
 
 // Check if data is available (simplified)
 uint8_t SX1278_Available(void) {
     uint8_t flags = SX1278_ReadReg(REG_IRQ_FLAGS);
     return ((flags & IRQ_RX_DONE_MASK) && !(flags & IRQ_PAYLOAD_CRC_ERROR_MASK));
 }
 
 // Receive data (simplified)
 uint8_t SX1278_Receive(uint8_t* data, uint8_t size) {
     if (!SX1278_Available()) {
         return 0;
     }
     
     uint8_t rxSize = SX1278_ReadReg(REG_RX_NB_BYTES);
     uint8_t currentAddr = SX1278_ReadReg(REG_FIFO_RX_CURRENT_ADDR);
     
     SX1278_WriteReg(REG_FIFO_ADDR_PTR, currentAddr);
     
     uint8_t count = (rxSize < size) ? rxSize : size;
     for (uint8_t i = 0; i < count; i++) {
         data[i] = SX1278_ReadReg(REG_FIFO);
     }
     
     SX1278_WriteReg(REG_IRQ_FLAGS, 0xFF); // Clear flags
     
     return count;
 }