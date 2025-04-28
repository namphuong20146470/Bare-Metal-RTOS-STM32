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
 
 // Rename buffer to avoid conflict with i2c.c
 static char spi_debug_buffer[100];
 
 // Use HAL_Delay instead of implementing our own
 // This function should be moved to a common utility file
 extern void HAL_Delay(uint32_t Delay);
 
 // Initialize SPI for SX1278 with much slower clock
 void SPI1_Init(void) {
     USART2_SendString("Initializing SPI1 interface...\r\n");
     
     // Enable clock for GPIOA and SPI1
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
     
     // Configure CS pin (PA9) as output FIRST
     GPIOA->MODER &= ~GPIO_MODER_MODER9_Msk;
     GPIOA->MODER |= GPIO_MODER_MODER9_0; // Output mode (01)
     GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_Msk; // High speed
     
     // Set CS high initially (inactive) BEFORE SPI configuration
     GPIOA->BSRR = GPIO_BSRR_BS9;
     HAL_Delay(10); // Delay after CS high for stability
     
     // RST pin (PA10) as output
     GPIOA->MODER &= ~GPIO_MODER_MODER10_Msk;
     GPIOA->MODER |= GPIO_MODER_MODER10_0; // Output mode (01)
     
     // DIO0 pin (PA8) as input with pull-down
     GPIOA->MODER &= ~GPIO_MODER_MODER8_Msk; // Input mode (00)
     GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD8_Msk;
     GPIOA->PUPDR |= GPIO_PUPDR_PUPD8_1; // Pull-down (10)
     
     // Now configure SPI pins
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
     
     // Set high speed for SPI pins
     GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED5_Msk | 
                       GPIO_OSPEEDR_OSPEED6_Msk | 
                       GPIO_OSPEEDR_OSPEED7_Msk;
     
     // Configure SPI1
     // Reset SPI first
     SPI1->CR1 = 0;
     HAL_Delay(10);
     
     // Configure SPI: MUCH SLOWER CLOCK for reliability
     // - Master mode
     // - CPOL=0, CPHA=0
     // - MSB first
     // - 8-bit data
     // - Prescaler = 256 (BR[2:0] = 111) for very slow clock
     SPI1->CR1 = SPI_CR1_MSTR |     // Master mode
                 SPI_CR1_SSM |      // Software slave management 
                 SPI_CR1_SSI |      // Internal slave select
                 (7 << SPI_CR1_BR_Pos); // Prescaler = 256 (much slower)
     
     HAL_Delay(10);
     
     // Enable SPI1
     SPI1->CR1 |= SPI_CR1_SPE;
     
     USART2_SendString("SPI1 interface initialized\r\n");
 }
 
 // Set CS pin high (inactive) with increased delay
 void SPI1_CS_High(void) {
     GPIOA->BSRR = GPIO_BSRR_BS9;  // Set bit
     HAL_Delay(5); // Longer delay for stability
 }
 
 // Set CS pin low (active) with increased delay
 void SPI1_CS_Low(void) {
     GPIOA->BSRR = GPIO_BSRR_BR9;  // Reset bit
     HAL_Delay(5); // Longer delay for stability
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
     USART2_SendString("Resetting SX1278 module...\r\n");
     
     // Multiple reset cycles for better reliability
     for (int i = 0; i < 3; i++) {
         // Reset low
         GPIOA->BSRR = GPIO_BSRR_BR10;
         HAL_Delay(50);  // Much longer delay
         
         // Reset high
         GPIOA->BSRR = GPIO_BSRR_BS10;
         HAL_Delay(100);  // Much longer delay
     }
     
     USART2_SendString("SX1278 reset sequence complete\r\n");
 }
 
 // Bit-bang SPI as a fallback for critical registers
 uint8_t SPI1_BitBang_Transfer(uint8_t data) {
     uint8_t received = 0;
     
     // Manual SCK (PA5), MOSI (PA7), MISO (PA6)
     // Set SCK and MOSI to output mode
     GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk);
     GPIOA->MODER |= GPIO_MODER_MODER5_0 | GPIO_MODER_MODER7_0;
     
     // Set MISO to input mode
     GPIOA->MODER &= ~GPIO_MODER_MODER6_Msk;
     
     // Transfer 8 bits
     for (int i = 7; i >= 0; i--) {
         // Set MOSI
         if (data & (1 << i)) {
             GPIOA->BSRR = GPIO_BSRR_BS7;
         } else {
             GPIOA->BSRR = GPIO_BSRR_BR7;
         }
         
         // Delay
         for (int j = 0; j < 100; j++) __NOP();
         
         // SCK high
         GPIOA->BSRR = GPIO_BSRR_BS5;
         
         // Delay
         for (int j = 0; j < 100; j++) __NOP();
         
         // Read MISO
         if (GPIOA->IDR & GPIO_IDR_ID6) {
             received |= (1 << i);
         }
         
         // SCK low
         GPIOA->BSRR = GPIO_BSRR_BR5;
         
         // Delay
         for (int j = 0; j < 100; j++) __NOP();
     }
     
     // Restore SPI pins to alternate function mode
     GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER6_Msk);
     GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER6_1);
     
     return received;
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
 
 // Force-write to a critical SX1278 register with bit-banging backup
 void SX1278_ForceWriteReg(uint8_t reg, uint8_t value) {
     // Try normal SPI first
     SPI1_CS_Low();
     SPI1_Transfer(reg | 0x80); // Set MSB for write operation
     SPI1_Transfer(value);
     SPI1_CS_High();
     
     // Verify write
     uint8_t readback = SX1278_ReadReg(reg);
     if (readback != value) {
         sprintf(spi_debug_buffer, "Force write required for reg 0x%02X. Trying bit-bang...\r\n", reg);
         USART2_SendString(spi_debug_buffer);
         
         // Try bit-bang SPI as fallback
         SPI1_CS_Low();
         SPI1_BitBang_Transfer(reg | 0x80);
         SPI1_BitBang_Transfer(value);
         SPI1_CS_High();
         
         // Verify again
         readback = SX1278_ReadReg(reg);
         sprintf(spi_debug_buffer, "After bit-bang: Wrote 0x%02X, Read 0x%02X\r\n", value, readback);
         USART2_SendString(spi_debug_buffer);
     }
 }
 
 // Write to an SX1278 register with verification and auto-retry
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
         HAL_Delay(5);
         
         // Verify all writes (increased verification)
         readback = SX1278_ReadReg(reg);
         if (readback == value) {
             break; // Successfully written
         }
         
         sprintf(spi_debug_buffer, "Register 0x%02X write failed. Wrote 0x%02X, read 0x%02X. Attempt %d\r\n", 
                 reg, value, readback, attempt+1);
         USART2_SendString(spi_debug_buffer);
         
         attempt++;
         HAL_Delay(10);
     } while (attempt < max_attempts);
     
     // Use force write for critical registers if still failing
     if (attempt >= max_attempts && (reg == REG_OP_MODE || reg == REG_PA_CONFIG)) {
         SX1278_ForceWriteReg(reg, value);
     }
 }
 
 // Read from an SX1278 register with retry
 uint8_t SX1278_ReadReg(uint8_t reg) {
     uint8_t value;
     uint8_t attempt = 0;
     const uint8_t max_attempts = 2;
     
     do {
         SPI1_CS_Low();
         SPI1_Transfer(reg & 0x7F); // Clear MSB for read operation
         value = SPI1_Transfer(0x00);
         SPI1_CS_High();
         
         // For version register, validate response
         if (reg == REG_VERSION && value != 0x12 && attempt < max_attempts-1) {
             USART2_SendString("Invalid version read, retrying...\r\n");
             attempt++;
             HAL_Delay(10);
             continue;
         }
         
         break;
     } while (attempt < max_attempts);
     
     return value;
 }
 
 // Set the operating mode with verification and force if needed
 void SX1278_SetMode(uint8_t mode) {
     uint8_t currentMode = SX1278_ReadReg(REG_OP_MODE);
     uint8_t newMode = (currentMode & 0x80) | mode;
     uint8_t readMode;
     uint8_t attempt = 0;
     const uint8_t max_attempts = 3;
     
     sprintf(spi_debug_buffer, "Mode change: 0x%02X -> 0x%02X\r\n", currentMode, newMode);
     USART2_SendString(spi_debug_buffer);
     
     // First try normal writes
     do {
         SX1278_WriteReg(REG_OP_MODE, newMode);
         
         // Longer delay for mode change
         HAL_Delay(50);
         
         // Verify mode was set
         readMode = SX1278_ReadReg(REG_OP_MODE);
         if (readMode == newMode) {
             sprintf(spi_debug_buffer, "Mode set success: 0x%02X\r\n", readMode);
             USART2_SendString(spi_debug_buffer);
             return;
         }
         
         sprintf(spi_debug_buffer, "Mode set failed. Wrote 0x%02X, read 0x%02X. Attempt %d\r\n", 
                 newMode, readMode, attempt+1);
         USART2_SendString(spi_debug_buffer);
         
         attempt++;
         HAL_Delay(20);
     } while (attempt < max_attempts);
     
     // If normal write fails, try force write and bit-banging
     USART2_SendString("Trying force mode change...\r\n");
     SX1278_ForceWriteReg(REG_OP_MODE, newMode);
 }
 
 // Set the frequency
 void SX1278_SetFrequency(float freq) {
     uint32_t frf = (uint32_t)((freq * 1000000.0) / 61.035);
     SX1278_WriteReg(REG_FRF_MSB, (frf >> 16) & 0xFF);
     SX1278_WriteReg(REG_FRF_MID, (frf >> 8) & 0xFF);
     SX1278_WriteReg(REG_FRF_LSB, frf & 0xFF);
     
     // Print current frequency setting
     sprintf(spi_debug_buffer, "Frequency set: %.2f MHz (FRF: 0x%06lX)\r\n", freq, frf);
     USART2_SendString(spi_debug_buffer);
 }
 
 // Hardware diagnostic test
 void SX1278_HardwareTest(void) {
     USART2_SendString("\r\n=== SX1278 DIAGNOSTIC TEST ===\r\n");
     
     // Test 1: SPI communication
     USART2_SendString("Test 1: SPI Read Version Register\r\n");
     for (int i = 0; i < 5; i++) {
         uint8_t version = SX1278_ReadReg(REG_VERSION);
         sprintf(spi_debug_buffer, "  Read %d: REG_VERSION=0x%02X (expected 0x12)\r\n", i+1, version);
         USART2_SendString(spi_debug_buffer);
         HAL_Delay(20);
     }
     
     // Rest of the diagnostic code...
     // Test 2: Register write/read consistency
     USART2_SendString("Test 2: Register Write/Read Test\r\n");
     
     // Save original sync word
     uint8_t origSync = SX1278_ReadReg(REG_SYNC_WORD);
     sprintf(spi_debug_buffer, "  Original SYNC=0x%02X\r\n", origSync);
     USART2_SendString(spi_debug_buffer);
     
     // Try writing different values
     uint8_t testValues[] = {0x12, 0x34, 0x56, 0x78};
     for (int i = 0; i < 4; i++) {
         SX1278_WriteReg(REG_SYNC_WORD, testValues[i]);
         uint8_t readVal = SX1278_ReadReg(REG_SYNC_WORD);
         sprintf(spi_debug_buffer, "  Write 0x%02X, Read 0x%02X %s\r\n", 
                 testValues[i], readVal, (testValues[i] == readVal) ? "OK" : "FAIL");
         USART2_SendString(spi_debug_buffer);
         HAL_Delay(20);
     }
     
     // Restore original sync word
     SX1278_WriteReg(REG_SYNC_WORD, origSync);
     
     // Test 3: Mode transitions
     USART2_SendString("Test 3: Mode Transition Test\r\n");
     
     // Test LoRa mode setting
     SX1278_WriteReg(REG_OP_MODE, 0x00); // FSK/OOK mode + sleep
     HAL_Delay(50);
     uint8_t readMode = SX1278_ReadReg(REG_OP_MODE);
     sprintf(spi_debug_buffer, "  FSK+Sleep: Wrote 0x00, Read 0x%02X\r\n", readMode);
     USART2_SendString(spi_debug_buffer);
     
     // Set LoRa mode
     SX1278_WriteReg(REG_OP_MODE, 0x80); // LoRa mode + sleep
     HAL_Delay(100);
     readMode = SX1278_ReadReg(REG_OP_MODE);
     sprintf(spi_debug_buffer, "  LoRa+Sleep: Wrote 0x80, Read 0x%02X\r\n", readMode);
     USART2_SendString(spi_debug_buffer);
     
     // Try standby
     SX1278_WriteReg(REG_OP_MODE, 0x81); // LoRa + standby
     HAL_Delay(50);
     readMode = SX1278_ReadReg(REG_OP_MODE);
     sprintf(spi_debug_buffer, "  LoRa+Standby: Wrote 0x81, Read 0x%02X\r\n", readMode);
     USART2_SendString(spi_debug_buffer);
     
     USART2_SendString("=== DIAGNOSTIC TEST COMPLETE ===\r\n\r\n");
 }
 
 // Initialize the SX1278 LoRa module with proper power-up sequence and diagnostics
 void SX1278_Init(void) {
     USART2_SendString("\r\n=== INITIALIZING SX1278 LORA MODULE ===\r\n");
     
     // Start with SPI initialization
     SPI1_Init();
     HAL_Delay(100);
     
     // Hard reset the module
     SX1278_Reset();
     HAL_Delay(100);
     
     // Run diagnostics to check hardware connection
     SX1278_DiagnosticTest();
     
     // Check if module is responding
     uint8_t version = SX1278_ReadReg(REG_VERSION);
     sprintf(spi_debug_buffer, "SX1278 Version: 0x%02X (should be 0x12)\r\n", version);
     USART2_SendString(spi_debug_buffer);
     
     if (version != 0x12) {
         USART2_SendString("WARNING: SX1278 not properly detected! Check wiring.\r\n");
         USART2_SendString("Will try to continue but may not work correctly.\r\n");
     }
     
     // ===== Begin initialization sequence =====
     USART2_SendString("Starting SX1278 initialization sequence...\r\n");
     
     // 1. Set sleep mode first (completely reset the LoRa state machine)
     // Try with force-write to ensure it works
     SX1278_ForceWriteReg(REG_OP_MODE, MODE_SLEEP);
     HAL_Delay(50);
     
     // 2. Verify sleep mode was entered
     uint8_t currentMode = SX1278_ReadReg(REG_OP_MODE);
     if ((currentMode & 0x07) != MODE_SLEEP) {
         sprintf(spi_debug_buffer, "Failed to enter sleep mode: 0x%02X\r\n", currentMode);
         USART2_SendString(spi_debug_buffer);
     }
     
     // Continued initialization
     // 3. Set LoRa mode (bit 7) while in sleep mode - critical step!
     SX1278_ForceWriteReg(REG_OP_MODE, 0x80);
     HAL_Delay(100);
     
     // 4. Verify LoRa mode was set
     currentMode = SX1278_ReadReg(REG_OP_MODE);
     if (!(currentMode & 0x80)) {
         sprintf(spi_debug_buffer, "Failed to enter LoRa mode: 0x%02X\r\n", currentMode);
         USART2_SendString(spi_debug_buffer);
     } else {
         USART2_SendString("Successfully entered LoRa mode\r\n");
     }
     
     // Rest of the initialization code...
     // 5. Move to standby for further configuration
     SX1278_ForceWriteReg(REG_OP_MODE, 0x81); // LoRa + Standby
     HAL_Delay(50);
     
     // 6. Configure RF carrier frequency - 433 MHz
     SX1278_SetFrequency(433.0);
     
     // 7. Configure TX power - PA_BOOST pin with max power
     // PA_BOOST enabled (bit 7), max power 17dBm
     SX1278_WriteReg(REG_PA_CONFIG, 0x8F);
     
     // 8. Configure PA ramp time (50us)
     SX1278_WriteReg(REG_PA_RAMP, 0x09);
     
     // 9. Over-current protection - limit to 100mA
     SX1278_WriteReg(REG_OCP, 0x1B);
     
     // 10. Improve receiver sensitivity
     SX1278_WriteReg(REG_LNA, 0x23); // Max gain, LNA enabled
     
     // 11. Configure modulation parameters
     // BW=125kHz, CR=4/5, Explicit header mode
     SX1278_WriteReg(REG_MODEM_CONFIG_1, 0x72);
     
     // SF7, CRC enabled
     SX1278_WriteReg(REG_MODEM_CONFIG_2, 0x74);
     
     // 12. Enable automatic AGC, Low Data Rate Optimization
     SX1278_WriteReg(REG_MODEM_CONFIG_3, 0x04);
     
     // 13. Set packet parameters
     SX1278_WriteReg(REG_PREAMBLE_MSB, 0x00);
     SX1278_WriteReg(REG_PREAMBLE_LSB, 0x08); // 8-symbol preamble
     
     // 14. CRITICAL: Set sync word to match ESP32 Arduino's default (0x12)
     SX1278_WriteReg(REG_SYNC_WORD, 0x12);
     
     // 15. Configure DIO pins
     SX1278_WriteReg(REG_DIO_MAPPING_1, 0x00);
     
     // 16. Reset FIFO pointers
     SX1278_WriteReg(REG_FIFO_TX_BASE_ADDR, 0x00);
     SX1278_WriteReg(REG_FIFO_RX_BASE_ADDR, 0x00);
     
     // 17. Clear all IRQ flags
     SX1278_WriteReg(REG_IRQ_FLAGS, 0xFF);
     
     // 18. Disable frequency hopping
     SX1278_WriteReg(REG_HOP_PERIOD, 0x00);
     
     // 19. Max payload length
     SX1278_WriteReg(REG_MAX_PAYLOAD_LENGTH, 0x80); // 128 bytes
     
     // Verify configuration
     uint8_t opMode = SX1278_ReadReg(REG_OP_MODE);
     uint8_t paConfig = SX1278_ReadReg(REG_PA_CONFIG);
     uint8_t modem1 = SX1278_ReadReg(REG_MODEM_CONFIG_1);
     uint8_t modem2 = SX1278_ReadReg(REG_MODEM_CONFIG_2);
     uint8_t syncWord = SX1278_ReadReg(REG_SYNC_WORD);
     
     sprintf(spi_debug_buffer, "LoRa Config: MODE=0x%02X, PA=0x%02X, M1=0x%02X, M2=0x%02X, SYNC=0x%02X\r\n", 
             opMode, paConfig, modem1, modem2, syncWord);
     USART2_SendString(spi_debug_buffer);
     
     // Final step - set to standby
     SX1278_ForceWriteReg(REG_OP_MODE, 0x81); // LoRa + Standby
     HAL_Delay(20);
     
     USART2_SendString("SX1278 initialization complete\r\n");
     USART2_SendString("=== INITIALIZATION COMPLETE ===\r\n\r\n");
 }
 
 // Send data with improved reliability and debugging
 void SX1278_Send(uint8_t* data, uint8_t size) {
     USART2_SendString("Starting LoRa transmission...\r\n");
     
     // Step 1: Enter standby mode first
     SX1278_ForceWriteReg(REG_OP_MODE, 0x81);  // LoRa + Standby
     HAL_Delay(20);
     
     // Rest of the send function
     // Step 2: Clear all IRQ flags
     SX1278_WriteReg(REG_IRQ_FLAGS, 0xFF);
     
     // Step 3: Reset and check FIFO ptr
     SX1278_WriteReg(REG_FIFO_ADDR_PTR, 0);
     uint8_t fifoPtr = SX1278_ReadReg(REG_FIFO_ADDR_PTR);
     if (fifoPtr != 0) {
         sprintf(spi_debug_buffer, "FIFO PTR not reset (0x%02X)\r\n", fifoPtr);
         USART2_SendString(spi_debug_buffer);
     }
     
     // Step 4: Write data to FIFO
     for (uint8_t i = 0; i < size; i++) {
         SX1278_WriteReg(REG_FIFO, data[i]);
     }
     
     // Step 5: Set payload length and verify
     SX1278_WriteReg(REG_PAYLOAD_LENGTH, size);
     uint8_t payloadLen = SX1278_ReadReg(REG_PAYLOAD_LENGTH);
     if (payloadLen != size) {
         sprintf(spi_debug_buffer, "Payload length mismatch: set %u, read %u\r\n", size, payloadLen);
         USART2_SendString(spi_debug_buffer);
     }
     
     // Step 6: Check state before TX
     uint8_t mode = SX1278_ReadReg(REG_OP_MODE);
     uint8_t flags = SX1278_ReadReg(REG_IRQ_FLAGS);
     uint8_t irq_mask = SX1278_ReadReg(REG_IRQ_FLAGS_MASK);
     sprintf(spi_debug_buffer, "Before TX - Mode: 0x%02X, Flags: 0x%02X, Mask: 0x%02X\r\n", 
             mode, flags, irq_mask);
     USART2_SendString(spi_debug_buffer);
     
     // Step 7: First clear masking of TX done IRQ
     SX1278_WriteReg(REG_IRQ_FLAGS_MASK, irq_mask & ~IRQ_TX_DONE_MASK);
     
     // Step 8: Enter TX mode - using FORCE WRITE for reliability
     SX1278_ForceWriteReg(REG_OP_MODE, 0x83);  // 0x80 (LoRa) + 0x03 (TX)
     HAL_Delay(20);
     
     // Step 9: Double-check mode was properly set
     mode = SX1278_ReadReg(REG_OP_MODE);
     if (mode != 0x83) {
         sprintf(spi_debug_buffer, "WARNING: TX mode not entered! Read: 0x%02X\r\n", mode);
         USART2_SendString(spi_debug_buffer);
         
         // Try a 2nd time with force-write
         USART2_SendString("Retrying TX mode entry with force write...\r\n");
         SX1278_ForceWriteReg(REG_OP_MODE, 0x83);
         HAL_Delay(50);
         
         mode = SX1278_ReadReg(REG_OP_MODE);
         sprintf(spi_debug_buffer, "TX mode verification: 0x%02X\r\n", mode);
         USART2_SendString(spi_debug_buffer);
     } else {
         USART2_SendString("TX mode entered successfully\r\n");
     }
     
     // Step 10: Wait for TX done with improved timeout handling
     uint32_t startTime = HAL_GetTick();
     uint32_t lastPrint = 0;
     uint8_t txDone = 0;
     
     while (!txDone && (HAL_GetTick() - startTime < 5000)) { // Extended timeout to 5 seconds
         flags = SX1278_ReadReg(REG_IRQ_FLAGS);
         
         // Check TX done bit
         if (flags & IRQ_TX_DONE_MASK) {
             txDone = 1;
             sprintf(spi_debug_buffer, "TX complete! Time: %lu ms\r\n", HAL_GetTick() - startTime);
             USART2_SendString(spi_debug_buffer);
             break;
         }
         
         // Print status every 500ms
         uint32_t now = HAL_GetTick();
         if (now - lastPrint >= 500) {
             lastPrint = now;
             mode = SX1278_ReadReg(REG_OP_MODE);
             flags = SX1278_ReadReg(REG_IRQ_FLAGS);
             sprintf(spi_debug_buffer, "TX wait - Mode: 0x%02X, Flags: 0x%02X, Time: %lu ms\r\n", 
                     mode, flags, now - startTime);
             USART2_SendString(spi_debug_buffer);
             
             // If mode changed unexpectedly, try to restore it
             if (mode != 0x83) {
                 USART2_SendString("TX mode lost! Restoring...\r\n");
                 SX1278_ForceWriteReg(REG_OP_MODE, 0x83);
                 HAL_Delay(20);
             }
         }
         
         HAL_Delay(10);
     }
     
     // Complete the send function
     // Step 11: Handle timeout
     if (!txDone) {
         flags = SX1278_ReadReg(REG_IRQ_FLAGS);
         mode = SX1278_ReadReg(REG_OP_MODE);
         sprintf(spi_debug_buffer, "TX timeout after %lu ms! Mode: 0x%02X, IRQ flags: 0x%02X\r\n", 
                 HAL_GetTick() - startTime, mode, flags);
         USART2_SendString(spi_debug_buffer);
         
         // Check if module is still responsive
         uint8_t version = SX1278_ReadReg(REG_VERSION);
         sprintf(spi_debug_buffer, "SX1278 version reg: 0x%02X\r\n", version);
         USART2_SendString(spi_debug_buffer);
         
         // Try to reset the module
         USART2_SendString("Attempting recovery reset...\r\n");
         SX1278_Reset();
         SX1278_ForceWriteReg(REG_OP_MODE, 0x81); // LoRa + Standby
         HAL_Delay(50);
     }
     
     // Step 12: Clear all IRQ flags
     SX1278_WriteReg(REG_IRQ_FLAGS, 0xFF);
     
     // Step 13: Go back to standby mode
     SX1278_ForceWriteReg(REG_OP_MODE, 0x81);  // LoRa + Standby
     HAL_Delay(20);
     
     // Final status update
     flags = SX1278_ReadReg(REG_IRQ_FLAGS);
     mode = SX1278_ReadReg(REG_OP_MODE);
     sprintf(spi_debug_buffer, "After TX - Mode: 0x%02X, Flags: 0x%02X\r\n", mode, flags);
     USART2_SendString(spi_debug_buffer);
 }
 
 // Check if data is available
 uint8_t SX1278_Available(void) {
     uint8_t flags = SX1278_ReadReg(REG_IRQ_FLAGS);
     
     // Debug IRQ flags if non-zero
     if (flags != 0) {
         sprintf(spi_debug_buffer, "IRQ Flags: 0x%02X\r\n", flags);
         USART2_SendString(spi_debug_buffer);
     }
     
     // Check RX_DONE and validate CRC
     return ((flags & IRQ_RX_DONE_MASK) && !(flags & IRQ_PAYLOAD_CRC_ERROR_MASK));
 }
 
 // Receive data with improved error handling
 uint8_t SX1278_Receive(uint8_t* data, uint8_t size) {
     if (!SX1278_Available()) {
         return 0;
     }
     
     // Get received bytes count
     uint8_t len = SX1278_ReadReg(REG_RX_NB_BYTES);
     
     // Debug receive length
     sprintf(spi_debug_buffer, "LORA RX: %u bytes received\r\n", len);
     USART2_SendString(spi_debug_buffer);
     
     // Get current FIFO address
     uint8_t currentAddr = SX1278_ReadReg(REG_FIFO_RX_CURRENT_ADDR);
     
     // Set FIFO pointer
     SX1278_WriteReg(REG_FIFO_ADDR_PTR, currentAddr);
     
     // Read data (with size limit)
     uint8_t count = (len < size) ? len : size;
     for (uint8_t i = 0; i < count; i++) {
         data[i] = SX1278_ReadReg(REG_FIFO);
     }
     
     // Clear IRQ flags
     SX1278_WriteReg(REG_IRQ_FLAGS, 0xFF);
     
     return count;
 }