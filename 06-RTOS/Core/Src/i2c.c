#include "i2c.h"
 #include "usart.h"
 #include <stdio.h>
 
 // Initialize I2C1 peripheral
// Initialize I2C1 peripheral
void I2C1_Init(void) {
    // Enable GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    
    // Enable I2C1 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    
    // Reset I2C1 trước khi cấu hình
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    for(volatile int i = 0; i < 1000; i++); 
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    
    // Configure PB8 (SCL) and PB9 (SDA) as Alternate Function
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOB->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1); // AF mode
    
    // Set to AF4 for I2C1
    GPIOB->AFR[1] &= ~(0xF << ((8-8)*4) | 0xF << ((9-8)*4));
    GPIOB->AFR[1] |= (4 << ((8-8)*4) | 4 << ((9-8)*4));
    
    // Configure output type as open drain
    GPIOB->OTYPER |= (GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
    
    // Enable pull-up
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0);
    
    // Thêm độ trễ trước khi cấu hình I2C
    for(volatile int i = 0; i < 100000; i++);
    
    // Reset I2C
    I2C1->CR1 |= I2C_CR1_SWRST;
    for(volatile int i = 0; i < 1000; i++); 
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    
    // Program the peripheral input clock in I2C_CR2 Register
    // Giảm clock xuống cho ổn định
    I2C1->CR2 |= 42; // 42MHz
    
    // Set I2C speed to 100kHz - Tăng CCR cho tốc độ thấp hơn và ổn định hơn
    I2C1->CCR = 210;
    
    // Tăng thời gian rise time
    I2C1->TRISE = 43;
    
    // Thêm độ trễ trước khi kích hoạt
    for(volatile int i = 0; i < 10000; i++);
    
    // Enable I2C1
    I2C1->CR1 |= I2C_CR1_PE;
    
    // Thêm độ trễ sau khi kích hoạt
    for(volatile int i = 0; i < 100000; i++);
}
 void I2C_ScanBus(void) {
    char msg[50];
    uint8_t devices_found = 0;
    
    USART2_SendString("\r\nScanning I2C bus...\r\n");
    
    for(uint8_t addr = 1; addr < 128; addr++) {
        // Start condition
        if (I2C_Start()) {
            sprintf(msg, "Error creating start condition\r\n");
            USART2_SendString(msg);
            continue;
        }
        
        // Send address with write bit
        I2C1->DR = (addr << 1) | 0;
        
        // Wait a bit
        for(volatile int i = 0; i < 1000; i++);
        
        // Check if device responded
        if (I2C1->SR1 & I2C_SR1_ADDR) {
            // Clear ADDR flag
            __IO uint32_t tmp = I2C1->SR1;
            tmp = I2C1->SR2;
            (void)tmp;
            
            sprintf(msg, "Device found at address: 0x%02X\r\n", addr);
            USART2_SendString(msg);
            devices_found++;
        }
        
        // Stop condition
        I2C_Stop();
        
        // Small delay between addresses
        for(volatile int i = 0; i < 10000; i++);
    }
    
    sprintf(msg, "Scan complete. %u devices found.\r\n", devices_found);
    USART2_SendString(msg);
}
 // Generate I2C start condition
 uint8_t I2C_Start(void) {
     // Generate start condition
     I2C1->CR1 |= I2C_CR1_START;
     
     // Wait until SB flag is set
     uint32_t timeout = 10000;
     while (!(I2C1->SR1 & I2C_SR1_SB)) {
         if (--timeout == 0) return 1;
     }
     return 0;
 }
 
 // Generate I2C stop condition
 void I2C_Stop(void) {
     // Generate stop condition
     I2C1->CR1 |= I2C_CR1_STOP;
     
     // Wait until I2C is not busy
     while (I2C1->SR2 & I2C_SR2_BUSY);
 }
 
 // Write one byte to I2C
 uint8_t I2C_Write(uint8_t data) {
     // Wait until TXE is set
     uint32_t timeout = 10000;
     while (!(I2C1->SR1 & I2C_SR1_TXE)) {
         if (--timeout == 0) return 1;
     }
     
     // Send data
     I2C1->DR = data;
     
     // Wait until BTF is set
     timeout = 10000;
     while (!(I2C1->SR1 & I2C_SR1_BTF)) {
         if (--timeout == 0) return 1;
     }
     
     return 0;
 }
 
 // Read one byte with ACK
 uint8_t I2C_Read_ACK(void) {
     // Enable ACK
     I2C1->CR1 |= I2C_CR1_ACK;
     
     // Clear ADDR flag by reading SR1 and SR2
     __IO uint32_t tmp = I2C1->SR1;
     tmp = I2C1->SR2;
     (void)tmp; // Prevent compiler warning
     
     // Wait until RXNE is set
     uint32_t timeout = 10000;
     while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
         if (--timeout == 0) return 0;
     }
     
     // Return received data
     return (uint8_t)I2C1->DR;
 }
 
 // Read one byte with NACK
 uint8_t I2C_Read_NACK(void) {
     // Disable ACK
     I2C1->CR1 &= ~I2C_CR1_ACK;
     
     // Clear ADDR flag by reading SR1 and SR2
     __IO uint32_t tmp = I2C1->SR1;
     tmp = I2C1->SR2;
     (void)tmp; // Prevent compiler warning
     
     // Generate stop condition
     I2C1->CR1 |= I2C_CR1_STOP;
     
     // Wait until RXNE is set
     uint32_t timeout = 10000;
     while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
         if (--timeout == 0) return 0;
     }
     
     // Return received data
     return (uint8_t)I2C1->DR;
 }
 
 // CRC calculation for SHT30
 uint8_t SHT30_CRC_Check(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum) {
     uint8_t crc = 0xFF;
     uint8_t bit = 0;
     uint8_t byteCtr;
     
     // Calculate 8-bit checksum with polynomial 0x31 (x^8 + x^5 + x^4 + 1)
     for (byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
         crc ^= (data[byteCtr]);
         for (bit = 8; bit > 0; bit--) {
             if (crc & 0x80) {
                 crc = (crc << 1) ^ 0x31;
             } else {
                 crc = (crc << 1);
             }
         }
     }
     
     return (crc == checksum);
 }
 
 // Initialize SHT30 sensor
 uint8_t SHT30_Init(void) {
     // Initialize I2C1
     I2C1_Init();
     
     // Test communication with sensor
     if (I2C_Start()) return 1;
     
     // Send slave address with write bit
     I2C1->DR = (SHT30_ADDR << 1) | 0;
     
     // Wait until ADDR flag is set
     uint32_t timeout = 10000;
     while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
         if (--timeout == 0) {
             I2C_Stop();
             return 1;
         }
     }
     
     // Clear ADDR flag by reading SR1 and SR2
     __IO uint32_t tmp = I2C1->SR1;
     tmp = I2C1->SR2;
     (void)tmp; // Prevent compiler warning
     
     I2C_Stop();
     return 0;
 }
 
 // Read temperature and humidity from SHT30
 uint8_t SHT30_Read_Temperature_Humidity(float *temperature, float *humidity) {
     uint8_t data[6];
     uint16_t temp_raw, humid_raw;
     
     // Start I2C communication
     if (I2C_Start()) return 1;
     
     // Send slave address with write bit
     I2C1->DR = (SHT30_ADDR << 1) | 0;
     
     // Wait until ADDR flag is set
     uint32_t timeout = 10000;
     while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
         if (--timeout == 0) {
             I2C_Stop();
             return 1;
         }
     }
     
     // Clear ADDR flag
     __IO uint32_t tmp = I2C1->SR1;
     tmp = I2C1->SR2;
     (void)tmp; // Prevent compiler warning
     
     // Send measurement command high byte
     if (I2C_Write((SHT30_MEAS_HIGH_REP >> 8) & 0xFF)) {
         I2C_Stop();
         return 1;
     }
     
     // Send measurement command low byte
     if (I2C_Write(SHT30_MEAS_HIGH_REP & 0xFF)) {
         I2C_Stop();
         return 1;
     }
     
     // Stop condition
     I2C_Stop();
     
     // Delay at least 15ms for measurement
     for (uint32_t i = 0; i < 500000; i++);
     
     // Restart to read data
     if (I2C_Start()) return 1;
     
     // Send slave address with read bit
     I2C1->DR = (SHT30_ADDR << 1) | 1;
     
     // Wait until ADDR flag is set
     timeout = 10000;
     while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
         if (--timeout == 0) {
             I2C_Stop();
             return 1;
         }
     }
     
     // Enable ACK
     I2C1->CR1 |= I2C_CR1_ACK;
     
     // Clear ADDR flag
     tmp = I2C1->SR1;
     tmp = I2C1->SR2;
     
     // Read 6 bytes (temperature MSB, LSB, CRC, humidity MSB, LSB, CRC)
     for (int i = 0; i < 5; i++) {
         // Wait until RXNE is set
         timeout = 10000;
         while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
             if (--timeout == 0) {
                 I2C_Stop();
                 return 1;
             }
         }
         
         // Read data
         data[i] = I2C1->DR;
         
         if (i == 4) {
             // For the last byte, disable ACK and generate stop condition
             I2C1->CR1 &= ~I2C_CR1_ACK;
             I2C1->CR1 |= I2C_CR1_STOP;
         }
     }
     
     // Read last byte
     timeout = 10000;
     while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
         if (--timeout == 0) return 1;
     }
     data[5] = I2C1->DR;
     
     // Check CRC for temperature and humidity
     if (!SHT30_CRC_Check(&data[0], 2, data[2]) || !SHT30_CRC_Check(&data[3], 2, data[5])) {
         return 2;
     }
     
     // Calculate raw values
     temp_raw = (data[0] << 8) | data[1];
     humid_raw = (data[3] << 8) | data[4];
     
     // Convert to physical values
     // T[°C] = -45 + 175 * temp_raw / 65535
     // RH[%] = 100 * humid_raw / 65535
     *temperature = -45.0f + 175.0f * (float)temp_raw / 65535.0f;
     *humidity = 100.0f * (float)humid_raw / 65535.0f;
     
     return 0;
 }