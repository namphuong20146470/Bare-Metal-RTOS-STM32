/*
 * i2c.h
 *
 *  Created on: Apr 25, 2025
 *      Author: phuong
 */

 #ifndef INC_I2C_H_
 #define INC_I2C_H_
 
 #include "stm32f4xx.h"
 #include <stdint.h>
 
 // SHT30 Definitions
 #define SHT30_ADDR                 0x44    // SHT30 I2C address (0x44 or 0x45)
 #define SHT30_MEAS_HIGH_REP_STRETCH 0x2C06 // Measurement command: high repeatability with clock stretching
 #define SHT30_MEAS_HIGH_REP         0x2400 // Measurement command: high repeatability without clock stretching
 
 // I2C Functions
 void I2C1_Init(void);
 uint8_t I2C_Start(void);
 void I2C_Stop(void);
 uint8_t I2C_Write(uint8_t data);
 uint8_t I2C_Read_ACK(void);
 uint8_t I2C_Read_NACK(void);
 void I2C_ScanBus(void) ;
 
 // SHT30 Functions
 uint8_t SHT30_Init(void);
 uint8_t SHT30_Read_Temperature_Humidity(float *temperature, float *humidity);
 uint8_t SHT30_CRC_Check(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);
 
 #endif /* INC_I2C_H_ */