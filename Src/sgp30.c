/*
 * sgp30.c
 *
 *  Created on: 25 mai 2024
 *      Author: phine
 */


#include "sgp30.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"


extern I2C_HandleTypeDef hi2c2;


void SGP30_Init() {
    uint16_t command = SGP30_INIT_AIR_QUALITY;
    HAL_I2C_Mem_Write(&hi2c2, SGP30_I2C_ADDRESS << 1, command, 2, NULL, 0, 10000);
    HAL_Delay(10);
}

void SGP30_Read_Air_Quality(uint16_t *eco2, uint16_t *tvoc){

	uint8_t buffer[6];
	uint16_t command = SGP30_MEASURE_AIR_QUALITY;

	 HAL_I2C_Mem_Write(&hi2c2, SGP30_I2C_ADDRESS << 1, command, 2, NULL, 0, 12000);
	 HAL_Delay(12);

	 HAL_I2C_Master_Receive(&hi2c2, SGP30_I2C_ADDRESS << 1, buffer, 6, 10000);

	    if (CalcCRC(buffer, 2) == buffer[2] && CalcCRC(buffer + 3, 2) == buffer[5]) {
	        *eco2 = (buffer[0] << 8) | buffer[1];
	        *tvoc = (buffer[3] << 8) | buffer[4];
	    } else {
	        *eco2 = 0;
	        *tvoc = 0;
	    }
}

uint8_t CalcCRC(uint8_t data[], uint8_t length) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

