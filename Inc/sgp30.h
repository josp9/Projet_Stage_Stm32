/*
 * sgp30.h
 *
 *  Created on: 25 mai 2024
 *      Author: phine
 */

#ifndef INC_SGP30_H_
#define INC_SGP30_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"


#define SGP30_I2C_ADDRESS 0x58
#define SGP30_INIT_AIR_QUALITY 0x2003
#define SGP30_MEASURE_AIR_QUALITY 0x2008

typedef struct{

uint16_t tvoc;

uint16_t eco2;

}sgp;

void SGP30_Init();
void SGP30_Read_Air_Quality(uint16_t *eco2, uint16_t *tvoc);
uint8_t CalcCRC(uint8_t data[], uint8_t length);



#endif /* INC_SGP30_H_ */
