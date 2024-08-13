/*
 * gps.h
 *
 *  Created on: May 19, 2024
 *      Author: phine
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

typedef struct{

    float longitude;
    float latitude;
    float altitude_ft;

    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;

} GPS_t;


void GPS_Init();
void GPS_UART_Data();
int GPS_validate_value(char *nmeastr);
void GPS_extraction(char *GPSsrt);
float GPS_nmea_conversion(float deg_coord, char nsew);

#endif /* INC_GPS_H_ */
