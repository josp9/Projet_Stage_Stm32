/*
 * gps.c
 *
 *  Created on: May 19, 2024
 *      Author: phine
 */
#include "gps.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

uint8_t rx_data = 0;
uint8_t rx_buffer[128];
uint8_t rx_index = 0;

GPS_t GPS;

void GPS_Init(){

	HAL_UART_Receive_IT(&huart1,(uint8_t *) &rx_data, 1);
}


void GPS_UART_Data(){

	if(rx_data != '\n' && rx_index < sizeof(rx_buffer)){
		rx_buffer[rx_index++] = rx_data;
	}else{

		if(GPS_validate_value((char*)rx_buffer)){

			GPS_extraction((char*)rx_buffer);
		}
		rx_index = 0;
		memset(rx_buffer, 0, sizeof(rx_buffer));
	}
	HAL_UART_Receive_IT(&huart1,&rx_data, 1);
}

int GPS_validate_value(char *nmeastr){

	char confirm[3];
	char confirmcalcul[3];
	int i = 0;
	int calcul_confirm = 0;

	i = 0;
	calcul_confirm = 0;

	if(nmeastr[i] == '$'){
		i++;
	}else{
		return 0;
	}

    while((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75)){
        calcul_confirm ^= nmeastr[i];
        i++;
    }

    if(i >= 75){
        return 0;
    }

    if (nmeastr[i] == '*'){
        confirm[0] = nmeastr[i+1];
        confirm[1] = nmeastr[i+2];
        confirm[2] = 0;
    }
    else{
    	return 0;
    }

    sprintf(confirmcalcul,"%02X",calcul_confirm);

    return((confirmcalcul[0] == confirm[0])&& (confirmcalcul[1] == confirm[1]));
}

void GPS_extraction(char *GPSsrt){

	if(!strncmp(GPSsrt, "$GPGGA", 6)){
	    	//$GPGGA,064036.289,4836.5375,N,00740.9373,E,1,04,3.2,200.2,M,,,,0000*0E
	    	if(sscanf(GPSsrt,"$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &GPS.utc_time, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.lock, &GPS.satelites, &GPS.hdop, &GPS.msl_altitude, &GPS.msl_units) >= 1){
					GPS.latitude = GPS_nmea_conversion(GPS.nmea_latitude, GPS.ns);
					GPS.longitude = GPS_nmea_conversion(GPS.nmea_longitude, GPS.ew);
	    	}
	    }
}

float GPS_nmea_conversion(float deg_coord, char nsew){
	int degree = (int)(deg_coord/100);
	    float minutes = deg_coord - degree*100;
	    float dec_deg = minutes / 60;
	    float decimal = degree + dec_deg;
	    if (nsew == 'S' || nsew == 'W')
	    {
	        decimal *= -1;
	    }
	    return decimal;
}


