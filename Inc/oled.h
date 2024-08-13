/*
 * oled.h
 *
 *  Created on: May 19, 2024
 *      Author: phine
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

#define SeeedOLED_Address               (0x3c << 1)
#define SeeedOLED_Command_Mode          0x80
#define SeeedOLED_Data_Mode             0x40
#define SeeedOLED_Display_Off_Cmd       0xAE
#define SeeedOLED_Display_On_Cmd        0xAF
#define SeeedOLED_Normal_Display_Cmd    0xA6
#define SeeedOLED_Inverse_Display_Cmd   0xA7
#define SeeedOLED_Activate_Scroll_Cmd   0x2F
#define SeeedOLED_Dectivate_Scroll_Cmd  0x2E
#define SeeedOLED_Set_Brightness_Cmd    0x81

#define PAGE_MODE         01
#define HORIZONTAL_MODE   02


void init();
void sendCommand(unsigned char command);
void sendData(unsigned char data);
void setNormalDisplay();
void setInverseDisplay();
void setPageMode();
void setHorizontalMode();
void setTextXY(unsigned char Row, unsigned char Column);
void clearDisplay();
void putChar(unsigned char C);
void putString(const char* String);
void putInt(int variable);
unsigned char floatToString(float floatNumber, char* buffer, unsigned char decimal);
unsigned char putFloat(float floatNumber, unsigned char decimal);

#endif /* INC_OLED_H_ */
