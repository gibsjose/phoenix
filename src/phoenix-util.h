/**********************************************************
*   Phoenix FSW - Utilities
*
*   25 Jan 2016
*
*   Joe Gibson 						(joseph.gibson@nasa.gov)
*	Jordi Vila Hernandez de Lorenzo	(jordi.vila.hdl@gmail.com)
*
*   License: MIT (http://gibsjose.mit-license.org)
*
*   https://github.com/gibsjose/phoenix
**********************************************************/

#ifndef PHOENIX_UTIL_H
#define PHOENIX_UTIL_H

#include <avr/io.h>
#include <string.h>
#include <stdlib.h>

#define PIN_51      (1 << 2)    //Bit to configure Pin 13 as output on PORTB
#define PIN_53      (1 << 0)    //Bit to configure Pin 53 as output on PORTB

void init_analog_input_pins(void);
double readBatteryVoltage(void);

void init_LEDs_as_outputs(void);

void LED_RED_ON(void);
void LED_RED_OFF(void);
void LED_RED_CHANGE_STATUS(void);

void LED_GREEN_ON(void);
void LED_GREEN_OFF(void);
void LED_GREEN_CHANGE_STATUS(void);
#endif//PHOENIX_UTIL_H
