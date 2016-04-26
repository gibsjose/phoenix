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

/*#define PIN_13      (1 << 5)    //Bit to configure Pin 13 as output on PORTB
#define PIN_12      (1 << 4)    //Bit to configure Pin 13 as output on PORTB
*/
void init_analog_input_pins(void);
double readBatteryVoltage(void);
void LED_ON(void);
void LED_OFF(void);
#endif//PHOENIX_UTIL_H
