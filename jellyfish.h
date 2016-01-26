/**********************************************************
*   Jellyfish FSW - Main
*
*   25 Jan 2016
*
*   Joe Gibson 						(joseph.gibson@nasa.gov)
*	Jordi Vila Hernandez de Lorenzo	(jordi.vila.hdl@gmail.com)
*
*   License: MIT (http://user.mit-license.org)
*
*   https://github.com/gibsjose/jellyfish
**********************************************************/

#ifndef JELLYFISH_H
#define JELLYFISH_H

#include "jellyfish-globals.h"   //Global definitions

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "jellyfish-i2c.h"
#include "utilities/delay.h"
#include "utilities/usart.h"

//USART Settings
#define FOSC F_CPU          //Clock Speed (Hz)
#define BAUD 9600           //Baud Rate
#define UBRR (((((FOSC * 10) / (16L * BAUD)) + 5) / 10) - 1)

//Timer Definitions
#define TIMER1_COUNT    ((FOSC / 1024) - 1)     //Timer 1 count value for CTC mode: 1 second, at 1024 prescaler

//Pin Definitions
#define LED_PIN     PB2     //PB2 is the board LED, PB5 is the Arduino LED
#define LED_DD      DDB2

int init(void);
int port_init(void);
int timer1_init(void);
int peripheral_init(void);
int device_init(void);

#endif//JELLYFISH_H
