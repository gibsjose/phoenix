/**********************************************************
*   Phoenix FSW - Main
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

#ifndef PHOENIX_H
#define PHOENIX_H

#include "phoenix-globals.h"   //Global definitions

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "phoenix-gyro.h"
#include "phoenix-receiver.h"
#include "phoenix-controls.h"
#include "phoenix-util.h"


#include "delay/delay.h"
#include "uart/uart.h"

//USART Settings
#define FOSC F_CPU          //Clock Speed (Hz)
#define BAUD 9600           //Baud Rate
#define UBRR (((((FOSC * 10) / (16L * BAUD)) + 5) / 10) - 1)

//Timer Definitions
#define TIMER1_COUNT    ((FOSC / 1024) - 1)     //Timer 1 count value for CTC mode: 1 second, at 1024 prescaler

//Pin Definitions
#define LED_PIN     PB5     //PB2 is the board LED, PB5 is the Arduino LED
#define LED_RED      DDB5

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Phoenix MODE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define FLY  1           //Normal Opeartion of Phoenix
#define CALIBRATE  2   //Just used on startup
#define IDLE 3           //Waiting for the appropiate receiver input signals to start the motors
#define STOP_MOTORS 4           //Waiting for the appropiate receiver input signals to start the motors

/*#define PIN_13      (1 << 5)    //Bit to configure Pin 13 as output on PORTB
#define PIN_12      (1 << 4)    //Bit to configure Pin 13 as output on PORTB*/

int init(void);
int port_init(void);
int timer1_init(void);
int peripheral_init(void);
int device_init(void);
void check_value(int);


#endif//PHOENIX_H
