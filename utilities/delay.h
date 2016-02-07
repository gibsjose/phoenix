#ifndef DELAY_H
#define DELAY_H

#include "../phoenix-globals.h"    //F_CPU definition

#include <util/delay.h>
#include <avr/interrupt.h>

int delay_us(unsigned long int);

#endif//DELAY_H
