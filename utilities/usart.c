/*
 * usart.cpp
 *
 * Created: 07/12/2011 15:17:35
 *  Author: Boomber
 * Modified: 09/15/2013 by R. Bossemeyer to usart.c
 * changed one line to placate GCC
 *
 */
#include "usart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>			// Conversions

void usart_init( unsigned int ubrr)
{
/*Set baud rate */
UBRR0H = (unsigned char)(ubrr>>8);
UBRR0L = (unsigned char)ubrr;
//Enable receiver and transmitter */
UCSR0B = (1<<RXEN0)|(1<<TXEN0);
/* Set frame format: 8data, 2stop bit */
UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}




void usart_send_byte( unsigned char data )
{
/* Wait for empty transmit buffer */
while ( !( UCSR0A & (1<<UDRE0)) )
;
/* Put data into buffer, sends the data */
//UDR0 = char(data);
UDR0 = data;
}

void usart_send_string(const char *str)
{

	  while (*str)
      usart_send_byte(*str++);

}

void usart_send_int(unsigned int d )
{
	char str[10];
	sprintf(str,"%u",d);
	usart_send_string(str);

}



unsigned char usart_receive( void )
{
/* Wait for data to be received */
while ( !(UCSR0A & (1<<RXC0)) )
;
/* Get and return received data from buffer */
return UDR0;
}
