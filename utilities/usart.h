/*
 * usart.h
 *
 * Created: 07/12/2011 15:16:27
 *  Author: Boomber
 */


#ifndef USART_H_
#define USART_H_

void usart_init( unsigned int ubrr);

void usart_send_byte( unsigned char data );
void usart_send_string(const char *str);
void usart_send_int(unsigned int d);

unsigned char usart_receive( void );


#endif /* USART_H_ */
