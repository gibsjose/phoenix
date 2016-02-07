#include "delay.h"

int delay_us(unsigned long int microseconds) {
	volatile unsigned cycles = microseconds/64;
	TCCR2A	= 0x00;
	TCCR2B =_BV(CS22) | _BV(CS21) | _BV(CS20); // set timer to use internal clock with 1:1024 pre-scale
	if(microseconds < 16321) {
		TCNT2 = 0;
		OCR2A = cycles;
		TIFR2 = _BV(OCF2A); // Set to clear bit 1
		while ((TIFR2 & _BV(OCF2A)) == 0); // NULL
		return(0);
	}
	else {
		TCNT2 = 0;
		OCR2A = 255;
		TIFR2 = _BV(OCF2A); // Set to clear bit 1
		while ((TIFR2 & _BV(OCF2A)) == 0); // NULL
		return(delay_us(microseconds - 16320));
	}
}
