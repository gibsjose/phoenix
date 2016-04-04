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

#include "phoenix-util.h"

extern void init_analog_input_pins(){
  // Select Vref=AVcc
ADMUX |= (1<<REFS0);
//set prescaller to 128 and enable ADC
ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);

}
extern double readBatteryVoltage(){
    //select ADC channel with safety mask
    ADMUX = 0x41; //4 because we use Vcc as a reference for the DAC, 0x01 beause we read the voltage at A1
    //single conversion mode
    ADCSRA |= (1<<ADSC);
    // wait until ADC conversion is complete
    while( ADCSRA & (1<<ADSC) );
    // Vx = (ADC / 1023) * 5
    //Vbatt = 2.5 · Vx + VDiode ( x2.5 because of R3 = 1.5K and R2 = 1K)
    // VDiode = 0.65 V
     return (((double)ADC/1023) * 5 * 2.5 + 0.65);
}

extern void LED_ON(){
	PORTB |= PIN_12;

}
extern void LED_OFF(){
	PORTB &= !PIN_12;
}
