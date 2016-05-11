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

void init_analog_input_pins(){
  // Select Vref=AVcc
ADMUX |= (1<<REFS0);
//set prescaller to 128 and enable ADC
ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);

}
double readBatteryVoltage(){
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

void init_LEDs_as_outputs(){
  DDRB |= PIN_10;
  DDRL |= PIN_49;

}
void LED_RED_ON(){
	PORTL |= PIN_49;

}
void LED_RED_OFF(){
	PORTL &= ~PIN_49;
}

void LED_RED_CHANGE_STATUS(){
  if((PORTL & (PIN_49)) == 0 ){
     PORTL |= (PIN_49);
  }
  else{
      PORTL &= ~(PIN_49);
    }
}

void LED_GREEN_ON(){
	PORTB |= PIN_10;

}
void LED_GREEN_OFF(){
	PORTB &= ~PIN_10;
}

void LED_GREEN_CHANGE_STATUS(){

  if((PORTB & (PIN_10)) == 0 ){
     PORTB |= (PIN_10);
  }
  else{
      PORTB &= ~(PIN_10);
    }
}
