/**********************************************************
*   Phoenix FSW - Controls
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

#include "phoenix-receiver.h"

void init_receiver_registers(){
  /**************************************************
 * PWM INPUT Capture *
 **************************************************/
 /**************************************************
* Setup PCINT10, Pin 14  *
* Setup PCINT9, Pin 15   *
* Setup PCINT3, Pin 50   *
* Setup PCINT1, Pin 52   *
**************************************************/
uart_puts("Configuring receiver registers\r\n");

TCCR5B = 0;
TCCR5A = 0;
TCCR5B |=  (1<<CS51)| (1<<CS50);
TCCR5C = 0;

//Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
PCICR |= (1 << PCIE1);    // set PCIE0 to enable PCMSK1 scan
PCMSK1 |= (1 << PCINT9);  // set PCINT9 (digital input 15) to trigger an interrupt on state change
PCMSK1 |= (1 << PCINT10); // set PCINT10 (digital input 14)to trigger an interrupt on state change
PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK1 scan
PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 50) to trigger an interrupt on state change
PCMSK0|= (1 << PCINT1);  // set PCINT1 (digital input 52)to trigger an interrupt on state change
PCMSK0|= (1 << PCINT0);  // set PCINT0 (digital input 53)to trigger an interrupt on state change
PCMSK0|= (1 << PCINT2);  // set PCINT2 (digital input 51)to trigger an interrupt on state change

  //chip Pin PJ1, arduino pin 14, channel Pitch
  DDRJ &= ~PIN_14;
  //chip Pin PJ0, arduino pin 15, channel Yaw
  DDRJ &= ~PIN_15;
  //chip Pin PB3, arduino pin 50, channel Pitch
  DDRB  &= (~PIN_50);
  //chip Pin PB1, arduino pin 52, channel Pitch
  DDRB &= (~PIN_52);
  //chip Pin PB1, arduino pin 52, channel Pitch
  DDRB &= (~PIN_53);
  //chip Pin PB1, arduino pin 52, channel Pitch
  DDRB &= (~PIN_51);


}


void receiver_memset(receiver_inputs_t * receiver){
  receiver->roll = 0;
  receiver->roll_scaled = 0;
  receiver->pitch = 0;
  receiver->pitch_scaled = 0;
  receiver->gas = 0;
  receiver->gas_scaled = 0;
  receiver->yaw = 0;
  receiver->yaw_scaled = 0;
}

//Scale receiver inputs
void receiver_scale(receiver_inputs_t * receiver) {
    //Scale ROLL
    if(receiver->roll < SCALE_CENTER_ROLL){                 //The actual receiver value is lower than the center value
        if(receiver->roll < SCALE_MIN_ROLL){                  //The actual receiver value is lower than the minimum value
            receiver->roll_scaled = 1000;             //Limit the lowest value to the value that was detected during setup
        }
        else{                                                 //Calculate and scale the actual value to a 1000 - 2000us value
            double difference = ((SCALE_CENTER_ROLL - receiver->roll) * (double)500.00) / (SCALE_CENTER_ROLL - SCALE_MIN_ROLL);
            receiver->roll_scaled = 1500 - (double) (difference * REVERSE_ROLL);
            double test = 1500 - (double) (difference * REVERSE_ROLL);
            if( (test > 1550) || (test< 1450)){uart_putd(difference);}
        }
    }
    else if(receiver->roll > SCALE_CENTER_ROLL){            //The actual receiver value is higher than the center value
        if(receiver->roll > SCALE_MAX_ROLL){                  //The actual receiver value is higher than the maximum value
            receiver->roll_scaled = 2000;             //Limit the highest value to the value that was detected during setup
        }
        else{                                                 //Calculate and scale the actual value to a 1000 - 2000us value
            double difference = ((receiver->roll - SCALE_CENTER_ROLL) * 500) / (SCALE_MAX_ROLL - SCALE_CENTER_ROLL);
            receiver->roll_scaled = 1500 + difference * REVERSE_ROLL;
        }
      }
    else{
        receiver->roll_scaled = 1500;
        }

    //Scale PITCH
    if(receiver->pitch < SCALE_CENTER_PITCH){                //The actual receiver value is lower than the center value
        if(receiver->pitch < SCALE_MIN_PITCH){                  //The actual receiver value is lower than the minimum value
          if(REVERSE_PITCH == 1){
            receiver->pitch_scaled = 1000;             //Limit the lowest value to the value that was detected during setup
        }
        else{
              receiver->pitch_scaled = 2000;
        }
      }
        else{                                                 //Calculate and scale the actual value to a 1000 - 2000us value
            double difference = ((SCALE_CENTER_PITCH - receiver->pitch) * 500) / (SCALE_CENTER_PITCH - SCALE_MIN_PITCH);
            receiver->pitch_scaled = 1500 - difference * REVERSE_PITCH;
        }
    }
    else if(receiver->pitch > SCALE_CENTER_PITCH){            //The actual receiver value is higher than the center value
        if(receiver->pitch > SCALE_MAX_PITCH){                  //The actual receiver value is higher than the maximum value
          if(REVERSE_PITCH == 1){
            receiver->pitch_scaled = 2000;             //Limit the highest value to the value that was detected during setup
        }
          else{
            receiver->pitch_scaled = 1000;             //Limit the highest value to the value that was detected during setup
          }
      }
        else{                                                 //Calculate and scale the actual value to a 1000 - 2000us value
            double difference = ((receiver->pitch - SCALE_CENTER_PITCH) * 500) / (SCALE_MAX_PITCH - SCALE_CENTER_PITCH);
            receiver->pitch_scaled = 1500 + difference * REVERSE_PITCH;
        }
    }
    else{
        receiver->pitch_scaled = 1500;
    }

    //Scale GAS
    if(receiver->gas < SCALE_CENTER_GAS){                 //The actual receiver value is lower than the center value
        if(receiver->gas < SCALE_MIN_GAS){
          if(REVERSE_GAS == 1) {                  //The actual receiver value is lower than the minimum value
            receiver->gas_scaled = 1000;             //Limit the lowest value to the value that was detected during setup
        }
        else{
            receiver->gas_scaled = 2000;
        }
      }
        else{                                                 //Calculate and scale the actual value to a 1000 - 2000us value
            double difference = ((SCALE_CENTER_GAS - receiver->gas) * 500) / (SCALE_CENTER_GAS - SCALE_MIN_GAS);
            receiver->gas_scaled = 1500 - difference * REVERSE_GAS;
        }
    }
    else if(receiver->gas > SCALE_CENTER_GAS){            //The actual receiver value is higher than the center value
        if(receiver->gas > SCALE_MAX_GAS){                  //The actual receiver value is higher than the maximum value
          if(REVERSE_GAS == 1){
            receiver->gas_scaled = 2000;             //Limit the highest value to the value that was detected during setup
        }
        else{
          receiver->gas_scaled = 1000;
        }
      }
        else{                                                 //Calculate and scale the actual value to a 1000 - 2000us value
            double difference = ((receiver->gas - SCALE_CENTER_GAS) * 500) / (SCALE_MAX_GAS - SCALE_CENTER_GAS);
            receiver->gas_scaled = 1500 + difference * REVERSE_GAS;
        }
    }
    else{
        receiver->gas_scaled = 1500;
    }

    //Scale YAW
    if(receiver->yaw < SCALE_CENTER_YAW){                 //The actual receiver value is lower than the center value
        if(receiver->yaw < SCALE_MIN_YAW){                  //The actual receiver value is lower than the minimum value
            receiver->yaw_scaled = 1000;             //Limit the lowest value to the value that was detected during setup
        }
        else{                                                 //Calculate and scale the actual value to a 1000 - 2000us value
            double difference = ((SCALE_CENTER_YAW - receiver->yaw) * 500) / (SCALE_CENTER_YAW - SCALE_MIN_YAW);
            receiver->yaw_scaled = 1500 - difference * REVERSE_YAW;
        }
    }
    else if(receiver->yaw > SCALE_CENTER_YAW){            //The actual receiver value is higher than the center value
        if(receiver->yaw > SCALE_MAX_YAW){                  //The actual receiver value is higher than the maximum value
            receiver->yaw_scaled = 2000;             //Limit the highest value to the value that was detected during setup
        }
        else{                                                 //Calculate and scale the actual value to a 1000 - 2000us value
            double difference = ((receiver->yaw - SCALE_CENTER_YAW) * 500) / (SCALE_MAX_YAW - SCALE_CENTER_YAW);
            receiver->yaw_scaled = 1500 + difference * REVERSE_YAW;
        }
    }
    else{
        receiver->yaw_scaled = 1500;
    }

    receiver->channel5_scaled = receiver->channel5;
    receiver->channel6_scaled = receiver->channel6;

}

//Calculate the setpoints (divide by the PILOTING_MODE)
//@TODO Optimizations on this division (power of two optimization)
void calculate_setpoints(receiver_inputs_t * receiver, setpoints_t * setpoints) {
    //Roll Setpoints
    //We need a little dead band of 16us for better results.
    if(receiver->roll_scaled > 1508){
        setpoints->roll = (receiver->roll_scaled - 1508)/PILOTING_MODE;
    }
    else if(receiver->roll_scaled < 1492){
        setpoints->roll = (receiver->roll_scaled - 1492)/PILOTING_MODE;
    }
    else{
        setpoints->roll = 0;
    }

    //Pitch Setpoints
    //We need a little dead band of 16us for better results.
    if(receiver->pitch_scaled > 1508){
        setpoints->pitch = (receiver->pitch_scaled - 1508)/PILOTING_MODE;
    }
    else if(receiver->roll_scaled < 1492){
        setpoints->pitch = (receiver->pitch_scaled - 1492)/PILOTING_MODE;
    }
    else{
        setpoints->pitch = 0;
    }

    //Yaw Setpoints
    //We need a little dead band of 16us for better results.
    if(receiver->yaw_scaled > 1508){
        setpoints->yaw = (receiver->yaw_scaled - 1508)/PILOTING_MODE;
    }
    else if(receiver->yaw_scaled < 1492){
        setpoints->yaw = (receiver->yaw_scaled - 1492)/PILOTING_MODE;
    }
    else{
        setpoints->yaw = 0;
    }
}

//Print the receiver and scaled receiver data
void receiver_print(receiver_inputs_t * receiver) {
    uart_puts("-------- Receiver Inputs --------\r\n");
    uart_puts("Roll: ");
    uart_putd(receiver->roll);
    uart_puts("\t");
    uart_putd(receiver->roll_scaled);
    uart_puts(" ---\r\n");


    uart_puts("Pitch: ");
    uart_putd(receiver->pitch);
    uart_puts("\t");
    uart_putd(receiver->pitch_scaled);
    uart_puts(" ---\r\n");

    uart_puts("Yaw: ");
    uart_putd(receiver->yaw);
    uart_puts("\t");
    uart_putd(receiver->yaw_scaled);
    uart_puts(" ---\r\n");

    uart_puts("Gas: ");
    uart_putd(receiver->gas);
    uart_puts("\t");
    uart_putd(receiver->gas_scaled);
    uart_puts(" ---\r\n");

    uart_puts("Channel 5: ");
    uart_putd(receiver->channel5);
    uart_puts("\t");
    uart_putd(receiver->channel5_scaled);
    uart_puts(" ---\r\n");

    uart_puts("Channel 6: ");
    uart_putd(receiver->channel6);
    uart_puts("\t");
    uart_putd(receiver->channel6_scaled);
    uart_puts(" ---\r\n");

}

void setpoints_print(setpoints_t * setpoints) {
  uart_puts("-------- Setpoints --------\r\n");
  uart_puts("Roll: ");
  uart_putd(setpoints->roll);
  uart_puts(" ---\t");

  uart_puts("Pitch: ");
  uart_putd(setpoints->pitch);
  uart_puts(" ---\t");

  uart_puts("Yaw: ");
  uart_putd(setpoints->yaw);
  uart_puts(" ---\r\n");
}
