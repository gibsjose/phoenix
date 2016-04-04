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

#include "phoenix-controls.h"

//Initialize PID settings for pitch, roll, yaw
void init_pid_settings(PID_roll_t *roll, PID_pitch_t *pitch, PID_yaw_t *yaw) {
  //Initialize settings
  PID_settings_t *settings;

  //Roll
  settings = &(roll->settings);
  settings->KP = P_ROLL;
  settings->KI = I_ROLL;
  settings->KD = D_ROLL;
  settings->upper_limit = UPPER_LIMIT;
  settings->lower_limit = LOWER_LIMIT;

  //Pitch
  settings = &(pitch->settings);
  settings->KP = P_PITCH;
  settings->KI = I_PITCH;
  settings->KD = D_PITCH;
  settings->upper_limit = UPPER_LIMIT;
  settings->lower_limit = LOWER_LIMIT;

  //Yaw
  settings = &(yaw->settings);
  settings->KP = P_YAW;
  settings->KI = I_YAW;
  settings->KD = D_YAW;
  settings->upper_limit = UPPER_LIMIT;
  settings->lower_limit = LOWER_LIMIT;
}

//Calculates the PID output from the input and settings
void pid_controller(PID_input_t * PID_input, PID_settings_t * PID_settings, PID_output_t * PID_output){

  PID_output->error = PID_input->target - PID_input->measurement;               //Calculate error signal

  // Proportional contribution
  PID_output->P_contribution = PID_settings->KP * PID_output->error;

  // Integral contribution
  PID_output->I_contribution = PID_input->accomulated_error;                    //Dumb but clearer
  PID_output->I_contribution = PID_output->I_contribution  + PID_settings->KI * PID_output->error;

  if(PID_output->I_contribution > PID_settings->upper_limit){ // Do not let the Integral gain build up too much
    PID_output->I_contribution = PID_settings->upper_limit;
  }
  if(PID_output->I_contribution < PID_settings->lower_limit){
    PID_output->I_contribution = PID_settings->lower_limit;
  }
  PID_input->accomulated_error = PID_output->I_contribution; //To be used next time we compte the contribution

  // Derivative contribution
  PID_output->D_contribution = PID_settings->KD * (PID_output->error - PID_input->last_error);
  PID_input->last_error = PID_output->error;

  // Add up all the contributions to form the command signal
  PID_output->ut = PID_output->P_contribution + PID_output->I_contribution + PID_output->D_contribution;

  // Make sure we are not over the limits, coerce output
  if(PID_output->ut > PID_settings->upper_limit){
    PID_output->ut = PID_settings->upper_limit;
  }
  if(PID_output->ut < PID_settings->lower_limit){
    PID_output->ut = PID_settings->lower_limit;
  }
}

//Performs calculation of PID outputs from gyro and setpoints
void calculate_pids(gyro_t * gyro, setpoints_t * setpoints, PID_roll_t * roll, PID_pitch_t * pitch, PID_yaw_t * yaw) {
  // ROLL Set inputs
  roll->input.target = setpoints->roll;
  roll->input.measurement = gyro->roll_filtered;

  //Populate the PID output for the ROLL axis
  pid_controller(&(roll->input), &(roll->settings), &(roll->output));

  // PITCH set inputs
  pitch->input.target = setpoints->pitch;
  pitch->input.measurement = gyro->pitch_filtered;

  //Populate the PID output for the PITCH axis
  pid_controller(&(pitch->input), &(pitch->settings), &(pitch->output));

  // YAW set inputs
  yaw->input.target = setpoints->yaw;
  yaw->input.measurement = gyro->yaw_filtered;

  //Populate the PID output for the YAW axis
  pid_controller(&(yaw->input), &(yaw->settings), &(yaw->output));
}

void init_esc_pins(){
  //Registers setup for ESC output
  //OC1A = Pin 9, OC1B = Pin 10, OC0A = Pin 6, OC0B = Pin 5
  //B (digital pin 8 to 13), C (analog input pins), D (digital pins 0 to 7)
  DDRB |= PIN_9 | PIN_10;
  DDRD |= PIN_5 | PIN_6;
  //Reset timer counters, so the PWM signals will be in phase
  TCNT1 = 0;
  TCNT0 = 0;

  //Configure Timer 1: Pins 9 & 10
  // CS12, CS11, CS10 = 100 (prescaler = 256)
  // WGM13, WGM12, WGM11, WGM10 = 0101 (Mode 5)
  TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10);
  TCCR1B = (1<<WGM12) | (1<<CS12);

  //Configure Timer 0: Pins 5 & 6
  // CS02, CS01, CS00 = 100 (prescaler = 256)
  // WGM02, WGM01, WGM00 = 011 (Mode 3)

  TCCR0A = (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00); //Fast PWM
  TCCR0B = (1<<CS02); //256 preescaler

  //Initial values under 1000 us, motors stopped
  OCR0A = 61;
  OCR0B = 61;
  OCR1A = 61;
  OCR1B = 61;
}

void calculate_esc_pulses_duration(volatile receiver_inputs_t *receiver, PID_roll_t * roll, PID_pitch_t * pitch, PID_yaw_t * yaw, ESC_outputs_t *esc){

  if (receiver->gas_scaled > 1800) receiver->gas_scaled = 1800;                            //We need some room to keep control at full throttle.
  esc->esc_1 = receiver->gas_scaled + roll->output.ut - pitch->output.ut + yaw->output.ut; //Calculate the pulse for esc 1 (front-left - CCW)
  esc->esc_2 = receiver->gas_scaled - roll->output.ut - pitch->output.ut - yaw->output.ut; //Calculate the pulse for esc 2 (front-right - CW)
  esc->esc_3 = receiver->gas_scaled - roll->output.ut + pitch->output.ut + yaw->output.ut; //Calculate the pulse for esc 3 (rear-right - CCW)
  esc->esc_4 = receiver->gas_scaled + roll->output.ut + pitch->output.ut - yaw->output.ut; //Calculate the pulse for esc 4 (rear-left - CW)

/* Can go away soon...
  esc_4 = throttle - pid_output_roll + pid_output_pitch  - pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
  esc_1 = throttle  + pid_output_roll + pid_output_pitch + pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
  esc_2 = throttle + pid_output_roll - pid_output_pitch - pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
  esc_3 = throttle - pid_output_roll - pid_output_pitch  + pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)*/


  /*   @TODO compensate for voltage drop if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
  esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
  esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
  esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
  esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
}*/

if (esc->esc_1 < 1200) esc->esc_1 = 1200;                                         //Keep the motors running.
if (esc->esc_2 < 1200) esc->esc_2 = 1200;                                         //Keep the motors running.
if (esc->esc_3 < 1200) esc->esc_3 = 1200;                                         //Keep the motors running.
if (esc->esc_4 < 1200) esc->esc_4 = 1200;                                         //Keep the motors running.

if(esc->esc_1 > 2000)esc->esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
if(esc->esc_2 > 2000)esc->esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
if(esc->esc_3 > 2000)esc->esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
if(esc->esc_4 > 2000)esc->esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.
}

void calculate_esc_pulses_to_stop_motors(ESC_outputs_t *esc){
  esc->esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
  esc->esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
  esc->esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
  esc->esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
}


void commandPWMSignals(ESC_outputs_t *esc){
  double temp;
  temp = (256*esc->esc_1)/4096 -1;
  OCR1A = round (temp);                                                           //OCR1A = Pin 9
  temp = (256*esc->esc_2)/4096 -1;
  OCR1B = round (temp);                                                           //OCR1B = Pin 10
  temp = (256*esc->esc_3)/4096 -1;
  OCR0A = round (temp);                                                            //OCR2A = Pin 11
  temp = (256*esc->esc_4)/4096 -1;
  OCR0B = round (temp);                                                            //OCR2B = Pin 3
}

void PWM_loop(int *sign){
  //Dumb loop that changes the PWM duty cycle
  OCR1A = OCR1A + *sign;
  if(OCR1A==255){
    *sign = -1;
  }
  if(OCR1A==0){
    *sign = 1;
  }


  }
