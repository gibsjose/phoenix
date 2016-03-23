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

//setpoints_t * setpoints will be a global volatile
void calculate_pids(gyro_t * gyro, PID_input_t * pid_input_roll, PID_input_t * pid_input_pitch, PID_input_t * pid_input_yaw,
  PID_settings_t * pid_settings_roll, PID_settings_t * pid_settings_yaw, PID_settings_t * pid_settings_pitch, PID_output_t * pid_output_roll
  , PID_output_t * pid_output_pitch, PID_output_t * pid_output_yaw ){

// ROLL Set inputs
  pid_input_roll->target = setpoints->roll;
  pid_input_roll->measurement = gyro->roll_filtered;
//Populate the PID output for the ROLL axis
  pid_controller(pid_input_roll, pid_settings_roll, pid_output_roll);

// PITCH set inputs
  pid_input_pitch->target = setpoints->pitch;
  pid_input_pitch->measurement = gyro->pitch_filtered;
//Populate the PID output for the PITCH axis
    pid_controller(pid_input_pitch, pid_settings_pitch, pid_output_pitch);

// YAW set inputs
    pid_input_yaw->target = setpoints->yaw;
    pid_input_yaw->measurement = gyro->yaw_filtered;
//Populate the PID output for the YAW axis
    pid_controller(pid_input_yaw, pid_settings_yaw, pid_output_yaw);
}

//Print something
//  uart_puts("something");

/*Control loop sequence

void calculate_pids(gyro_t * gyro, PID_input_t * pid_input_roll, PID_input_t * pid_input_pitch, PID_input_t * pid_input_yaw,
  PID_settings_t * pid_settings_roll, PID_settings_t * pid_settings_yaw, PID_settings_t * pid_settings_pitch, PID_output_t * pid_output_roll
  , PID_output_t * pid_output_pitch , PID_output_t * pid_output_yaw )


*/
