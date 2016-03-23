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
