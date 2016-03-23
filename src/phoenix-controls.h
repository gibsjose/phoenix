/**********************************************************
*   Phoenix FSW - Gyroscope
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

#ifndef PHOENIX_CONTROLS_H
#define PHOENIX_CONTROLS_H

#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include "i2cmaster/i2cmaster.h"

#include "phoenix-globals.h"
#include "phoenix-gyro.h"
#include "phoenix-receiver.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Gains for the ROLL PID, are the same as the PITCH PID
#define P_ROLL  1.4
#define I_ROLL 0.05
#define D_ROLL 15

#define P_PITCH  1.4
#define I_PITCH 0.05
#define D_PITCH 15

#define P_YAW 4
#define I_YAW 0.02
#define D_YAW 0

#define UPPER_LIMIT 400
#define LOWER_LIMIT -400

//PID gains and settings data
typedef struct PID_settings_t {
    //Coeficients
    double KP;           // Proportional gain
    double KI;           // Integral gain
    double KD;           // Derivative gain

    //Limits of the contribution of each PID controller
    double upper_limit;
    double lower_limit;
} PID_settings_t;

typedef struct PID_input_t{
  double target;                   // Velocity target , that is going to be the setpoint
  double measurement;               // Velocity measurement, that is going to be the gyroscope readout
  double last_error;               // Used internally for the PID_controller calculations
  double accomulated_error;        // Used internally for the PID_controller calculations
}PID_input_t;

typedef struct PID_output_t{
  double ut;                       // Command signal, contribution to be applied to the ESCs
  double error;                       // Error signal
  double P_contribution;           // Useful telemetry and monitoring data to adjust PID gains
  double I_contribution;           // Useful telemetry and monitoring data to adjust PID gains
  double D_contribution;           // Useful telemetry and monitoring data to adjust PID gains
}PID_output_t;

typedef struct PID_roll_t {
    PID_input_t input;
    PID_settings_t settings;
    PID_output_t output;
}PID_roll_t;

typedef struct PID_pitch_t {
    PID_input_t input;
    PID_settings_t settings;
    PID_output_t output;
}PID_pitch_t;

typedef struct PID_yaw_t {
    PID_input_t input;
    PID_settings_t settings;
    PID_output_t output;
}PID_yaw_t;

//Function declarations
void init_pid_settings(PID_roll_t *, PID_pitch_t *, PID_yaw_t *);
void calculate_pids(gyro_t *, setpoints_t *, PID_roll_t *, PID_pitch_t *, PID_yaw_t *);
void pid_controller(PID_input_t *, PID_settings_t *, PID_output_t *);

#endif//PHOENIX_CONTROLS_H
