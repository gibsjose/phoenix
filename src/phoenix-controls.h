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
#include "phoenix.h"
#include "phoenix-receiver.h"
#include "delay/delay.h"
#include "uart/uart.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Phoenix operation MODE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//We will limit the maximum angular speed that the Roll, Pitch and Yaw receivers channel can request to Phoenix
//See calculate_setpoints(receiver_inputs_t * receiver, setpoints_t * setpoints) for details

//Values a gyro range of +-500 dps.
#define EXTREME  1.5     //Allows 500/1.5 = 333.3 degrees per second max angular rate (= max setpoint)
#define SPORT  3         //Allows 500/3 = 166.67 degrees per second max angular rate (= max setpoint)
#define RELAX  4         //Allows 500/4 = 125 degrees per second max angular rate (= max setpoint)

#define MODE SPORT

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

//Function declarations
void calculate_pids(gyro_t *,  PID_input_t *,  PID_input_t *,  PID_input_t *,
   PID_settings_t *, PID_settings_t *, PID_settings_t *, PID_output_t *, PID_output_t *, PID_output_t *);
void pid_controller(PID_input_t *, PID_settings_t *, PID_output_t *);




//Question: What does return OK do when the expected return is a uint8_t ?
/*//Function declarations
uint8_t gyro_init(gyro_t *);
*/

#endif//PHOENIX_CONTROLS_H
