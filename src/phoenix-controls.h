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
#include "delay/delay.h"
#include "uart/uart.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Gains for the ROLL PID, are the same as the PITCH PID
#define P_ROLL  1.4
#define I_ROLL  0.05
#define D_ROLL  15

#define P_YAW  4
#define I_YAW  0.02
#define D_YAW  0

#define UPPER_LIMIT  400
#define LOWER_LIMIT  -400

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Radio receiver adjustment values
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//These values are used to fit the reciever inputs into a range from 1000 to 2000.
//The pilot should set these values on a startup calibration procedure
#define SCALE_CENTER_ROLL  1500
#define SCALE_MAX_ROLL  2000
#define SCALE_MIN_ROLL  1000

#define SCALE_CENTER_PITCH  1500
#define SCALE_MAX_PITCH  2000
#define SCALE_MIN_PITCH  1000

#define SCALE_CENTER_GAS  1500
#define SCALE_MAX_GAS  2000
#define SCALE_MIN_GAS  1000

#define SCALE_CENTER_YAW  1500
#define SCALE_MAX_YAW  2000
#define SCALE_MIN_YAW  1000

// Some remotes have the receiver inputs are reversed
#define REVERSE_ROLL  1 //1 channel not reversed, -1 channel reversed
#define REVERSE_PITCH  1
#define REVERSE_GAS  1
#define REVERSE_YAW  1




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



//Remote controller receiver inputs for the Futaba 7C transmitter and Futaba R617FS receiver.
typedef struct receiver_inputs_t {
    // roll, pitch, gas and yaw receiver inputs.
    double roll;            // Roll channel 1
    double pitch;           // Pitch channel 2
    double gas;             // Throttle channel 3
    double yaw;             // Yaw channel 4

    // The raw inputs will not exactly be on the [1000,2000] microsecond range. The scaled values will be on that range
    double roll_scaled;            // Roll channel 1
    double pitch_scaled;           // Pitch channel 2
    double gas_scaled;             // Throttle channel 3
    double yaw_scaled;             // Yaw channel 4

} receiver_inputs_t;

typedef struct setpoints_t{
  // The receiver values from [1000 to 2000] sharp,  are scaled again and transformed into actual setpoints for the PID controller with a different range
  double roll;
  double pitch;
  double yaw;
}setpoints_t;

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
void receiver_read(receiver_inputs_t *);
void receiver_scale(receiver_inputs_t *);
void calculate_setpoints(receiver_inputs_t *, setpoints_t *);
void calculate_pids(setpoints_t *, gyro_t *);
void pid_controller(PID_input_t *, PID_settings_t *, PID_output_t *);




//Question: What does return OK do when the expected return is a uint8_t ?
/*//Function declarations
uint8_t gyro_init(gyro_t *);
*/

#endif//PHOENIX_CONTROLS_H
