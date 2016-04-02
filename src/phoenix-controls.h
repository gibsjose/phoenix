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

//Registers setup for ESC output
//OCR1A = Pin 9, OCR1B = Pin 10, OCR2A = Pin 11, OCR2B = Pin 3
//B (digital pin 8 to 13), C (analog input pins), D (digital pins 0 to 7)

#define PIN_9            (1 << 1)    //Bit to configure Pin 9 as output on PORTB
#define PIN_10           (1 << 2)    //Bit to configure Pin 10 as output on PORTB
#define PIN_11           (1 << 3)    //Bit to configure Pin 11 as output on PORTB
#define PIN_3           (1 << 3)    //Bit to configure Pin 3 as output on PORTD
/* Apparently they are defined on the arduino library already..
#define COM2A1          (1<<7)
#define COM2B1          (1<<5)
#define WGM21           (1<<1)
#define WGM20           (1<<0)
#define CS22           (1<<2)
#define CS21           (1<<1)
*/

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
  double measurement;              // Velocity measurement, that is going to be the gyroscope readout
  double last_error;               // Used internally for the PID_controller calculations
  double accomulated_error;        // Used internally for the PID_controller calculations
}PID_input_t;

typedef struct PID_output_t{
  double ut;                       // Command signal, contribution to be applied to the ESCs
  double error;                    // Error signal
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

typedef struct ESC_outputs_t{
  double esc_1;
  double esc_2;
  double esc_3;
  double esc_4;
}ESC_outputs_t;


//Function declarations
void init_pid_settings(PID_roll_t *, PID_pitch_t *, PID_yaw_t *);
void calculate_pids(gyro_t *, setpoints_t *, PID_roll_t *, PID_pitch_t *, PID_yaw_t *);
void pid_controller(PID_input_t *, PID_settings_t *, PID_output_t *);
void init_esc_pins();
void calculate_esc_pulses_duration(volatile receiver_inputs_t*, PID_roll_t*, PID_pitch_t*,PID_yaw_t*, ESC_outputs_t*);
void calculate_esc_pulses_to_stop_motors(ESC_outputs_t*);
void commandPWMSignals(ESC_outputs_t*);
void PWM_loop(int*);
#endif//PHOENIX_CONTROLS_H
