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
//Comentaris 24/05
/**
1. Channel 5 must reset the integrator
2. Els setpoints es queden posats. Tried with RELAX mode
3. Disable interrupts until after gyro calibration
*/
// Gains for the ROLL PID, are the same as the PITCH PID

#define P_ROLL  2.3 // Fast oscillation = 2.5, lower 50% 1.25. 2nd Round fast oscillation at 2.5
#define I_ROLL 0.06 // Oscillation starting at 1.2.. 50% is 0.06
#define D_ROLL 22 //35 restless, 30 smooth again, lower 25% = 22

#define P_PITCH  2.3// Fast oscillation = 2.5, lower 50% 1.25. 2nd Round fast oscillation at 2.5
#define I_PITCH 0.06 // Oscillation starting at 1.2.. 50% is 0.06
#define D_PITCH 22 // 35 restless, 30 smooth again, lower 25% = 22

#define P_YAW 4 //3 original
#define I_YAW 0.03 //0.02 original
#define D_YAW 0

/*#define P_ROLL  1.4
#define I_ROLL 0.05
#define D_ROLL 15

#define P_PITCH  1.4
#define I_PITCH 0.05
#define D_PITCH 15

#define P_YAW 4
#define I_YAW 0.02
#define D_YAW 0*/

#define UPPER_LIMIT 400
#define LOWER_LIMIT -400

//Registers setup for ESC output

#define PIN_13            (1 << 7)    //DDB7: Bit to configure Pin 13 as output on PORTB on atmega2560
#define PIN_4             (1 << 5)    //DDG5: Bit to configure Pin 4 as output on PORTG on atmega2560
#define PIN_11            (1 << 5)    //DDB5: Bit to configure Pin 11 as output on PORTB on atmega2560
#define PIN_12            (1 << 6)    //DDB6: Bit to configure Pin 12 as output on PORTB on atmega2560

/*USED for atmega328p
#define PIN_9            (1 << 1)    //Bit to configure Pin 9 as output on PORTB
#define PIN_10           (1 << 2)    //Bit to configure Pin 10 as output on PORTB
#define PIN_11           (1 << 3)    //Bit to configure Pin 11 as output on PORTB
#define PIN_3           (1 << 3)    //Bit to configure Pin 3 as output on PORTD
#define PIN_5           (1 << 5)    //Bit to configure Pin 5 as output on PORTD
#define PIN_6           (1 << 6)    //Bit to configure Pin 6 as output on PORTD*/

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
  double desired;                   // Velocity desired , that is going to be the setpoint
  double measured;              // Velocity measured, that is going to be the gyroscope readout
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
void print_pid_settings(PID_settings_t *, PID_settings_t *, PID_settings_t *);
void print_pid_outputs(PID_output_t *,PID_output_t *,PID_output_t *);
void print_pid_inputs(PID_input_t *,PID_input_t *,PID_input_t *);
void calculate_pids(gyro_t *, setpoints_t *, PID_roll_t *, PID_pitch_t *, PID_yaw_t *);
void pid_controller(PID_input_t *, PID_settings_t *, PID_output_t *);
void init_esc_registers(void);
void calculate_esc_pulses_duration(receiver_inputs_t*, PID_roll_t*, PID_pitch_t*,PID_yaw_t*, ESC_outputs_t*);
void debug_calculate_esc_pulses_duration(receiver_inputs_t*, ESC_outputs_t*);
void calculate_esc_pulses_to_stop_motors(ESC_outputs_t*);
void commandPWMSignals(ESC_outputs_t*);
void PWM_resetRegisters(void);
void PWM_loop(int*);
void reset_accoumulated_error_PID_input(PID_roll_t *, PID_pitch_t *, PID_yaw_t *);
#endif//PHOENIX_CONTROLS_H
