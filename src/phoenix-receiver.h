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

#ifndef PHOENIX_RECEIVER_H
#define PHOENIX_RECEIVER_H

#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include "i2cmaster/i2cmaster.h"

#include "phoenix-globals.h"
#include "phoenix-gyro.h"
#include "delay/delay.h"
#include "uart/uart.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Phoenix operation PILOTING_MODE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//We will limit the maximum angular speed that the Roll, Pitch and Yaw receivers channel can request to Phoenix
//See calculate_setpoints(receiver_inputs_t * receiver, setpoints_t * setpoints) for details

//Values a gyro range of +-500 dps.
#define EXTREME  1.5     //Allows 500/1.5 = 333.3 degrees per second max angular rate (= max setpoint)
#define SPORT  3         //Allows 500/3 = 166.67 degrees per second max angular rate (= max setpoint)
#define RELAX  4         //Allows 500/4 = 125 degrees per second max angular rate (= max setpoint)

#define PILOTING_MODE RELAX


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Define pins used for the PWM receiver signals
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PIN_14            (1 << 1)    //DDJ1: Bit to configure Pin 14 as output on PORTJ on atmega2560
#define PIN_15            (1 << 0)    //DDJ0: Bit to configure Pin 15 as output on PORTJ on atmega2560
#define PIN_50            (1 << 3)    //DDB3: Bit to configure Pin 49 as output on PORTB on atmega2560
#define PIN_52            (1 << 1)    //DDB1: Bit to configure Pin 49 as output on PORTB on atmega2560
#define PIN_51            (1 << 2)    //DDB2: Bit to configure Pin 51 as output on PORTB on atmega2560
#define PIN_53            (1 << 0)    //DDB0: Bit to configure Pin 53 as output on PORTB on atmega2560


/* Previously used for the Input Capture Registers */
//#define PIN_48            (1 << 1)    //DDL1: Bit to configure Pin 48 as output on PORTL on atmega2560
//#define PIN_49            (1 << 0)    //DDL0: Bit to configure Pin 49 as output on PORTL on atmega2560




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Radio receiver adjustment values
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//These values are used to fit the reciever inputs into a range from 1000 to 2000.
//The pilot should set these values on a startup calibration procedure
#define SCALE_CENTER_ROLL  1500
#define SCALE_MAX_ROLL  1908
#define SCALE_MIN_ROLL  1092

#define SCALE_CENTER_PITCH  1500
#define SCALE_MAX_PITCH  1908
#define SCALE_MIN_PITCH  1098

#define SCALE_CENTER_GAS  1520
#define SCALE_MAX_GAS  1916
#define SCALE_MIN_GAS  1116

#define SCALE_CENTER_YAW  1496
#define SCALE_MAX_YAW  1908
#define SCALE_MIN_YAW  1096

// Some remotes have the receiver inputs are reversed
#define REVERSE_ROLL  1 //1 channel not reversed, -1 channel reversed
#define REVERSE_PITCH  -1
#define REVERSE_GAS  -1
#define REVERSE_YAW  1

//Remote controller receiver inputs for the Futaba 7C transmitter and Futaba R617FS receiver.
typedef struct receiver_inputs_t {
    // roll, pitch, gas and yaw receiver inputs.
    double roll;            // Roll channel 1
    double pitch;           // Pitch channel 2
    double gas;             // Throttle channel 3
    double yaw;             // Yaw channel 4
    double channel5;
    double channel6;

    // The raw inputs will not exactly be on the [1000,2000] microsecond range. The scaled values will be on that range
    double roll_scaled;            // Roll channel 1
    double pitch_scaled;           // Pitch channel 2
    double gas_scaled;             // Throttle channel 3
    double yaw_scaled;             // Yaw channel 4
    double channel5_scaled;
    double channel6_scaled;

} receiver_inputs_t;

typedef struct setpoints_t{
  // The receiver values from [1000 to 2000] sharp,  are scaled again and transformed into actual setpoints for the PID controller with a different range
  double roll;
  double pitch;
  double yaw;
}setpoints_t;

//Function declarations
//void receiver_read(receiver_inputs_t *);
void receiver_scale(receiver_inputs_t *);
void calculate_setpoints(receiver_inputs_t *, setpoints_t *);
void receiver_print(receiver_inputs_t *);
void init_receiver_registers(void);
void setpoints_print(setpoints_t *);
void receiver_memset(receiver_inputs_t *);


#endif//PHOENIX_RECEIVER_H
