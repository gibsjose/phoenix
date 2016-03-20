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
#include "delay/delay.h"
#include "uart/uart.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//PID gains and settings
typedef struct PID_Settings_t {
    //Coeficients
    double P;           // Proportional gain
    double I;           // Integral gain
    double D;           // Derivative gain

    //Limits of the contribution of each PID controller
    double upper_limit;
    double lower_limit;
} PID_Settings_t;

PID_Settings_t pid_settings_roll = {
    1.4,               // Gain setting for the roll P-controller
    0.05,              // Gain setting for the roll I-controller
    15,                // Gain setting for the roll D-controller
    400,               // Maximum output of the PID-controller
    -400,              // Minimum output of the PID-controller
};

PID_Settings_t pid_settings_pitch = pid_settings_roll ;

PID_Settings_t pid_settings_yaw = {
    4,                 // Gain setting for the yaw P-controller
    0.02,              // Gain setting for the yaw I-controller
    0,                 // Gain setting for the yaw D-controller
    400,               // Maximum output of the PID-controller
    -400,              // Minimum output of the PID-controller
};

//Remote controller receiver inputs for the Futaba 7C transmitter and Futaba R617FS receiver.
typedef struct receiver_t {
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
} receiver_t;

//These values are used to fit the reciever inputs into a range from 1000 to 2000.
//The pilot should set these values on a startup calibration procedure
#define SCALE_CENTER_ROLL = 1500;
#define SCALE_MAX_ROLL = 2000;
#define SCALE_MIN_ROLL = 1000;

#define SCALE_CENTER_PITCH = 1500;
#define SCALE_MAX_PITCH = 2000;
#define SCALE_MIN_PITCH = 1000;

#define SCALE_CENTER_GAS = 1500;
#define SCALE_MAX_GAS = 2000;
#define SCALE_MIN_GAS = 1000;

#define SCALE_CENTER_YAW = 1500;
#define SCALE_MAX_YAW = 2000;
#define SCALE_MIN_YAW = 1000;


//Question: What does return OK do when the expected return is a uint8_t ?
void receiver_read(receiver_t *);
void receiver_scale(receiver_t *);




//I2C Address
#define GYRO_ADDR           0xD6    //Gyroscope I2C Address: 0xD6 = 214 = 0b11010010

//Configuration
#define GYRO_CALIBRATION_STEPS  200    ///Number of measurements during gyro calibration

#define ROLL_COEFFICIENT     1.0        //Non-inverted
#define PITCH_COEFFICIENT   -1.0        //Inverted
#define YAW_COEFFICIENT     -1.0        //Inverted

//Register Definitions
#define GYRO_CTRL_1_REG     0x20        //@TODO Comments for these
#define GYRO_CTRL_2_REG     0x21
#define GYRO_CTRL_3_REG     0x22
#define GYRO_CTRL_4_REG     0x23
#define GYRO_CTRL_5_REG     0x24

//Configuration Definitions
////Control Register 1
#define GYRO_PD             (1 << 3)    //Power enable
#define GYRO_ZEN            (1 << 2)    //Z axis enable
#define GYRO_YEN            (1 << 1)    //Y axis enable
#define GYRO_XEN            (1 << 0)    //X axis enable

////Control Register 4
#define GYRO_BDU            (1 << 7)    //Block data update (0: contiguous update, 1: not updated until read)
#define GYRO_FS1            (1 << 5)    //Full Scale Selection Bits (5:4)
#define GYRO_FS0            (1 << 4)    //  00 -> 245 dps
                                        //  01 -> 500 dps
                                        //  1x -> 2000 dps

#define GYRO_AUTO_INC       (1 << 7)    //Auto increment
#define GYRO_READ_REG       0x28        //Read register (0x28)

//Gyro data
typedef struct gyro_t {
    //Pitch, roll, yaw
    double roll;            //Roll (X)
    double pitch;           //Pitch (Y)
    double yaw;             //Yaw (Z)

    //Calibration offsets
    double roll_offset;
    double pitch_offset;
    double yaw_offset;
} gyro_t;

//Function declarations
uint8_t gyro_init(gyro_t *);
uint8_t gyro_calibrate(gyro_t *);
uint8_t gyro_read(gyro_t *);
void gyro_print(gyro_t *);
void gyro_loop(gyro_t *);

#endif//PHOENIX_GYRO_H
