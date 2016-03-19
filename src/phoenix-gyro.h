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

#ifndef PHOENIX_GYRO_H
#define PHOENIX_GYRO_H

#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include "i2cmaster/i2cmaster.h"

#include "phoenix-globals.h"
#include "delay/delay.h"
#include "uart/uart.h"

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
