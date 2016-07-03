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
#include "phoenix-util.h"
#include "delay/delay.h"
#include "uart/uart.h"

//I2C Address
#define GYRO_ADDR           0xD6    //Gyroscope I2C Address: 0xD6 = 214 = 0b11010010

//Configuration
#define GYRO_CALIBRATION_STEPS  4000    ///Number of measurements during gyro calibration

#define ROLL_COEFFICIENT    1.0        //Non-inverted
#define PITCH_COEFFICIENT   1.0        //Non-inverted
#define YAW_COEFFICIENT     -1.0        //Non-inverted


//Register Definitions
#define GYRO_CTRL_1_REG     0x20        //@TODO Comments for these
#define GYRO_CTRL_2_REG     0x21
#define GYRO_CTRL_3_REG     0x22
#define GYRO_CTRL_4_REG     0x23
#define GYRO_CTRL_5_REG     0x24

//Configuration Definitions
// See datasheet at https://www.adafruit.com/datasheets/L3GD20H.pdf pg 36
////Control Register 1
#define GYRO_PD             (1 << 3)    //Power enable
#define GYRO_ZEN            (1 << 2)    //Z axis enable
#define GYRO_YEN            (1 << 1)    //Y axis enable
#define GYRO_XEN            (1 << 0)    //X axis enable

////Control Register 4
#define GYRO_BDU            (1 << 7)    //Block data update (0: contiguous update, 1: not updated until read)
#define GYRO_FS1            (0 << 5)    //Full Scale Selection Bits (5:4)
#define GYRO_FS0            (1 << 4)    //  00 -> 245 dps
                                        //  01 -> 500 dps
                                        //  1x -> 2000 dps

//Gyro readout scale coefficient transform gyro readout counts into degrees per second
#define SCALE_COEFFICIENT   0.0175 // 17.5 mili-dregrees per second per count if using  GYRO_FS1 = 0 and GYRO_FS0 = 1

//We low-pass filter the velocity read from the gyro. 0.8 and FILTER_COEFFICIENT = 0.2 are good values
#define FILTER_COEFFICIENT  0.8  //That is the weight we will assign to the new gyro readouts

#define GYRO_AUTO_INC       (1 << 7)    //Auto increment
#define GYRO_READ_REG       0x28        //Read register (0x28)

//Gyro data
typedef struct gyro_t {
    //Pitch, roll, yaw raw readouts
    double roll;            //Roll (X)
    double pitch;           //Pitch (Y)
    double yaw;             //Yaw (Z)

    //Calibration offsets
    double roll_offset;
    double pitch_offset;
    double yaw_offset;

    //Gyro filtered Velocities
    double roll_filtered;
    double pitch_filtered;
    double yaw_filtered;

} gyro_t;

//Function declarations
uint8_t gyro_init(gyro_t *);
uint8_t gyro_calibrate(gyro_t *);
uint8_t gyro_read(gyro_t *);
void gyro_scale(gyro_t *);
void gyro_scale_offset(gyro_t *);
void gyro_filter(gyro_t *);
void gyro_print(gyro_t *);
void gyro_print_offsets(gyro_t *);
void gyro_loop(gyro_t *);
void gyro_read_scaleOffset_filter(gyro_t *);

#endif//PHOENIX_GYRO_H
