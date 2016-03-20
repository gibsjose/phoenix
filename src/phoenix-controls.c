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

//JVila: This should actually be on a interrput subroutine..
void receiver_read(receiver_inputs_t * receiver){

}

//TO FINISH.
void receiver_scale(receiver_inputs_t * receiver){

  receiver->roll_scaled = receive
}


void receiver_read(gyro_t * gyro) {
    uart_puts("P: ");
    uart_putd(gyro->pitch);
    uart_puts(" --- ");

    uart_puts(" R: ");
    uart_putd(gyro->roll);
    uart_puts(" --- ");

    uart_puts(" Y: ");
    uart_putd(gyro->yaw);
    uart_puts(" ---\r\n");
}




//Initialize gyroscope
uint8_t gyro_init(gyro_t * gyro) {
    uart_puts("Initializing Gyroscope\r\n");
    uint8_t ret = 0;

    //Allocate memory for the gyro data structure if need be
    if(gyro == NULL) {
        return ERROR;
    }

    //Begin TWI communication
    ret = i2c_start(GYRO_ADDR + I2C_WRITE);
    if(ret) {
        i2c_stop();
        uart_puts("<-- ERROR: Unable to start gyroscope communication-->\r\n");
        return ERROR;
    }

    //Set the pointer to the control 1 register
    ret = i2c_write(GYRO_CTRL_1_REG);
    if(ret) {
        i2c_stop();
        uart_puts("<-- ERROR: Could not set pointer to gyroscope CTRL 1 register -->\r\n");
        return ERROR;
    }

    // Turn on the gyroscope and enable all axes (0x0F)
    ret = i2c_write(GYRO_PD | GYRO_ZEN | GYRO_YEN | GYRO_XEN);
    if(ret) {
        i2c_stop();
        uart_puts("<-- ERROR: Could not write to gyroscope -->\r\n");
        return ERROR;
    }

    //Start communication again
    ret = i2c_rep_start(GYRO_ADDR + I2C_WRITE);
    if(ret) {
        i2c_stop();
        uart_puts("<-- ERROR: Unable to start gyroscope communication-->\r\n");
        return ERROR;
    }

    //Set the pointer to the control 4 register
    ret = i2c_write(GYRO_CTRL_4_REG);
    if(ret) {
        i2c_stop();
        uart_puts("<-- ERROR: Could not set pointer to gyroscope CTRL 1 register -->\r\n");
        return ERROR;
    }

    //Block data update: output registers not updated until read and set full scale
    // selection to 500 dps (FS1:FS0 as 01) (0x90)
    ret = i2c_write(GYRO_BDU | GYRO_FS0);
    if(ret) {
        i2c_stop();
        uart_puts("<-- ERROR: Could not write to gyroscope -->\r\n");
        return ERROR;
    }

    //Release bus
    i2c_stop();

    //Give the gyroscope a bit of time (250ms) to start
    delay_us(250000);

    return OK;
}

//Calibrate gyroscope
uint8_t gyro_calibrate(gyro_t * gyro) {
    uart_puts("Starting gyroscope calibration tes\r\n");

    if(gyro == NULL) {
        uart_puts("NULL\r\n");
        return ERROR;
    }

    //Sum up 2000 measurements
    for (int i = 0; i < GYRO_CALIBRATION_STEPS; i++ ){
        gyro_read(gyro);
        gyro->roll_offset += gyro->roll;
        gyro->pitch_offset += gyro->pitch;
        gyro->yaw_offset += gyro->yaw;

        //Print a '.' every 100 readings
        if(!(i % 100)) {
            uart_puts(".");
        }

        //Delay 4ms
        delay_us(4000);
    }

    return OK;
}

//Read pitch, roll, and yaw from gyroscope and
// apply calibration offsets
uint8_t gyro_read(gyro_t * gyro) {
    char msb = 0;
    char lsb = 0;

    if(gyro == NULL) {
        return ERROR;
    }

    //Point to the read register
    i2c_start(GYRO_ADDR + I2C_WRITE);           //Begin TWI communication
    i2c_write(GYRO_READ_REG | GYRO_AUTO_INC);   //Point to read register (0x28) and enable auto increment

    //Read all six bytes
    i2c_rep_start(GYRO_ADDR + I2C_READ);

    //Roll
    lsb = i2c_readAck();
    msb = i2c_readAck();
    gyro->roll = ROLL_COEFFICIENT * ((msb << 8) | lsb) - gyro->roll_offset;

    //Pitch
    lsb = i2c_readAck();
    msb = i2c_readAck();
    gyro->pitch = PITCH_COEFFICIENT * ((msb << 8) | lsb) - gyro->pitch_offset;

    //Yaw
    lsb = i2c_readAck();
    msb = i2c_readNak();
    gyro->yaw = YAW_COEFFICIENT * ((msb << 8) | lsb) - gyro->yaw_offset;

    i2c_stop();

    return OK;
}

//Print the gyro data
void gyro_print(gyro_t * gyro) {
    uart_puts("P: ");
    uart_putd(gyro->pitch);
    uart_puts(" --- ");

    uart_puts(" R: ");
    uart_putd(gyro->roll);
    uart_puts(" --- ");

    uart_puts(" Y: ");
    uart_putd(gyro->yaw);
    uart_puts(" ---\r\n");
}

//Gyro test loop
void gyro_loop(gyro_t * gyro) {
    //Wait 250ms
    delay_us(250000);

    //Take a reading
    gyro_read(gyro);

    //Print gyro data
    gyro_print(gyro);
}
