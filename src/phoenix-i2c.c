/**********************************************************
*   Phoenix FSW - I2C
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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "phoenix-i2c.h"

//Initialize I2C peripheral
void rfcx_i2c_init(void) {
    i2c_init();
}

//Initialize temperature sensor
int rfcx_temp_init() {
    unsigned char value, value2;
    value = 0x01;
    value2 = 0x04;

    int ret = 0;

    char str[512];
    memset(str, 0, 512);

    unsigned char address;

    address = TEMP_ADDR;

    //sprintf(str, "Device address: 0x%02X\r\n", address);
    //uart_puts(str);

    //Begin TWI communication
    ret = i2c_start(address + I2C_WRITE);
    if(ret) {
        i2c_stop();
        uart_puts("<-- ERROR: Unable to start communication-->\r\n");
        return ERROR;
    }

    //Set the pointer to the configuration register
    ret = i2c_write(value);
    if(ret) {
        i2c_stop();
        uart_puts("<-- ERROR: Could not set pointer to LM75 config register -->\r\n");
        return ERROR;
    }

    //Enable the temp sensor
    ret = i2c_write(value2);
    if(ret) {
        i2c_stop();
        uart_puts("<-- ERROR: Could not enable LM75 -->\r\n");
        return ERROR;
    }

    //Release bus
    i2c_stop();

    return OK;
}

//Initialize ADC
int rfcx_adc_init() {
    unsigned char value, value2, value3;
    value = 0x01;
    value2 = 0x41;
    value3 = 0xEF;

    i2c_start_wait(ADC_ADDR);

    //Set the pointer to the configuration register
    i2c_write(value);

    //Put the ADC in single conversion mode
    i2c_write(value2);

    //Set the data rate to 3300, disable the comparator
    i2c_write(value3);

    return OK;
}

//Initialize humidity sensor
int rfcx_humid_init(void) {
    return OK;
}

//Shutdown (stop) I2C peripheral
//@TODO Does this function even make sense?
void rfcx_i2c_shutdown(void) {
    i2c_stop();
}

void rfcx_temp_data_init(temp_data_t * data) {
    memset(&(data->raw), 0, sizeof(temp_raw_t));
    data->temperature = 0.0;
}

void rfcx_humid_data_init(humid_data_t * data) {
    memset(&(data->raw), 0, sizeof(humid_raw_t));
    data->humidity = 0.0;
    data->temperature = 0.0;
    data->status = HUMID_STATUS_NORMAL;
}

void rfcx_adc_data_init(adc_data_t * data) {
    memset(&(data->raw), 0, sizeof(adc_raw_t));
    data->input_voltage = 0.0;
    data->input_current = 0.0;
    data->output_voltage = 0.0;
    data->output_current = 0.0;
}

//Shutdown temperature sensor:
// Set Temp Sensor control address to
// active high comparator mode, shutdown mode
void rfcx_temp_shutdown() {
    unsigned char value, value2;
    value = 0x01;
    value2 = 0x05;

    i2c_start_wait(TEMP_ADDR);

    //Set the pointer to the configuration register
    i2c_write(value);

    //Put the temp sensor in shutdown mode
    i2c_write(value2);
    i2c_stop();
}

void rfcx_adc_shutdown(void) {
    return;
}

void rfcx_humid_shutdown(void) {
    return;
}

//Read Temperature data from the LM75B
int rfcx_read_temp(temp_data_t * data) {
    int ret = 0;

    //Write the pointer register in the sensor to point to the temp register
    i2c_start_wait(TEMP_ADDR + I2C_WRITE);
    ret = i2c_write(0x00);
    if(ret) {
        i2c_stop();
        uart_puts("<-- ERROR: Could not set pointer register to temp (0x00)-->\r\n");
        return ERROR;
    }

    //Read both bytes for the temperature (msb first, then lsb)
    ret = i2c_rep_start(TEMP_ADDR + I2C_READ);
    if(ret) {
        i2c_stop();
        uart_puts("<-- ERROR: Could not repeat start temp sensor-->\r\n");
        return ERROR;
    }

    data->raw.msb = i2c_readAck();
    data->raw.lsb = i2c_readNak();
    i2c_stop();

    //Convert data
    convert_temp_data(data);

    return OK;
}

int rfcx_read_adc(adc_data_t * data) {
    //Perform all reads
    rfcx_read_adc_pin(data, ADC_INPUT_VOLTAGE_PIN);
    rfcx_read_adc_pin(data, ADC_OUTPUT_VOLTAGE_PIN);
    rfcx_read_adc_pin(data, ADC_INPUT_CURRENT_PIN);
    rfcx_read_adc_pin(data, ADC_OUTPUT_CURRENT_PIN);

    //Convert data
    convert_adc_data(data);

    return OK;
}

//This example might help: https://github.com/adafruit/Adafruit_ADS1X15/blob/master/Adafruit_ADS1015.cpp
int rfcx_read_adc_pin(adc_data_t * data, int pin) {
    unsigned char value, value2, value3;
    value = 0x01;

    switch(pin) {
        case ADC_INPUT_VOLTAGE_PIN:     //Pin AIN0 - Input voltage
            value2 = 0xC1;
            break;
        case ADC_OUTPUT_VOLTAGE_PIN:    //Pin AIN1 - Output voltage
            value2 = 0xD1;
            break;
        case ADC_INPUT_CURRENT_PIN:     //Pin AIN2 - Input current
            value2 = 0xE1;
            break;
        case ADC_OUTPUT_CURRENT_PIN:    //Pin AIN3 - Output current
            value2 = 0xF1;
            break;
        default:                        //Pin AIN0 - Input voltage
            value2 = 0xC1;
            break;
    }

    //value3 = 0xEF;
    value3 = 0x83;

    i2c_start_wait(ADC_ADDR + I2C_WRITE);

    //Set the pointer to the configuration register
    i2c_write(value);

    //Put the ADC in single conversion mode, select pin
    i2c_write(value2);

    //Set the data rate to 3300, disable the comparator
    i2c_write(value3);
    //i2c_stop();

    value = 0x00;
    i2c_rep_start(ADC_ADDR + I2C_WRITE);
    //Set the pointer to the conversion register
    i2c_write(value);
    i2c_stop();

    unsigned char msb, lsb;

    //Read from the conversion register
    i2c_rep_start(ADC_ADDR + I2C_READ);
    msb = i2c_readAck();
    lsb = i2c_readNak();

    //Store correctly based on pin
    switch(pin) {
        case ADC_INPUT_VOLTAGE_PIN:
            uart_puts("Input Voltage: ");
            data->raw.input_voltage_msb = msb;
            data->raw.input_voltage_lsb = lsb;
            break;
        case ADC_OUTPUT_VOLTAGE_PIN:
            uart_puts("Output Voltage: ");
            data->raw.output_voltage_msb = msb;
            data->raw.output_voltage_lsb = lsb;
            break;
        case ADC_INPUT_CURRENT_PIN:
            uart_puts("Input Current: ");
            data->raw.input_current_msb = msb;
            data->raw.input_current_lsb = lsb;
            break;
        case ADC_OUTPUT_CURRENT_PIN:
            uart_puts("Output Current: ");
            data->raw.output_current_msb = msb;
            data->raw.output_current_lsb = lsb;
            break;
        default:
            data->raw.input_voltage_msb = msb;
            data->raw.input_voltage_lsb = lsb;
            break;
    }

    char str[32];
    sprintf(str, "msb: 0x%02X, lsb: 0x%02X\r\n", msb, lsb);
    uart_puts(str);

    return OK;
}

int rfcx_read_humid(humid_data_t * data) {
    int ret= 0;

    //Issue a 'Measurement Request' command
    i2c_start_wait(HUMID_ADDR + I2C_WRITE);
    i2c_stop();

    //Delay ~37ms (datasheet specifies 36.65ms)
    // NOTE:    This could be avoided if necessary, and we
    //          will just always get the data from the
    //          previous conversion.
    delay_us(HUMID_CONV_TIME);

    //Fetch humidity + temp data
    ret = i2c_rep_start(HUMID_ADDR + I2C_READ);
    if(ret) {
        i2c_stop();
        uart_puts("<-- ERROR: Could not repeat start humidity sensor-->\r\n");
        return ERROR;
    }

    data->raw.humid_msb = i2c_readAck();
    data->raw.humid_lsb = i2c_readAck();
    data->raw.temp_msb = i2c_readAck();
    data->raw.temp_lsb = i2c_readNak();
    i2c_stop();

    //Status bits are two msb's of humid_msb
    data->status = (data->raw.humid_msb & 0xC0) >> 6;

    //Perform conversion
    convert_humid_data(data);

    return OK;
}


void convert_temp_data(temp_data_t * data) {
    int tmp = 0;
    float result = 0.0;

    int msb = (int)data->raw.msb;
    int lsb = (int)data->raw.lsb;

    //Shift 'dem bits around
    tmp = ((msb << 8) | (lsb & ~0x1F)) >> 5;

    //Check sign of data
    if((msb & 0x80) == 0x80) {
        result = (float)(tmp) * 0.125;
    }
    else {
        result = -1.0 * (float)(~tmp + 1) * 0.125;
    }

    data->temperature = result;
}

void convert_adc_data(adc_data_t * data) {
    data->input_voltage = convert_adc_data_pin(data, ADC_INPUT_VOLTAGE_PIN);
    data->input_current = convert_adc_data_pin(data, ADC_INPUT_CURRENT_PIN);
    data->output_voltage = convert_adc_data_pin(data, ADC_OUTPUT_VOLTAGE_PIN);
    data->output_current = convert_adc_data_pin(data, ADC_OUTPUT_CURRENT_PIN);
}

//Perform conversion for individual pin
float convert_adc_data_pin(adc_data_t * data, int pin) {
    //2-byte conversion register
    int tmp = 0;
    int msb = 0;
    int lsb = 0;
    float voltage = 0.0;

    //Some C pointer witchcraft...
    msb = (int) *((unsigned char *)&(data->raw) + (pin * 2 * sizeof(unsigned char)));
    lsb = (int) *((unsigned char *)&(data->raw) + (pin * 2 * sizeof(unsigned char)) + 1);

    char str[32];
    sprintf(str, "msb: 0x%02X, lsb: 0x%02X\r\n", msb, lsb);
    uart_puts(str);

    //Shift 'em
    tmp = ((msb << 8) | lsb) >> 4;

    sprintf(str, "tmp: 0x%02X\r\n", tmp);
    uart_puts(str);

    //Scale by resolution
    voltage = ((float)tmp / ADC_RESOLUTION) * ADC_VOLTAGE_MAX;

    char tmp_str[16];
    char message[32];

    dtostrf((double)voltage, 5, 2, tmp_str);
    sprintf(message, "Raw Voltage (%d):  %sV\r\n", (pin + 4), tmp_str);
    uart_puts(message);

    switch(pin) {
        case ADC_INPUT_VOLTAGE_PIN:
            voltage *= ADC_VOLTAGE_SCALE;
            break;
        case ADC_OUTPUT_VOLTAGE_PIN:
            voltage *= ADC_VOLTAGE_SCALE;
            break;
        case ADC_INPUT_CURRENT_PIN:
            voltage *= ADC_CURRENT_SCALE;
            break;
        case ADC_OUTPUT_CURRENT_PIN:
            voltage *= ADC_CURRENT_SCALE;
            break;
        default:
            break;
    }

    //Return voltage/current
    return voltage;
}

void convert_humid_data(humid_data_t * data) {
    uint16_t tmp_humid = 0;
    uint16_t tmp_temp = 0;

    uint16_t humid_msb = (uint16_t)data->raw.humid_msb;
    uint16_t humid_lsb = (uint16_t)data->raw.humid_lsb;
    uint16_t temp_msb = (uint16_t)data->raw.temp_msb;
    uint16_t temp_lsb = (uint16_t)data->raw.temp_lsb;

    //Bit shifting
    tmp_humid = ((humid_msb & ~0xC0) << 8) | humid_lsb;
    tmp_temp = ((temp_msb << 8) | (temp_lsb & ~0x03)) >> 2;

    data->humidity = ((float)tmp_humid / HUMID_COUNTS) * 100.0;
    data->temperature = (((float)tmp_temp / TEMP_COUNTS) * 165.0) - 40.0;

    return;
}

void rfcx_humid_status_string(char * str, unsigned char status) {
    switch(status) {
        case HUMID_STATUS_NORMAL:
            sprintf(str, "Normal");
            break;
        case HUMID_STATUS_STALE:
            sprintf(str, "STALE DATA");
            break;
        case HUMID_STATUS_COMMAND:
            sprintf(str, "Command Mode");
            break;
        case HUMID_STATUS_DIAG:
            sprintf(str, "Diagnostic Condition");
            break;
        default:
            sprintf(str, "UNKNOWN");
            break;
    }
}
