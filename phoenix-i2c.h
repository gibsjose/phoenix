/**********************************************************
*   Phoenix FSW - I2C
*
*   25 Jan 2016
*
*   Joe Gibson 						(joseph.gibson@nasa.gov)
*	Jordi Vila Hernandez de Lorenzo	(jordi.vila.hdl@gmail.com)
*
*   License: MIT (http://user.mit-license.org)
*
*   https://github.com/gibsjose/phoenix
**********************************************************/

#ifndef PHOENIX_I2C_H
#define PHOENIX_I2C_H

#include <avr/io.h>
#include <string.h>
#include "i2cmaster/i2cmaster.h"

#include "phoenix-globals.h"
#include "utilities/delay.h"
#include "utilities/usart.h"

//Simple Error Representation
#define OK      0
#define ERROR   1

//I2C Addresses
#define TEMP_ADDR   0x90    //1001CBA0, A = B = C = 0 (pulled up/down in hardware)
#define ADC_ADDR    0x92    //1001XX1X, ADDR connected to VCC
#define HUMID_ADDR  0x4E    //01001110, Pre-defined address (0x27 << 1)

//ADC Pins
#define ADC_INPUT_VOLTAGE_PIN   0x00
#define ADC_OUTPUT_VOLTAGE_PIN  0x01
#define ADC_INPUT_CURRENT_PIN   0x02
#define ADC_OUTPUT_CURRENT_PIN  0x03

//ADC Conversions
#define ADC_VOLTAGE_SCALE       (1.0 / (2100.0 / (1540.0 + 2100.0)))    //1.54k + 2.1k resistor divider = ~1.733333333 scale
#define ADC_VOLTAGE_MAX         3.3                             //ADC has range of 0-3.3V (Vcc)
#define ADC_RESOLUTION          (0x01 << 12)                    //12 bit resolution
#define ADC_OPAMP_GAIN          (1.0 + (2550.0 / 200000.0))     //Vout = (1 + Rf/Ri)*Vin, Vin = (V+ - V-), Rf = 2.55k, Ri = 200k
#define ADC_CUR_SENSE_RES       (0.51)                          //Current sense resistor = 0.51Ohms
#define ADC_CURRENT_SCALE       (ADC_OPAMP_GAIN / ADC_CUR_SENSE_RES)    //Opamp gain, then divide by sense resistor

//Humidity Sensor Status
#define HUMID_STATUS_NORMAL     0x00    //Normal operation
#define HUMID_STATUS_STALE      0x01    //Stale data
#define HUMID_STATUS_COMMAND    0x02    //Command mode
#define HUMID_STATUS_DIAG       0x03    //Diagnostic condition

//Humidity Sensor Conversion
#define HUMID_COUNTS            0x3FFF  //2^14 - 1 = 16383
#define TEMP_COUNTS             0x3FFF
#define HUMID_CONV_TIME         0x9088  //37000us -> 37ms (HIH6130 datasheet specifies 36.65ms conversion time)

//Data structure for LM75BD temp sensor
typedef struct temp_raw_t {
    unsigned char msb;
    unsigned char lsb;
}temp_raw_t;

typedef struct temp_data_t {
    temp_raw_t raw;         //Raw bytes
    float temperature;      //Temperature
}temp_data_t;

//Data structure for ADS1015 external ADC
typedef struct adc_raw_t {
    unsigned char input_voltage_msb;
    unsigned char input_voltage_lsb;
    unsigned char output_voltage_msb;
    unsigned char output_voltage_lsb;
    unsigned char input_current_msb;
    unsigned char input_current_lsb;
    unsigned char output_current_msb;
    unsigned char output_current_lsb;
}adc_raw_t;

typedef struct adc_data_t {
    adc_raw_t raw;          //Raw bytes
    float input_voltage;    //Input voltage (V)
    float output_voltage;   //Output voltage (V)
    float input_current;    //Input current (A)
    float output_current;   //Output current (A)
}adc_data_t;

//Data structures for HIH6130 humidity + temp sensor
typedef struct humid_raw_t {
    unsigned char humid_msb;
    unsigned char humid_lsb;
    unsigned char temp_msb;
    unsigned char temp_lsb;
}humid_raw_t;

typedef struct humid_data_t {
    humid_raw_t raw;        //Raw bytes
    float humidity;         //Relative humidity
    float temperature;      //Temperature
    unsigned char status;   //Status
}humid_data_t;

//Initialization
void rfcx_i2c_init(void);
int rfcx_temp_init(void);
int rfcx_adc_init(void);
int rfcx_humid_init(void);

//Data Initialization
void rfcx_temp_data_init(temp_data_t *);
void rfcx_humid_data_init(humid_data_t *);
void rfcx_adc_data_init(adc_data_t *);

//Shutdown
void rfcx_i2c_shutdown(void);
void rfcx_temp_shutdown(void);
void rfcx_adc_shutdown(void);
void rfcx_humid_shutdown(void);

//Read
int rfcx_read_temp(temp_data_t *);
int rfcx_read_adc_pin(adc_data_t *, int);
int rfcx_read_adc(adc_data_t *);
int rfcx_read_humid(humid_data_t *);

//Static Conversion Helpers
void convert_temp_data(temp_data_t *);
void convert_adc_data(adc_data_t *);
float convert_adc_data_pin(adc_data_t *, int);
void convert_humid_data(humid_data_t *);

//String Conversion Helpers
void rfcx_humid_status_string(char *, unsigned char);

#endif//PHOENIX_I2C_H
