/**********************************************************
*   Phoenix FSW - Main
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

#include "phoenix.h"

//Don't forget `volatile`!
volatile receiver_inputs_t *receiver;
volatile int current_time = 0 ; //Used for the interrupt subroutine of the Receiver Inputs
volatile int last_channel_1 = 0;
volatile int last_channel_2 = 0;
volatile int last_channel_3 = 0;
volatile int last_channel_4 = 0;
volatile int timer_1, timer_2, timer_3, timer_4;

volatile int fDebug_receiver = 0 ;
volatile int fDebug_escs = 1 ;
volatile int fDebug_battery = 0;
volatile int fDebug_gyro = 1;

//Timer 1 Compare Interrupt Vector (1s CTC Timer)
ISR(TIMER1_COMPA_vect) {
	//Blink LED
	//PORTB ^= _BV(LED_PIN);

	//Set flag to indicate that we should read the receivers
}

int main(void) {
	//malloc data structures
	gyro_t *gyro = (gyro_t *)malloc(sizeof(gyro_t));
	memset(gyro, 0, sizeof(gyro_t));

	//receiver_inputs_t *receiver = (receiver_inputs_t*)malloc(sizeof(receiver_inputs_t));
	setpoints_t *setpoints = (setpoints_t*)malloc(sizeof(setpoints_t));
	memset(setpoints, 0, sizeof(setpoints_t));

	PID_roll_t *pid_roll = (PID_roll_t *)malloc(sizeof(PID_roll_t));
	memset(pid_roll, 0, sizeof(PID_roll_t));
	PID_pitch_t *pid_pitch = (PID_pitch_t *)malloc(sizeof(PID_pitch_t));
	memset(pid_pitch, 0, sizeof(PID_pitch_t));
	PID_yaw_t *pid_yaw = (PID_yaw_t *)malloc(sizeof(PID_yaw_t));
	memset(pid_yaw, 0, sizeof(PID_yaw_t));
	ESC_outputs_t *esc= (ESC_outputs_t*)malloc(sizeof(ESC_outputs_t));
	memset(esc, 0, sizeof(ESC_outputs_t));

	int sign = 1;
	int ret = 0;
	int start = 0;
	double battery_voltage;

	//Initialize UART at 9600 baud
	uart_init(UART_BAUD_SELECT(BAUD, F_CPU));

	//Initialization
	uart_puts("Initializing ...\r\n");

	ret = init();
	if(ret) {
		uart_puts("<-- ERROR: Initialization failed -->\r\n");
	} else {
		uart_puts("Initialization successful\r\n");
	}

	if(fDebug_gyro == 1){
		//Initialize gyroscope
		gyro_init(gyro);

		//Calibrate gyroscope. BE careful
		gyro_calibrate(gyro);
	}

	//Initialize PID settings for roll, pitch, yaw
	uart_puts("Initializing PID Settings \r\n");

	init_pid_settings(pid_roll, pid_pitch, pid_yaw);

	//Initialize esc pins as outputs and timer registers
	uart_puts("Initializing ESC pins and configuring timing registers \r\n");
	init_esc_pins();
	//init_receiver_pins();
	init_analog_input_pins();

	// Read initial batt voltage //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
	//Main Loop
	while(true) {
		// @TODO set start status

		//Read and print the gyro data
		if (fDebug_gyro == 1){
			gyro_loop(gyro);
		}

		//Calculate the PID output to feed into the ESCs
		//		calculate_pids(gyro, setpoints, pid_roll, pid_pitch, pid_yaw);
		// //1260 / 1023 = 1.2317.
		battery_voltage = readBatteryVoltage();
		if(fDebug_battery == 1){
			uart_puts("Battery Voltage = ");
			uart_putd(battery_voltage);
			uart_puts("\r\n");
			delay_us(100);
		}
		//	battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

		if (start == 2){ //The motors are started
			//Read battery voltage()
			uart_puts("Calculating ESC pulses duration \r\n");
			calculate_esc_pulses_duration(receiver, pid_roll, pid_pitch, pid_yaw, esc);
		}

		else{
			//uart_puts("Calculating ESC pulses duration to stop motors \r\n");
			calculate_esc_pulses_to_stop_motors(esc);
		}

		//uart_puts("Commanding PWM signals \r\n");
		//commandPWMSignals(esc);
		if(fDebug_escs == 1){
			PWM_loop(&sign);
		}


	}

	return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT2_vect){
	//	current_time = micros();
	//Channel 1, ROLL =========================================
	if(PIND & (1 << 4) ){                                       //Is input 4 high?
		if(last_channel_1 == 0){                                   //Input 4 changed from 0 to 1
			last_channel_1 = 1;                                      //Remember current input state
			timer_1 = current_time;                                  //Set timer_1 to current_time
		}
	}
	else if(last_channel_1 == 1){                                //Input 4 is not high and changed from 1 to 0
		last_channel_1 = 0;                                        //Remember current input state
		receiver->roll = current_time - timer_1;                //Channel 1 is current_time - timer_1
	}
	//Channel 2, PITCH =========================================
	if(PIND & (1 << 3)  ){                                       //Is input 3 high?
		if(last_channel_2 == 0){                                   //Input 3 changed from 0 to 1
			last_channel_2 = 1;                                      //Remember current input state
			timer_2 = current_time;                                  //Set timer_2 to current_time
		}
	}
	else if(last_channel_2 == 1){                                //Input 6 is not high and changed from 1 to 0
		last_channel_2 = 0;                                        //Remember current input state
		receiver->pitch = current_time - timer_2;                //Channel 2 is current_time - timer_2

	}
	//Channel 3, GAS =========================================
	if(PIND & (1 << 2) ){                                        //Is input 2 high?
		if(last_channel_3 == 0){                                   //Input 2 changed from 0 to 1
			last_channel_3 = 1;                                      //Remember current input state
			timer_3 = current_time;                                  //Set timer_3 to current_time
		}
	}
	else if(last_channel_3 == 1){                                //Input 2 is not high and changed from 1 to 0
		last_channel_3 = 0;                                        //Remember current input state
		receiver->gas = current_time - timer_3;                //Channel 3 is current_time - timer_3
	}
	//Channel 4, YAW =========================================
	if(PIND & (1 << 7)  ){                                       //Is input 7 high?
		if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
			last_channel_4 = 1;                                      //Remember current input state
			timer_4 = current_time;                                  //Set timer_4 to current_time
		}
	}
	else if(last_channel_4 == 1){                                //Input 7 is not high and changed from 1 to 0
		last_channel_4 = 0;                                        //Remember current input state
		receiver->yaw = current_time - timer_4;                //Channel 4 is current_time - timer_4
	}

	receiver_scale(receiver);

	if(fDebug_receiver == 1){
		receiver_print(receiver);
	}
}


int init(void) {
	int ret = 0;

	//Clear interrupts
	cli();

	//Initialize ports
	port_init();

	//Initialize peripherals
	peripheral_init();

	//Initialize devices
	ret = device_init();

	//Enable interrupts
	sei();

	return ret;
}

int port_init(void) {
	//Initialize LED as output
	//DDRB |= _BV(LED_DD);

	//Initialize battery inputs/outputs
	// rfcx_batteries_init();

	return 0;
}

int timer1_init(void) {
	//Initialize Timer 1
	TCCR1A = 0;
	TCCR1B = 0;

	//Set CTC compare value (1 second)
	OCR1A = TIMER1_COUNT;

	//Enable CTC mode
	TCCR1B |= (1 << WGM12);

	//Enable 1024 prescaler
	TCCR1B |= (1 << CS10);
	TCCR1B |= (1 << CS12);

	//Enable Timer 1 output compare interrupt
	TIMSK1 |= (1 << OCIE1A);

	return 0;
}

int peripheral_init(void) {
	//Initialize Timer 1
	timer1_init();

	//Initialize I2C (TWI) peripheral as a whole
	i2c_init();

	return 0;
}

int device_init(void) {
	int ret = 0;

	//Initialize external I2C temp sensor (LM75BD)
	// ret = rfcx_temp_init();
	// if(ret) {
	// 	uart_puts("<-- ERROR: Error initializing temp sensor -->\r\n");
	// 	return ret;
	// } else {
	// 	uart_puts("Successfully initialized temp sensor\r\n");
	// }

	//Initialize external I2C ADC (ADS1015)
	// ret = rfcx_adc_init();
	// if(ret) {
	// 	 uart_puts("<-- ERROR: Error initializing ADC -->\r\n");
	// } else {
	// 	 uart_puts("Successfully initialized ADC\r\n");
	// }

	// //Initialize external I2C humidity sensor (HIH6130)
	// ret = rfcx_humid_init();
	// if(ret) {
	// 	uart_puts("<-- ERROR: Error initializing humidity sensor -->\r\n");
	// 	return ret;
	// } else {
	// 	uart_puts("Successfully initialized humidity sensor\r\n");
	// }

	return ret;
}
