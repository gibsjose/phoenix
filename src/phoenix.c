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
volatile int fDebug_receiver = 1 ;
volatile int fDebug_escs = 0 ;
volatile int fDebug_battery = 1;
volatile int fDebug_gyro = 1;
volatile int fDebug_pid_settings = 1;

/**************************************************
* Variables used for the receiver inputs *
**************************************************/
volatile int last_channel_1, last_channel_2,last_channel_3, last_channel_4;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
volatile int current_time1, current_time0;
volatile unsigned int timer_1, timer_2, timer_3, timer_4;
volatile int pin14Counter, pin15Counter, pin50Counter, pin52Counter;
int debug_pin14Counter, debug_pin15Counter, debug_pin50Counter, debug_pin52Counter;
volatile bool receiver_gas_received = false;
volatile bool receiver_roll_received = false;
volatile bool receiver_pitch_received = false;
volatile bool receiver_yaw_received = false;



int main(void) {
	//malloc data structures
	LED_RED_ON();
	int masterLoopIndex = 0 ;
	gyro_t *gyro = (gyro_t *)malloc(sizeof(gyro_t));
	memset(gyro, 0, sizeof(gyro_t));
	receiver_inputs_t *receiver = (receiver_inputs_t*)malloc(sizeof(receiver_inputs_t));
	receiver_memset(receiver);
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
	PWM_resetRegisters();

	//Enable interrupts, default value of SREG is 0

	init();

	//init_receiver_pins();
	init_analog_input_pins();

	//Initialize UART at 9600 baud
	uart_init(UART_BAUD_SELECT(BAUD, F_CPU));

	//Initialization
	uart_puts("Initializing ...\r\n");
	//DDRB |= PIN_12; //Configure Red LED as output
	uart_puts("Initializing ESC & Receiver registers \r\n");
	delay_us(250000);

	if(fDebug_gyro == 1){
		//Initialize gyroscope
		gyro_init(gyro);

		//Calibrate gyroscope. BE careful
		gyro_calibrate(gyro);
	}

	//Initialize PID settings for roll, pitch, yaw
	uart_puts("Initializing PID Settings \r\n");

	init_pid_settings(pid_roll, pid_pitch, pid_yaw);
  if(fDebug_pid_settings == 1){
    print_pid_settings(pid_roll->settings,pid_pitch->settings,pid_yaw->settings,);
  }

	init_receiver_registers();
	init_esc_registers();
	// Read initial batt voltage //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
	//Main Loop

		LED_RED_ON();
		LED_GREEN_ON();
	while(true) {
		// @TODO set start status
		//Read and print the gyro data
		if (fDebug_gyro == 1){
			gyro_loop(gyro);
		}
		if(fDebug_receiver == 1){
			if(receiver_gas_received && receiver_roll_received && receiver_pitch_received && receiver_yaw_received){

			receiver->gas = 4*receiver_input_channel_3;
			receiver->roll = 4*receiver_input_channel_1;
			receiver->yaw = 4*receiver_input_channel_4;
			receiver->pitch = 4*receiver_input_channel_2;
			receiver_scale(receiver);
	  //	receiver_print(receiver);
		//	calculate_setpoints(receiver,setpoints);
		//	setpoints_print(setpoints);
			debug_calculate_esc_pulses_duration(receiver,esc);
			commandPWMSignals(esc);

///////****************************************//////////////
//***** Used to check that we do not miss PWM inputs *****//
/*			debug_pin14Counter =  pin14Counter;
			debug_pin15Counter = pin15Counter;
			debug_pin50Counter = pin50Counter;
			debug_pin52Counter = pin52Counter;
			uart_puts("\r\n Pin counters : 14,15,50,52\r\n");
			uart_putd(debug_pin14Counter);
			uart_putd(debug_pin15Counter);
			uart_putd(debug_pin50Counter);
			uart_putd(debug_pin52Counter);*/
///////****************************************//////////////
			//delay_us(1000000);
			receiver_gas_received = false;
			receiver_roll_received = false;
			receiver_pitch_received = false;
			receiver_yaw_received = false;

		}
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
		//	calculate_esc_pulses_to_stop_motors(esc);
		}

		masterLoopIndex ++ ;
	}
	return 0;
}

////////////////////////////////////////////////////////////////////
//This routine is called every time input 50, 52 change state
////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect){
	//uart_puts("Entering PCINT0_vect");
	current_time0 = TCNT5L | (((int)(TCNT5H))<<8);
	//Arduino Input 50, Channel 3 GAS =========================================
	if(PINB & (1<<3)){                                        //Is input 50 high?
		if(last_channel_3 == 0){                                   //Input 50 changed from 0 to 1
			last_channel_3 = 1;                                      //Remember current input state
			timer_3 = current_time0;                                  //Set timer_3 to current_time0
		}
	}
	else if(last_channel_3 == 1){                                //Input 50 is not high and changed from 1 to 0
		last_channel_3 = 0;                                        //Remember current input state
		receiver_input_channel_3= (current_time0 - timer_3);     //Channel 3 is current_time0 - timer_3
		if(	receiver_input_channel_3<0){receiver_input_channel_3 = receiver_input_channel_3 + 65535; }
		pin50Counter++;
		receiver_gas_received = true;
		/* Serial.print(pin50Counter);
		Serial.print(". Pin 50 = ");
		Serial.println(receiver_input_channel_3);*/
	//	check_value(receiver_input_channel_3);
	}
	//Arduino Input 52, Channel 1 ROLL =========================================
	if(PINB & (1<<1)){                                       //Is input 52 high?
		if(last_channel_1 == 0){                                   //Input 52 changed from 0 to 1
			last_channel_1 = 1;                                      //Remember current input state
			timer_1 = current_time0;                                  //Set timer_1 to current_time0
		}
	}
	else if(last_channel_1 == 1){                                //Input 14 is not high and changed from 1 to 0
		last_channel_1 = 0;                                        //Remember current input state
		receiver_input_channel_1 = (current_time0 - timer_1);    //Channel 1 is current_time0 - timer_1
		if(	receiver_input_channel_1<0){receiver_input_channel_1 = receiver_input_channel_1 + 65535; }
		pin52Counter++;
		receiver_roll_received = true;

		/* Serial.print(pin52Counter);
		Serial.print(". Pin 52 = ");
		Serial.println(receiver_input_channel_1);*/
	//	check_value(receiver_input_channel_1);
	}
}

////////////////////////////////////////////////////////////////////
//This routine is called every time input 14, 15 change state
////////////////////////////////////////////////////////////////////
ISR(PCINT1_vect){
	//uart_puts("Entering PCINT1_vect");
	current_time1 = TCNT5L | (((int)(TCNT5H))<<8);
	//Arduino Input 15, Channel 4 YAW =========================================
	if(PINJ & (1<<0)){                                        //Is input 15 high?
		if(last_channel_4 == 0){                                   //Input 15 changed from 0 to 1
			last_channel_4 = 1;                                      //Remember current input state
			timer_4 = current_time1;                                  //Set timer_1 to current_time1
		}
	}
	else if(last_channel_4 == 1){                                //Input 15 is not high and changed from 1 to 0
		last_channel_4 = 0;                                        //Remember current input state
		receiver_input_channel_4= (current_time1 - timer_4);      //Channel 4 is current_time1 - timer_1
		if(	receiver_input_channel_4<0){receiver_input_channel_4 = receiver_input_channel_4 + 65535; }
		pin15Counter++;
		receiver_yaw_received = true;
		/*Serial.print(pin15Counter);
		Serial.print(". Pin 15 = ");
		Serial.println(receiver_input_channel_4);*/
	//	check_value(receiver_input_channel_4);
	}
	//Arduino Input 14, Channel 2 PITCH =========================================
	if(PINJ & (1<<1) ){                                       //Is input 14 high?
		if(last_channel_2 == 0){                                   //Input 14 changed from 0 to 1
			last_channel_2 = 1;                                      //Remember current input state
			timer_2 = current_time1;                                  //Set timer_2 to current_time1
		}
	}
	else if(last_channel_2 == 1){                                //Input 14 is not high and changed from 1 to 0
		last_channel_2 = 0;                                        //Remember current input state
		receiver_input_channel_2 = (current_time1 - timer_2);     //Channel 2 is current_time1 - timer_2
		if(	receiver_input_channel_2<0){receiver_input_channel_2 = receiver_input_channel_2 + 65535; }
		pin14Counter++;
		receiver_pitch_received = true;
		/*Serial.print(pin14Counter);
		Serial.print(". Pin 14 = ");
		Serial.println(receiver_input_channel_2);*/
		//check_value(receiver_input_channel_2);
	}
}

void check_value(int value){
	if((value > 1900) || (value < 1100)){
		//uart_putd(value);
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
	//timer1_init();

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
