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
volatile int fDebug_receiver = 0 ;
volatile int fDebug_receiver_booleans = 0;
volatile int fDebug_escs = 0 ;
volatile int fDebug_battery = 0;
volatile int fDebug_gyro = 0;
volatile int fDebug_pid_settings_input_output = 0;
volatile unsigned long fDebug_masterLoopIndex = 0;

volatile unsigned long triggerMasterLoopCounter = 0;
volatile bool triggerMasterLoop = false;

/**************************************************
* Variables used for the receiver inputs *
**************************************************/
volatile int last_channel_1, last_channel_2,last_channel_3, last_channel_4, last_channel_5, last_channel_6;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;
volatile int current_time1, current_time0;
volatile unsigned int timer_1, timer_2, timer_3, timer_4, timer_5, timer_6;
volatile int pin14Counter, pin15Counter, pin50Counter, pin52Counter, pin51Counter, pin53Counter;
int debug_receiver_pitch_Counter, debug_receiver_yaw_Counter, debug_receiver_gas_Counter, debug_receiver_roll_Counter, debug_receiver_channel5_Counter, debug_receiver_channel6_Counter;
volatile bool receiver_gas_received = false;
volatile bool receiver_roll_received = false;
volatile bool receiver_pitch_received = false;
volatile bool receiver_yaw_received = false;
volatile bool receiver_channel5_received = false;
volatile bool receiver_channel6_received = false;



int main(void) {
  init_LEDs_as_outputs();
  LED_RED_ON();
  init_master_Loop_timerRegisters();
  int idle_loop_counter = 0;
  unsigned long masterLoopIndexCheck = 0;
  unsigned long masterLoopIndex = 0 ;
  //malloc data structures
  gyro_t *gyro = (gyro_t *)malloc(sizeof(gyro_t));
  memset(gyro, 0, sizeof(gyro_t));
  receiver_inputs_t *receiver = (receiver_inputs_t*)malloc(sizeof(receiver_inputs_t));
  //Not sure why receiver_memset(receiver);
  memset(receiver, 0, sizeof(receiver_inputs_t));
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
  PWM_resetRegisters();

  int ret = 0;
  int MODE = CALIBRATE;
  double battery_voltage;


  //Enable interrupts, default value of SREG is 0

  init();

  //init_receiver_pins();
  init_analog_input_pins();

  //Initialize UART at 9600 baud
  uart_init(UART_BAUD_SELECT(BAUD, F_CPU));

  //Initialization
  uart_puts("Initializing ...\r\n");
  uart_puts("Initializing ESC & Receiver registers \r\n");
  delay_us(250000);

 //Disable Interrupts for gyro calibration
  //Initialize gyroscope
  gyro_init(gyro);

  //Calibrate gyroscope. BE careful
  gyro_calibrate(gyro);
  if(fDebug_gyro == 1){
    gyro_read_scaleOffset_filter(gyro);
    gyro_print(gyro);
  }
  //Initialize PID settings for roll, pitch, yaw
  uart_puts("Initializing PID Settings \r\n");

  //Initialize the gains for the PIDs
  init_pid_settings(pid_roll, pid_pitch, pid_yaw);
  print_pid_settings(&pid_roll->settings, &pid_pitch->settings, &pid_yaw->settings);
  init_receiver_registers(); //Resets PORTB!
  init_esc_registers();

  // Read initial batt voltage //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.

  MODE = IDLE;

  //////**********//////
  while (MODE == IDLE) {
    // Starting the motors: GAS low and YAW left.
    if(receiver_gas_received && receiver_roll_received && receiver_pitch_received && receiver_yaw_received && receiver_channel5_received){
      receiver->gas = 4*receiver_input_channel_3; //4 is the relation between the timer frequency and the pwm frequency
      receiver->roll = 4*receiver_input_channel_1;
      receiver->yaw = 4*receiver_input_channel_4;
      receiver->pitch = 4*receiver_input_channel_2;
      receiver->channel5 = 4*receiver_input_channel_5;
      receiver->channel6 = 4*receiver_input_channel_6;

      //Initialize PID settings for roll, pitch, yaw
      //uart_puts("Receiver received \r\n");
      receiver_scale(receiver);
      //receiver_print(receiver);
      gyro_read_scaleOffset_filter(gyro);
      //gyro_print(gyro);
      receiver_print(receiver);
      if( (receiver->gas_scaled <= 1020) && (receiver->gas_scaled >= 990) &&  (receiver->yaw_scaled <= 1020) && (receiver->yaw_scaled >= 990) ){
        MODE = FLY;
        reset_accoumulated_error_PID_input(pid_roll, pid_pitch, pid_yaw);
      }
      else{
        if((idle_loop_counter%3) == 0){
          LED_GREEN_CHANGE_STATUS(); // Waiting for correct input
        }

      }
      receiver_gas_received = false;
      receiver_roll_received = false;
      receiver_pitch_received = false;
      receiver_yaw_received = false;
      receiver_channel5_received = false;
      receiver_channel6_received = false;
      idle_loop_counter++;
    }

    if(fDebug_receiver_booleans){
      uart_puts(" \r\n\r\nReceiver booleans:");
      if(receiver_gas_received){uart_puts("\r\nreceiver_gas_received = true");}
      else if(receiver_roll_received){uart_puts("\r\nreceiver_roll_received = true");}
      else if(receiver_pitch_received){uart_puts("\r\nreceiver_pitch_received = true");}
      else if(receiver_yaw_received){uart_puts("\r\nreceiver_yaw_received = true");}
      else if(receiver_channel5_received){uart_puts("\r\nreceiver_channel5_received = true");}
      else if(receiver_channel6_received){uart_puts("\r\nreceiver_channel6_received = true");}
      else {uart_puts("\r\nno receiver channels received");}
    }
  }

  LED_GREEN_ON();

  //Main Loop
  while(true) {

    while(triggerMasterLoop == false){/*Wait*/}
      masterLoopIndex ++ ;
      if(fDebug_masterLoopIndex ==1){
        if((masterLoopIndex%100) == 0 ){
          masterLoopIndexCheck = triggerMasterLoopCounter;
          uart_puts("\n\rMasterLoopIndex = ");
          uart_putd(masterLoopIndex);
          uart_puts("\n\rMasterLoopIndexCheck = ");
          uart_putd(masterLoopIndexCheck);
        }
      }

      //Read the gyroscope
      gyro_read_scaleOffset_filter(gyro);
      //print the gyro data
      if (fDebug_gyro == 1){
        //Print gyro filtered data
        gyro_print(gyro);
      }


      if(receiver_gas_received && receiver_roll_received && receiver_pitch_received && receiver_yaw_received && receiver_channel5_received){
        receiver->gas = 4*receiver_input_channel_3; //4 is the relation between the timer frequency and the pwm frequency
        receiver->roll = 4*receiver_input_channel_1;
        receiver->yaw = 4*receiver_input_channel_4;
        receiver->pitch = 4*receiver_input_channel_2;
        receiver->channel5 = 4*receiver_input_channel_5;
        receiver->channel6 = 4*receiver_input_channel_6;

        receiver_scale(receiver);
        calculate_setpoints(receiver,setpoints);


        if(fDebug_receiver == 1){
          receiver_print(receiver);
          setpoints_print(setpoints);
          //debug_calculate_esc_pulses_duration(receiver,esc);
          //commandPWMSignals(esc);

          ///////****************************************//////////////
          //***** Used to check that we do not miss PWM inputs *****//
          debug_receiver_pitch_Counter =  pin14Counter;
          debug_receiver_yaw_Counter = pin15Counter;
          debug_receiver_gas_Counter = pin50Counter;
          debug_receiver_roll_Counter = pin52Counter;
          debug_receiver_channel5_Counter = pin53Counter;
          debug_receiver_channel6_Counter = pin51Counter;

          uart_puts("\r\n Pin counters : 14,15,50,52,53,51\r\n");
          uart_putd(debug_receiver_pitch_Counter);
          uart_putd(debug_receiver_yaw_Counter);
          uart_putd(debug_receiver_gas_Counter);
          uart_putd(debug_receiver_roll_Counter);
          uart_putd(debug_receiver_channel5_Counter);
          uart_putd(debug_receiver_channel6_Counter);
          ///////****************************************//////////////
        }
        receiver_gas_received = false;
        receiver_roll_received = false;
        receiver_pitch_received = false;
        receiver_yaw_received = false;
        receiver_channel5_received = false;
        receiver_channel6_received = false;

      }

      //Stopping the motors: GAS low and YAW right.
      if((MODE == FLY) && (receiver->gas_scaled <= 1020) && (receiver->gas_scaled >= 990) &&  (receiver->yaw_scaled >= 1950) && (receiver->yaw_scaled <= 2000) ){
        MODE = STOP_MOTORS;
        LED_GREEN_OFF();
      }
      // Starting the motors: GAS low and YAW left.
      if( (MODE == STOP_MOTORS) && (receiver->gas_scaled <= 1020) && (receiver->gas_scaled >= 990) &&  (receiver->yaw_scaled <= 1020) && (receiver->yaw_scaled >= 990) ){
        MODE = FLY;
        reset_accoumulated_error_PID_input(pid_roll, pid_pitch, pid_yaw);
        LED_GREEN_ON();
      }

      //A complementary filter is used to reduce noise.
      battery_voltage = battery_voltage*0.92+ readBatteryVoltage() * 0.08;
      if((battery_voltage < 12.2) && (battery_voltage > 6) ){LED_RED_ON();}
      if(fDebug_battery == 1){
        uart_puts("\r\n --- Battery Voltage ---\r\n");
        uart_putd(battery_voltage);
        uart_puts(" volts \r\n");
        delay_us(100);
      }
      //	battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

      if(receiver->channel5_scaled > 1500){
        if(fDebug_receiver == 1){
              uart_puts("\r\n --- Reseting PID errors ---\r\n");
        }
        reset_accoumulated_error_PID_input(pid_roll, pid_pitch, pid_yaw);
      }

      if (MODE == FLY){ //The motors are started
        //Calculate the PID output to feed into the ESCs
        calculate_pids(gyro, setpoints, pid_roll, pid_pitch, pid_yaw);
        //Calculate the length of the PWM signal to feed to each motor. receiver used for gas only
        calculate_esc_pulses_duration(receiver, pid_roll, pid_pitch, pid_yaw, esc);
        //Change the registers to spin each motor
        commandPWMSignals(esc);
        if(fDebug_pid_settings_input_output == 1){
          //print_pid_settings(&(pid_roll->settings),&(pid_pitch->settings),&(pid_yaw->settings));
        //  print_pid_outputs(&(pid_roll->output),&(pid_pitch->output),&(pid_yaw->output));
          print_pid_inputs(&(pid_roll->input),&(pid_pitch->input),&(pid_yaw->input));
        }
      }

      if(MODE == STOP_MOTORS){
        //uart_puts("Calculating ESC pulses duration to stop motors \r\n");
        calculate_esc_pulses_to_stop_motors(esc);
        commandPWMSignals(esc);
      }
      triggerMasterLoop = false;

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
  if(PINB & (1<<3)){                                           //Is input 50 high?
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

  }
  //Arduino Input 52, Channel 1 ROLL =========================================
  if(PINB & (1<<1)){                                          //Is input 52 high?
    if(last_channel_1 == 0){                                   //Input 52 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time0;                                  //Set timer_1 to current_time0
    }
  }
  else if(last_channel_1 == 1){                                //Input 52 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = (current_time0 - timer_1);    //Channel 1 is current_time0 - timer_1
    if(	receiver_input_channel_1<0){receiver_input_channel_1 = receiver_input_channel_1 + 65535; }
    pin52Counter++;
    receiver_roll_received = true;

  }

  //Arduino Input 53, Channel 5 =========================================
  if(PINB & (1<<0)){                                           //Is input 53 high?
    if(last_channel_5 == 0){                                   //Input 53 changed from 0 to 1
      last_channel_5 = 1;                                      //Remember current input state
      timer_5 = current_time0;                                  //Set timer_5 to current_time0
    }
  }
  else if(last_channel_5 == 1){                                //Input 53 is not high and changed from 1 to 0
    last_channel_5 = 0;                                        //Remember current input state
    receiver_input_channel_5 = (current_time0 - timer_5);    //Channel 5 is current_time0 - timer_5
    if(	receiver_input_channel_5<0){receiver_input_channel_5 = receiver_input_channel_5 + 65535; }
    pin53Counter++;
    receiver_channel5_received = true;
  }

  //Arduino Input 51, Channel 6 =========================================
  if(PINB & (1<<2)){                                           //Is input 51 high?
    if(last_channel_6 == 0){                                   //Input 51 changed from 0 to 1
      last_channel_6 = 1;                                      //Remember current input state
      timer_6 = current_time0;                                  //Set timer_6 to current_time0
    }
  }
  else if(last_channel_6 == 1){                                //Input 51 is not high and changed from 1 to 0
    last_channel_6 = 0;                                        //Remember current input state
    receiver_input_channel_6 = (current_time0 - timer_6);    //Channel 6 is current_time0 - timer_6
    if(	receiver_input_channel_6<0){receiver_input_channel_6 = receiver_input_channel_6 + 65535; }
    pin51Counter++;
    receiver_channel6_received = true;
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

  }
}


////////////////////////////////////////////////////////////////////
//This routine is called to trigger the Masterloop
////////////////////////////////////////////////////////////////////
ISR(TIMER3_COMPA_vect)  {
    triggerMasterLoop = true;
    triggerMasterLoopCounter++;
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
