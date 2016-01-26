#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro

//Declaring variables
int cal_int;
unsigned long UL_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;

//Setup routine
void setup(){
  Wire.begin();                                      //Start the I2C as master
  Serial.begin(9600);                                //Start the serial connetion @ 9600bps

  //The gyro is disabled by default and needs to be started
  Wire.beginTransmission(105);                       //Start communication with the gyro (adress 1101001)
  Wire.write(0x20);                                  //We want to write to register 20
  Wire.write(0x0F);                                  //Set the register bits as 00001111 (Turn on the gyro and enable all axis)
  Wire.endTransmission();                            //End the transmission with the gyro
  Wire.beginTransmission(105);                       //Start communication with the gyro (adress 1101001)
  Wire.write(0x23);                                  //We want to write to register 23
  Wire.write(0x80);                                  //Set the register bits as 10000000 (Block Data Update active)
  Wire.endTransmission();                            //End the transmission with the gyro


  delay(250);                                        //Give the gyro time to start
  
  //Let's take multiple samples so we can determine the average gyro offset
  Serial.print("Starting calibration...");           //Print message
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){    //Take 2000 readings for calibration
    gyro_signalen();                                 //Read the gyro output
    gyro_roll_cal += gyro_roll;                      //Ad roll value to gyro_roll_cal
    gyro_pitch_cal += gyro_pitch;                    //Ad pitch value to gyro_pitch_cal
    gyro_yaw_cal += gyro_yaw;                        //Ad yaw value to gyro_yaw_cal
    if(cal_int%100 == 0)Serial.print(".");           //Print a dot every 100 readings
    delay(4);                                        //Wait 4 milliseconds before the next loop
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset
  Serial.println(" done!");                          //2000 measures are done!
  gyro_roll_cal /= 2000;                             //Divide the roll total by 2000
  gyro_pitch_cal /= 2000;                            //Divide the pitch total by 2000
  gyro_yaw_cal /= 2000;                              //Divide the yaw total by 2000
  
}
//Main program
void loop(){
  delay(250);                                        //Wait 250 microseconds for every loop  
  gyro_signalen();                                   //Read the gyro signals
  print_output();                                    //Print the output
}

void gyro_signalen(){
  Wire.beginTransmission(105);                       //Start communication with the gyro (adress 1101001)
  Wire.write(168);                                   //Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                            //End the transmission
  Wire.requestFrom(105, 6);                          //Request 6 bytes from the gyro
  while(Wire.available() < 6);                       //Wait until the 6 bytes are received
  lowByte = Wire.read();                             //First received byte is the low part of the angular data
  highByte = Wire.read();                            //Second received byte is the high part of the angular data
  gyro_roll = ((highByte<<8)|lowByte);               //Multiply highByte by 256 and ad lowByte
  if(cal_int == 2000)gyro_roll -= gyro_roll_cal;     //Only compensate after the calibration
  lowByte = Wire.read();                             //First received byte is the low part of the angular data
  highByte = Wire.read();                            //Second received byte is the high part of the angular data
  gyro_pitch = ((highByte<<8)|lowByte);              //Multiply highByte by 256 and ad lowByte
  gyro_pitch *= -1;                                  //Invert axis
  if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;   //Only compensate after the calibration
  lowByte = Wire.read();                             //First received byte is the low part of the angular data
  highByte = Wire.read();                            //Second received byte is the high part of the angular data
  gyro_yaw = ((highByte<<8)|lowByte);                //Multiply highByte by 256 and ad lowByte
  gyro_yaw *= -1;                                    //Invert axis
  if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;       //Only compensate after the calibration
}

void print_output(){
  Serial.print("Pitch:");
  if(gyro_pitch >= 0)Serial.print("+");
  Serial.print(gyro_pitch/57.14286,0);               //Convert to degree per second
  if(gyro_pitch/57.14286 - 2 > 0)Serial.print(" NoU");
  else if(gyro_pitch/57.14286 + 2 < 0)Serial.print(" NoD");
  else Serial.print(" ---");
  Serial.print("  Roll:");
  if(gyro_roll >= 0)Serial.print("+");
  Serial.print(gyro_roll/57.14286,0);                //Convert to degree per second
  if(gyro_roll/57.14286 - 2 > 0)Serial.print(" RwD");
  else if(gyro_roll/57.14286 + 2 < 0)Serial.print(" RwU");
  else Serial.print(" ---");
  Serial.print("  Yaw:");
  if(gyro_yaw >= 0)Serial.print("+");
  Serial.print(gyro_yaw/57.14286,0);                 //Convert to degree per second
  if(gyro_yaw/57.14286 - 2 > 0)Serial.println(" NoR");
  else if(gyro_yaw/57.14286 + 2 < 0)Serial.println(" NoL");
  else Serial.println(" ---");
}
