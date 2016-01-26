#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro

byte lowByte;
unsigned long timer;
int adress;

//Setup routine
void setup(){
  Wire.begin();                //Start the I2C as master
  Serial.begin(9600);          //Start the serial connetion @ 9600bps
  delay(250);                  //Give the gyro time to start 
}
//Main program
void loop(){
  if(adress == 0){
    Serial.println("Searching for divice");
    for(adress = 0; adress < 255; adress ++){
      Wire.beginTransmission(adress);
      Wire.write(0x0F);
      Wire.endTransmission();
      Wire.requestFrom(adress, 1);
      timer = millis() + 100;
      while(Wire.available() < 1 && timer > millis());
      lowByte = Wire.read();
      if(lowByte == 211){
        Serial.println("");
        Serial.print("Sensor L3G4200 found @ adress:");
        Serial.println(adress);
        adress = 256;
      }
      else if(lowByte == 215){
        Serial.println("");
        Serial.print("Sensor L3GD20H found @ adress:");
        Serial.println(adress);
        adress = 256;
      }
      else Serial.print(".");
    }
    if(adress == 255){
      Serial.println("");
      Serial.println("divice not found!");
    }
  }
}

