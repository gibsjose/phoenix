#include <Servo.h>

Servo myservo;  // create servo object to control a servo


//functions to be used these are defined at the end of the code
 void incrementStep();
 void decrementStep();
int val=1520;    // variable to read the value from the analog pin
int angle;
int res = 20;
int frequency = 50;
float periodMicro = (1000000/frequency); 
void setup()
{
  Serial.begin(9600);
  pinMode(9,OUTPUT);
}

void loop()
{
  //digitalWrite(9,HIGH);
    if (Serial.available() > 0) {
      // read incoming serial data:
    char inChar = Serial.read();
    if(inChar == 'u'){
  digitalWrite(9,HIGH);
      }
     if(inChar == 'd'){
  digitalWrite(9,LOW);
      }
      
    }

 
  
}


















