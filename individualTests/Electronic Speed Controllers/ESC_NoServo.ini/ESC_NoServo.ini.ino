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
    // check for incoming serial data:
  if (Serial.available() > 0) {
    // read incoming serial data:
    char inChar = Serial.read();
    if(inChar == 'u'){
        incrementStep();
      }
     if(inChar == 'd'){
        decrementStep();
      }
      if(inChar == 'c'){
      val=1520;  
      }
       if(inChar == 'i'){
      val=2000;  
      }
       if(inChar == 'f'){
      val=1000;  
      }
     
    // Type the next ASCII value from what you received:
    Serial.write(inChar);
  }

  float test=2.3;
  Serial.print(test);
  Serial.print(periodMicro);
  digitalWrite(9,HIGH);
  delayMicroseconds(val);
  digitalWrite(9, LOW);
  delayMicroseconds(periodMicro- val); 
  
}

void incrementStep(){
  if((val+res)>2000){
    val=2000;
    }
   else{
    val = val + res;
    }
  }

void decrementStep(){
  if((val-res)<1000){
    val=1000;
    }
   else{
    val = val - res;
    }
  }

















