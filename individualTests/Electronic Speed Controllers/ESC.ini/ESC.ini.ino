#include <Servo.h>

Servo myservo;  // create servo object to control a servo


//functions to be used these are defined at the end of the code
 void incrementStep();
 void decrementStep();
int val=1520;    // variable to read the value from the analog pin
int angle;
int res = 20;
void setup()
{
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop()
{
 /* val = 2000;
  delay(100);
  myservo.writeMicroseconds(val);
  Serial.println(val);*/

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

  
  //angle= map(val, 1000, 2000, 544, 2400);
  //Serial.write(val);
   // myservo.write(angle);
  myservo.writeMicroseconds(val);
  //SoftwareServo::refresh();
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

















