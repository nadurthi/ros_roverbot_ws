
/*
In1 In2 PWM Out1  Out2  Mode
H H H/L L L Short brake
L H H L H CCW
L H L L L Short brake
H L H H L CW
H L L L L Short brake
L L H OFF OFF Stop
 * 
 * 
 */

//#include <Servo.h>
#include <Wire.h>

#define LeftPwm 9
#define LeftPinB 8
#define LeftPinF 7

#define StbyPin 18

#define RightPwm 10
#define RightPinF 19
#define RightPinB 20

#define offsetind 0
#define delayind 1
#define LPWMind 2
#define RPWMind 3
#define dirind 4

int c[5];
int flag;
int dly;
unsigned long time;
//Servo servoL; 
//Servo servoR; 

void setup() {
  flag=0;
  c[offsetind]=0;
  c[delayind]=0;
  c[LPWMind]=0;
  c[RPWMind]=0;
  c[dirind]=0;
  
  pinMode(StbyPin, OUTPUT);
  digitalWrite(StbyPin, LOW);

  pinMode(LeftPinF, OUTPUT);
  pinMode(LeftPinB, OUTPUT);
  pinMode(RightPinF, OUTPUT);
  pinMode(RightPinB, OUTPUT);

  digitalWrite(LeftPinF, LOW);
  digitalWrite(LeftPinB, LOW);
  digitalWrite(RightPinF, LOW);
  digitalWrite(RightPinB, LOW);
  
  Wire.begin(8); 
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);          

  //servoL.attach(9);
//  servoR.attach(10);

//  servoL.write(0);
//  servoR.write(0);
  analogWrite(LeftPwm, 0);
  analogWrite(RightPwm, 0);
  
  Serial.println("Ready");
  digitalWrite(StbyPin, HIGH);
}

void loop() {
  delay(1);
  
  if(flag==1){
    
    dly=c[delayind]*4;

    
    switch (c[dirind]) {
      case 22:
        digitalWrite(LeftPinF, HIGH);
        digitalWrite(LeftPinB, LOW);
        digitalWrite(RightPinF, HIGH);
        digitalWrite(RightPinB, LOW);
        break;
      case 11:
        digitalWrite(LeftPinF, LOW);
        digitalWrite(LeftPinB, HIGH);
        digitalWrite(RightPinF, LOW);
        digitalWrite(RightPinB, HIGH);
        break;
      case 21:
        digitalWrite(LeftPinF, HIGH);
        digitalWrite(LeftPinB, LOW);
        digitalWrite(RightPinF, LOW);
        digitalWrite(RightPinB, HIGH);
        break;
      case 12:
        digitalWrite(LeftPinF, LOW);
        digitalWrite(LeftPinB, HIGH);
        digitalWrite(RightPinF, HIGH);
        digitalWrite(RightPinB, LOW);
        break;
      default:
        digitalWrite(LeftPinF, LOW);
        digitalWrite(LeftPinB, LOW);
        digitalWrite(RightPinF, LOW);
        digitalWrite(RightPinB, LOW);
        break;
    }
    analogWrite(LeftPwm, c[LPWMind]);
    analogWrite(RightPwm, c[RPWMind]);

    c[offsetind]=0;
    c[delayind]=0;
    c[LPWMind]=0;
    c[RPWMind]=0;
    c[dirind]=0;
    
    time = millis();
    flag=0;

  }

  if((millis()-time>dly) || (millis()-time>2000) ){
    analogWrite(LeftPwm, 0);
    analogWrite(RightPwm, 0);
    digitalWrite(LeftPinF, LOW);
    digitalWrite(LeftPinB, LOW);
    digitalWrite(RightPinF, LOW);
    digitalWrite(RightPinB, LOW);
  
    
  }
}


void receiveEvent(int howMany) {

  int i=0;
  int d;
  while (0 < Wire.available()) { 
    d = Wire.read(); 
    c[i]=d;
    ++i;
  }
  

  
  if(i<5){
    c[offsetind]=0;
    c[delayind]=0;
    c[LPWMind]=0;
    c[RPWMind]=0;
    c[dirind]=0;
    
    flag=0;
  }
  else{
    flag=1;
  }
  
  
 
}
