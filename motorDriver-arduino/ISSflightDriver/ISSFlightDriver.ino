//I2C Motor Test 6
//Arduino receive command via I2C from RaspPi
//Created by Peter Sinkovitz, Garrett Hamilton, Alec Engl
//Updated on 27JUL21 by Alec Engl to Read Feedback from Stepper Motor Encoder

#include <Wire.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <Encoder.h>

#define CHA 2 //An interrupt pin
#define CHB 3 //An interrupt pin
//#define SLEEP 3
#define MS1 6
#define MS2 7
#define STEP 8
#define DIR 9


// General Motor Params, from MicroMo Datasheets
const int pulsesPerRev_motor = 20; // 18deg per pulse at motor
const float gearRatio = 16807/243; // Gearhead takes apprx 69.2 revs of motor to fully rotate shaft
const int pulsesPerRev_encoder = 10*4; // Encoder "rotates" fully after two full revs of motor

// Specific Motor Params, from Arm Hardware Constraints (degrees)
const float thetaMin = 0;
const float thetaMax = 180;
const float theta0 = 0; // Initial position of motor
const int MotorID = 8; // Motor ID number

// Motor Driver Params
//const int sampdt = 1000; //microseconds; frequency of sampling on analog/digital pins
const int stepMode = 1;
const int halfstepdt = 1000/stepMode; //microseconds; half of the total time it takes to issue 'HIGH-LOW' to STEP
const float pulsesPerRev_shaft = gearRatio*pulsesPerRev_motor*stepMode;  //Gearhead takes 69.2 revs of motor to fully rotate shaft, multiplied by number of fractional steps
const float encoder2shaftAngle = 360/(gearRatio*pulsesPerRev_encoder);

// I2C Params, Loop Variables
int iCount = 0; // Used to decode i2c communications
int T = 0; // Added Tracker
int forward = 1; // Forward or backward (Forward = 1, backward = 0)
int multiple = 0; // How many multiples of 180
int n = 0; // Ticks for desired angle
int memadd1 = 0;
int memadd2 = 2;
int multi; // Multiple of 180 for memory use
float motangle; // Actual angle for memory use
int motcount; // Ticks for memory use
float theta2n = 31;
int ThetaD = 0;
long oldPosition  = -999;
float oldTheta = 0;
int activeMotor = 0; // Is motor on


//-------------------
// FUNCTIONS
//-------------------

void configEasyDriver(int stepMode) {
  // stepMode corresponds to a type of stepping:
  // 1 = full steps (0,0)
  // 2 = half steps (1,0)
  // 4 = quarter steps (0,1)
  // 8 = eighth steps (1,1) [DEFAULT]

  pinMode(MS1,OUTPUT);
  pinMode(MS2,OUTPUT);
  
  if(stepMode==1){
    digitalWrite(MS1,LOW);
    digitalWrite(MS2, LOW);
  }else if(stepMode==2){
    digitalWrite(MS1,HIGH);
    digitalWrite(MS2,LOW);
  }else if(stepMode==4){
    digitalWrite(MS1,LOW);
    digitalWrite(MS2,HIGH);
  }else if(stepMode==8){
    digitalWrite(MS1,HIGH);
    digitalWrite(MS2,HIGH);
  }
}

void receiveData(int byteCount) {
  while(Wire.available()){    // slave may send less than requested
    
    //Serial.println("Message received:");
    int message = Wire.read(); // receive a byte as character
    //Serial.println(message);
    
    if(iCount == 4){
      ThetaD = message + (multiple*180);
      Serial.print("Theta D = ");
      Serial.println(ThetaD);

      n = ThetaD*theta2n;
      Serial.print("N = ");
      Serial.println(n);
      T = 0;
      if(forward == 1){
        ForwardTurn();
      }
      else if(forward == 0){
        BackwardTurn();
      }
      else{
        Serial.println("ERROR: INVALID MANEUVER - EXERCISE ABORTED");
      }
      
      digitalWrite(DIR, LOW);
      digitalWrite(STEP, LOW);
      digitalWrite(SLEEP,LOW);
     

      ThetaD = 0;
      activeMotor = 0;//////////////////////////////////////////////////////////////
      iCount = 5;
    }    

    if(iCount == 3){
      multiple = message;
      Serial.print("Multiple: ");
      if(multiple == 1){
        Serial.println("YES");
      }
      else if(multiple == 0){
        Serial.println("NO");
      }
      else{
        Serial.println("ERROR - INVALID");
      }
      iCount = 4;
    }
    
    if(iCount == 2){
      forward = message;
      Serial.print("Motor Direction: ");
      if(forward == 1){
        Serial.println("Positive.");
      }
      else if(forward == 0){
        Serial.println("Negatve.");
      }
      else{
        Serial.println("UNDEFINED - ERROR.");
      }
      
      iCount = 3;
    }

    if(iCount == 1){
      iCount = 2;
    }
    
    if(iCount == 0){
      if(message == MotorID && activeMotor == 0){
        activeMotor = 1;
        Serial.println("Motor Active");
        iCount = 1;
        digitalWrite(SLEEP,HIGH);
      }
      else{
        Serial.println("Motor Inactive");
        iCount = 0;
        activeMotor = activeMotor + 1;
        if(activeMotor >= 5){
          activeMotor = 0;
        }
      }
    }
    if(iCount == 5){
      Serial.println("MOTOR RESET");
      Serial.println(" ");
      Serial.println("Current motor position:  ");
      multi = EEPROM.read(memadd1);
      Serial.print("Multiple: ");
      Serial.println(multi);
      motcount = EEPROM.read(memadd2);
      Serial.print("Encoder Motor Count: ");
      Serial.println(motcount);
      if(multi>128){
        motangle = (multi-256)*6.375 + motcount/theta2n;
      }
      if(multi<128){
        motangle = (multi)*6.375 + motcount/theta2n;
      }
      if(motangle<0){
        motangle = motangle + 360;
      }
      if(motangle>360){
        motangle = motangle - 360;
      }
      Serial.print("Actual Motor Angle: ");
      Serial.println(motangle);
      Serial.println(" ");
      iCount = 0;
    }
  }
}  

void ForwardTurn() {
  Serial.println("front");
  
  while(T<n){
    
    digitalWrite(DIR, LOW);
    digitalWrite(STEP,HIGH);
    delayMicroseconds(halfdt);
    digitalWrite(STEP, LOW);
    delayMicroseconds(halfdt);

    long newPosition = myEnc.read();
    if (newPosition != oldPosition) {
      oldPosition = newPosition;
      newTheta = newPosition*encoder2shaftAngle;
      oldTheta = newTheta;
      Serial.print(newPosition);
      Serial.print(": ");
      Serial.println(newTheta);
    
    T++;
    
    multi = EEPROM.read(memadd1);
    motcount = EEPROM.read(memadd2);
    //Serial.println(motcount);
    if(motcount==255){
      multi = multi + 1;
    }
    
    motcount = motcount + 1;
    EEPROM.write(memadd1, multi);
    EEPROM.write(memadd2, motcount);
  }
}

void BackwardTurn() {
  Serial.println("back");
  
  while(T<n){
  
    digitalWrite(DIR, HIGH);
    digitalWrite(STEP,HIGH);
    delayMicroseconds(halfdt);
    digitalWrite(STEP, LOW);
    delayMicroseconds(halfdt);

    long newPosition = myEnc.read();
    if (newPosition != oldPosition) {
      oldPosition = newPosition;
      newTheta = newPosition*encoder2shaftAngle;
      oldTheta = newTheta;
      Serial.print(newPosition);
      Serial.print(": ");
      Serial.println(newTheta);
    
    T++;
    
    Serial.println(T);
    multi = EEPROM.read(memadd1);
    motcount = EEPROM.read(memadd2);
    //Serial.println(motcount);
    if(motcount==0){
      multi = multi - 1;
    }
    
    EEPROM.write(memadd1, multi);
    EEPROM.write(memadd2, motcount);
  }
}

//-------------------
// MAIN
//-------------------

void setup() {    
  Serial.begin(9600);
  Wire.begin(0x9);
  Wire.onReceive(receiveData);     
  
  pinMode(SLEEP, INPUT_PULLUP);
  digitalWrite(SLEEP,LOW);
        
  pinMode(MS1, OUTPUT); //MS1
  pinMode(MS2, OUTPUT); //MS2
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(CHA, INPUT); //CHA
  pinMode(CHB, INPUT); //CHB
  
  pinMode(3,OUTPUT); //????
  digitalWrite(STEP, LOW);
  digitalWrite(DIR, LOW); //LOW = CCW, HIGH = CW with NSTAR wiring config v1

  configEasyDriver(stepMode);
  
  Encoder myEnc(CHA, CHB);
  //   avoid using pins with LEDs attached
  // Change these two numbers to the pins connected to your encoder.
  //   Best Performance: both pins have interrupt capability
  //   Good Performance: only the first pin has interrupt capability
  //   Low Performance:  neither pin has interrupt capability


  // This will have to change to i2c!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  

  Serial.println(" ");
  Serial.println("Current motor position:  ");
  multi = EEPROM.read(memadd1);
  Serial.print("Multiple: ");
  Serial.println(multi);
  motcount = EEPROM.read(memadd2);
  Serial.print("Encoder Motor Count: ");
  Serial.println(motcount);
  if(multi>128){
    motangle = (multi-256)*6.375 + motcount/theta2n;
  }
  if(multi<128){
    motangle = (multi)*6.375 + motcount/theta2n;
  }
    if(motangle<0){
    motangle = motangle + 360;
  }
  if(motangle>360){
    motangle = motangle - 360;
  }
  Serial.print("Actual Motor Angle: ");
  Serial.println(motangle);
  Serial.println(" ");
}

void loop() {
  delay(100);
}
