
/* Arduino Read Feedback from Stepper Motor Encoder

Created by Alec Engl, with [a lot of] help from https://www.youtube.com/watch?v=J9cDEef0IbQ

*/


#include <Encoder.h>

//#define analogCHA 1
//#define analogCHB 2
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

// Motor Driver Params
//const int sampdt = 1000; //microseconds; frequency of sampling on analog/digital pins
const int stepMode = 1;
const int halfstepdt = 1000/stepMode; //microseconds; half of the total time it takes to issue 'HIGH-LOW' to STEP
const float pulsesPerRev_shaft = gearRatio*pulsesPerRev_motor*stepMode;  //Gearhead takes 69.2 revs of motor to fully rotate shaft, multiplied by number of fractional steps
const float encoder2shaftAngle = 360/(gearRatio*pulsesPerRev_encoder);

// Loop variables
//int aCHA = 0, aCHB = 0, dCHA = 0, dCHB = 0;
//int long count = 0; // Starts on 0th iteration
//int long supercount = 0;
volatile int long virtualEncoderCount = 0; //updated by the interrupt service routine (ISR)****
int long lastCount = 0; //***
float virtualShaftPosition = theta0; //***
const int bouncedt = 93; //microseconds***

// Command
const int discreteDrive = 1; //0 = continuous driving, 1 = discrete driving
float theta = 36000;
//int long thetaStepperCount = theta/encoder2shaftAngle;  /conversion may lead to imprecision
int long thetaStepperCount = 2;
long oldPosition  = -999;
float oldTheta = 0;

// ---------
//   CODE
// ---------

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(CHA, CHB);
//   avoid using pins with LEDs attached

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

void setup() {               
  pinMode(MS1, OUTPUT); //MS1
  pinMode(MS2, OUTPUT); //MS2
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(CHA, INPUT); //CHA
  pinMode(CHB, INPUT); //CHB
  digitalWrite(STEP, LOW);
  digitalWrite(DIR, LOW); //LOW = CCW, HIGH = CW with NSTAR wiring config v1

  configEasyDriver(stepMode);
  //attachInterrupt(digitalPinToInterrupt(CHA), isr, LOW);
  
  Serial.begin(115200); //***

  if(discreteDrive){
    for(int long i=0; i<thetaStepperCount; i++){
      digitalWrite(STEP, HIGH);
      delayMicroseconds(halfstepdt);         
      digitalWrite(STEP, LOW);
      delayMicroseconds(halfstepdt);
      
      long newPosition = myEnc.read();
      if (newPosition != oldPosition) {
        oldPosition = newPosition;
        newTheta = newPosition*encoder2shaftAngle;
        oldTheta = newTheta;
        Serial.print(newPosition);
        Serial.print(": ");
        Serial.println(newTheta);
    }
  }
}

void loop() {
  
    digitalWrite(STEP, HIGH);
    delayMicroseconds(halfstepdt);         
    digitalWrite(STEP, LOW);
    delayMicroseconds(halfstepdt);
    
    long newPosition = myEnc.read();
    if (newPosition != oldPosition) {
      oldPosition = newPosition;
      newTheta = newPosition*encoder2shaftAngle;
      oldTheta = newTheta;
      Serial.print(newPosition);
      Serial.print(": ");
      Serial.println(newTheta);
  
/*
  if(virtualEncoderCount != lastCount) { //Volatile variable is updated automatically outside of this loop
    // Write new position to serial monitor
    
    Serial.print(virtualEncoderCount > lastCount ? "Up :" : "Down:");
    Serial.print(virtualEncoderCount);
    
    
    // Transform from Encoder Counts to Traversed Angle
    virtualShaftPosition =  virtualEncoderCount*360/(pulsesPerRev_shaft*gearRatio);
    Serial.print("; ");
    Serial.print(virtualShaftPosition);
    Serial.println();
    
    // Constrain Value
    //virtualShaftPosition = min(thetaMax, max(thetaMin, virtualShaftPosition)); // Beware of false shifting

    // Update value
    lastCount = virtualEncoderCount;
  }
*/
}

/*
void loop() {
  digitalWrite(8, HIGH);
  delay(1);         
  digitalWrite(8, LOW);
  delay(1);
  if((count%1000)==0){
    Serial.println(count);
  }
  count++;
}
*/
