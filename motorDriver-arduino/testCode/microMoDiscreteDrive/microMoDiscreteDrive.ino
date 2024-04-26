
/* Arduino Read Feedback from Stepper Motor Encoder

Created by Alec Engl, with [a lot of] help from https://www.youtube.com/watch?v=J9cDEef0IbQ

*/

//#define analogCHA 1
//#define analogCHB 2
#define CHA 2 //An interrupt pin
#define CHB 4
#define SLEEP 3
#define MS1 6
#define MS2 7
#define STEP 8
#define DIR 9

// General Motor Params, from MicroMo Datasheets
const int pulsesPerRev_motor = 20; // 18deg per pulse at motor
const float gearRatio = 69.2; // Gearhead takes 69.2 revs of motor to fully rotate shaft
const int pulsesPerRev_encoder = 10; // Encoder "rotates" fully after two full revs of motor

// Specific Motor Params, from Arm Hardware Constraints (degrees)
const float thetaMin = 0;
const float thetaMax = 180;
const float theta0 = 0; // Initial position of motor

// Motor Driver Params
const int halfstepdt = 1000; //microseconds; half of the total time it takes to issue 'HIGH-LOW' to STEP
//const int sampdt = 1000; //microseconds; frequency of sampling on analog/digital pins
const int stepMode = 1;
const float pulsesPerRev_shaft = gearRatio*pulsesPerRev_motor*stepMode;  //Gearhead takes 69.2 revs of motor to fully rotate shaft, multiplied by number of fractional steps
const float encoder2shaftAngle = 360/(pulsesPerRev_motor*gearRatio);

// Loop variables
//int aCHA = 0, aCHB = 0, dCHA = 0, dCHB = 0;
//int long count = 0; // Starts on 0th iteration
//int long supercount = 0;
//volatile int long virtualEncoderCount = 0; //updated by the interrupt service routine (ISR)****
//int long lastCount = 0; //***
//float virtualShaftPosition = theta0; //***
///const int bouncedt = 0; //microseconds***

// Command
float theta = 36000;
//int long thetaStepperCount = theta/encoder2shaftAngle;  /conversion may lead to imprecision
int long thetaStepperCount = 2;

// ---------
// INTERRUPT
// ---------

/*
void isr() {
  static unsigned long lastInterruptTime = 0;
  //unsigned long = interruptTime = millis();
  unsigned long interruptTime = micros();
  // millis()
  // Returns the number of milliseconds passed since the Arduino board began running the current program. This number will overflow (go back to zero), after approximately 50 days.
   
   if((interruptTime-lastInterruptTime) > bouncedt) { //debounces all disturbances longer than bouncedt
    //Indicates that A is rising
    if(digitalRead(CHB) == LOW) {
      //travelling CW
      virtualEncoderCount--;
    } else {
      //travelling CCW
      virtualEncoderCount++;
    }
    
    // Keep previous time value as history for debouncing
    lastInterruptTime = interruptTime;
   }
} 
*/

// ---------
//   CODE
// ---------

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
  Serial.print(thetaStepperCount);

  
  
  for(int long i=0; i<thetaStepperCount; i++){ //MUST be long int if exceeds 2-byte (2^15-1)
    digitalWrite(STEP, HIGH);
    delayMicroseconds(halfstepdt);         
    digitalWrite(STEP, LOW);
    delayMicroseconds(halfstepdt);
  }
}

void loop() {
/*
  if(virtualEncoderCount != lastCount) { //Volatile variable is updated automatically outside of this loop
    // Write new position to serial monitor
    
    Serial.print(virtualEncoderCount > lastCount ? "Up :" : "Down:");
    Serial.print(virtualEncoderCount);
    
    
    // Transform from Encoder Counts to Traversed Angle
    virtualShaftPosition =  virtualEncoderCount*;
    Serial.print("; ");
    Serial.print(virtualShaftPosition);
    Serial.println();
    
    // Constrain Value
    //virtualShaftPosition = min(thetaMax, max(thetaMin, virtualShaftPosition)); // Beware of false shifting

    // Update value
    lastCount = virtualEncoderCount;
  }
*/
delay(100);
}
