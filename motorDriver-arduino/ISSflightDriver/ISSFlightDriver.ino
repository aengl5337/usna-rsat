//I2C Motor Test 8
//Arduino receive command via I2C from RaspPi
//Created by Peter Sinkovitz, Garrett Hamilton, Alec Engl
//REV6: 27JUL21 by Alec Engl to Read Feedback from Stepper Motor Encoder
//REV7: 02AUG21 by Alec Engl to reintroduce SLEEP pin power consumption reduction mechanic
//REV8: 05AUG21 by Alec Engl to changed variable/function naming and heavily commented

#include <Wire.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <Encoder.h>

#define CHA 2 //An interrupt pin
#define CHB 4 //An interrupt pin
#define SLEEP 3
#define MS1 6
#define MS2 7
#define STEP 8
#define DIR 9


// General Motor Params, from MicroMo Datasheets
const int pulsesPerRev_motor = 20; // 18deg per pulse at motor
const double gearRatio = 16807/243; // Gearhead takes apprx 69.2 revs of motor to fully rotate shaft
const int pulsesPerRev_encoder = 10*4; // Encoder "rotates" fully after two full revs of motor

// Specific Motor Params, from Arm Hardware Constraints (degrees) ***make an indexable array of lims and inits for each motor
const int MotorID = 7; // ***Motor ID number***
//const float thetaMin = 0;
//const float thetaMax = 180; // Make motor redundant in Arduino also?  Since it can store absolute position of motor?
const float theta0 = 0; // Initial position of motor

// Motor Driver Params
const int stepMode = 8; //Due to selected wiring config, we have no control over this number -- is 8 by default, 1/8 microsteps
const int halfstepdt = 1000/stepMode; //microseconds; half of the total time it takes to issue 'HIGH-LOW' to STEP
const double pulsesPerRev_shaft = gearRatio*pulsesPerRev_motor*stepMode;  //Gearhead takes ~69.2 revs of motor to fully rotate shaft, multiplied by number of fractional steps
const double theta2stepperCount = pulsesPerRev_shaft/360; //convert input command angle to stepper counts (~31)
const double encoder2shaftAngle = 360/(gearRatio*pulsesPerRev_encoder);

// I2C Commanding/Loop Variables
int iCount = 0; // Used to decode i2c communications

int relCommandTheta = 0; // "Relative Desired theta" -- the integer angle that's received from I2C command from RPi for motor to rotate RELATIVELY to its current position, can we make this not int???
int relCommandThetaMulti = 0; // How many multiples of 180?  Since transmitting over I2C in 1 byte, can't go over 255, and thus 180 is a clean angle ????? - why does this exist

int activeMotor = 0; // Is motor on could be bool???
bool forward = 1; // Forward (CCW) or backward (CW) (Forward = 1, backward = 0)

int relCommandStepperCount = 0; // Based on relCommandTheta, how many steps should motor rotate?
int relCurrentStepperCount = 0; // Updated step count -- once relCurrentStepperCount==relCommandStepperCount, angle command has been fully executed

// Encoder Tracking/Loop Variables
long oldPosition  = -999; //??? why -999
float oldTheta = 0;
long newPosition = 0;
float newTheta = 0;

// EEPROM/Absolute Position Tracking/Loop Variables
byte absMemAdd2 = 0; // Memory address for 2nd byte ('multi')
byte absMemAdd1 = 2; // Memory address for 1st byte ('absMemStepperCount1'), why NOT '1'??? can we rearrange these also

// may be able to avoid using two variables here, since absCount = count+256*multi
byte absMemStepperCount1;     // 1st byte of absolute step-position of motor over its lifetime, [0,255]
byte absMemStepperCount2;     // 2nd byte of absolute step-position of motor over its lifetime, [0,255]*256
float absMemStepperAngle;     // Actual angle for memory use, *not actually stored anywhere????

// Loop Variables **may be able to remove a bunch of '=0' and merely initiate
//Peter's


//-------------------
// ENCODER INIT.
//-------------------
  
  Encoder myEnc(CHA, CHB);
  //   avoid using pins with LEDs attached
  // Change these two numbers to the pins connected to your encoder.
  //   Best Performance: both pins have interrupt capability
  //   Good Performance: only the first pin has interrupt capability
  //   Low Performance:  neither pin has interrupt capability


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

void turnStepper(bool forward) { // if 'forward' non-bool, pretty sure this throws an exception
  if forward //CCW
    digitalWrite(DIR, LOW);
    Serial.println("CCW");
  else !forward //CW
    digitalWrite(DIR, LOW);
    Serial.println("CW");
  /*
  else // if 'forward' non-bool
    Serial.println("ERROR: INVALID MANEUVER - EXERCISE ABORTED");
  */
  
  // Rapidly turn STEP on/off in order to rotate motor.  Will rotate until theoretically reached commanded angle.
  while(relCurrentStepperCount<relCommandStepperCount){
    
    digitalWrite(STEP,HIGH);
    delayMicroseconds(halfstepdt);
    digitalWrite(STEP, LOW);
    delayMicroseconds(halfstepdt);

    // When command one step, poll encoder to get feedback on angular position
    newPosition = myEnc.read();
    if (newPosition != oldPosition) {
      oldPosition = newPosition;
      newTheta = newPosition*encoder2shaftAngle; // Converts from encoder counts to angle
      oldTheta = newTheta;
      Serial.print(newPosition);
      Serial.print(": ");
      Serial.println(newTheta);
    
    relCurrentStepperCount++;
    Serial.println(relCurrentStepperCount);

    absMemStepperCount2 = EEPROM.read(absMemAdd2);
    absMemStepperCount1 = EEPROM.read(absMemAdd1);
    //Serial.println(absMemStepperCount1);
    if forward{
      if(absMemStepperCount1==255){
        absMemStepperCount2 = absMemStepperCount2 + 1;
      }
    }else{
      if(absMemStepperCount1==0){
        absMemStepperCount2 = absMemStepperCount2 - 1;
      }
    }
    
    //absMemStepperCount1 = absMemStepperCount1 + 1; //????
    EEPROM.write(absMemAdd2, absMemStepperCount2);
    EEPROM.write(absMemAdd1, absMemStepperCount1);
    }
  }
}

void receiveData(int byteCount) {
  while(Wire.available()){    // slave may send less than requested
    
    //Serial.println("Message received:");
    int commandByte = Wire.read(); // receive a byte as character
        
    if(iCount == 4){
      relCommandTheta = commandByte + (relCommandThetaMulti*180);
      Serial.print("Desired Theta = ");
      Serial.println(relCommandTheta);

      relCommandStepperCount = relCommandTheta*theta2stepperCount;
      Serial.print("Commanded Stepper Count = ");
      Serial.println(relCommandStepperCount);
      relCurrentStepperCount = 0; //resets every time a new command comes in
      turnStepper(forward);     
      
      digitalWrite(DIR, LOW);
      digitalWrite(STEP, LOW); //????
      digitalWrite(SLEEP,LOW);
     

      relCommandTheta = 0; // Reset desired theta to indicate it's been reached
      activeMotor = 0;//////////////////////////////////////////////////////////////
      iCount = 5; // iCount 4 ==> 5
    }    

    if(iCount == 3){
      relCommandThetaMulti = commandByte;
      Serial.print("Multiple: ");
      if(relCommandThetaMulti == 1){
        Serial.println("YES");
      }
      else if(relCommandThetaMulti == 0){
        Serial.println("NO");
      }
      else{
        Serial.println("ERROR - INVALID");
      }
      iCount = 4; // iCount 3 ==> 4
    }
    
    if(iCount == 2){
      forward = commandByte;
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
      
      iCount = 3; // iCount 2 ==> 3
    }

    if(iCount == 1){
      iCount = 2; // iCount 1 ==> 2
    }

    if(iCount == 0){ // Every 5 "mississippi's," check to see if motor has been called:
                     // If has been, this will trigger the 'if'.  
                     // If not, then it will roll into the 'else' and wait another 5 "mississippi's"
      if(commandByte == MotorID && activeMotor == 0){  
        // if motor is called by RPi, and is currently asleep/inactive:
        activeMotor = 1;  // ...Indicate that motor is active,
        Serial.println("Motor Active");
        iCount = 1; // ...iCount 0 ==> 1,
        digitalWrite(SLEEP,HIGH); // ...And wake up motor.
      }else{
        // if motor isn't called by RPi, ***or is called and is currently active***???:
        Serial.println("Motor Inactive"); // ...Indicate that motor is inactive,
        iCount = 0; // ...iCount 0 ==> 0 (stays put),
        // Use activeMotor as a counter, once this statement has passed 5 times then make it '0' again -- see above for explanation
        activeMotor++;
        if(activeMotor >= 5){
          activeMotor = 0;
        }
      }
    }
    
    if(iCount == 5){
      Serial.println("MOTOR RESET");
      Serial.println(" ");
      Serial.println("Current motor position:  ");
      absMemStepperCount2 = EEPROM.read(absMemAdd2);// MAKE THIS A FUNCTION
      Serial.print("Multiple: ");
      Serial.println(absMemStepperCount2);
      absMemStepperCount1 = EEPROM.read(absMemAdd1);
      Serial.print("Stepper Motor Count: ");
      Serial.println(absMemStepperCount1);
      if(absMemStepperCount2>128){
        motangle = (absMemStepperCount2-256)*6.375 + absMemStepperCount1/theta2stepperCount;
      }
      if(absMemStepperCount2<128){
        motangle = (absMemStepperCount2)*6.375 + absMemStepperCount1/theta2stepperCount;
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


//-------------------
// MAIN
//-------------------

void setup() {    
  Serial.begin(9600);
  Wire.begin(0x9);
  Wire.onReceive(receiveData);     
  /*
  Wire.onReceive(handler)
  Description
  Registers a function to be called when a slave device receives a transmission from a master.
  
  Parameters
  handler: the function to be called when the slave receives data; this should take a single int parameter (the number of bytes read from the master) and return nothing, e.g.: void myHandler(int numBytes)
  */
  
  pinMode(SLEEP, INPUT_PULLUP); //?????
  digitalWrite(SLEEP,LOW); //????
        
  pinMode(MS1, OUTPUT); //MS1
  pinMode(MS2, OUTPUT); //MS2
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(CHA, INPUT); //CHA
  pinMode(CHB, INPUT); //CHB
  
  pinMode(SLEEP,OUTPUT); //????
  digitalWrite(STEP, LOW);
  digitalWrite(DIR, LOW); //LOW = CCW, HIGH = CW with NSTAR wiring config v1

  configEasyDriver(stepMode);

  // This will have to change to i2c!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  

  Serial.println(" ");
  Serial.println("Current motor position:  ");
  absMemStepperCount2 = EEPROM.read(absMemAdd2);
  //Serial.print("[FROM MEM] Multiple: "); //??? DO AN ADDITION OPERATION HERE
  //Serial.println(absMemStepperCount2);
  absMemStepperCount1 = EEPROM.read(absMemAdd1);
  Serial.print("[FROM MEM] Absolute Motor Count: ")
  Serial.println(absMemStepperCount1);
  if(absMemStepperCount2>128){
    motangle = (absMemStepperCount2-256)*6.375 + absMemStepperCount1/theta2stepperCount;
  }
  if(absMemStepperCount2<128){
    motangle = (absMemStepperCount2)*6.375 + absMemStepperCount1/theta2stepperCount;
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
