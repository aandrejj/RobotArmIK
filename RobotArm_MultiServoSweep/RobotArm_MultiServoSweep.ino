/*        
       DIY Arduino Robot Arm 
*/

//
//
// NEXT is https://github.com/siteswapjuggler/RAMP
//         https://github.com/siteswapjuggler/RAMP/blob/master/examples/ramp_servo/ramp_servo.ino
//        
//         https://github.com/roTechnic/InverseKinematicArm
//         https://github.com/roTechnic/InverseKinematicArm/blob/main/inverse_kinematics.ino
//
//  https://github.com/aandrejj/RobotArmIK/pulls



#include "ServosManager.h"
#include "InverseKinematics.h"
#include "Buttons.h"
#include "ServoRamp.h"
#include "LinearRampXYZ.h"
#include "RampOnce.h"
#include "RoboArmTurn.h"

#include <Arduino.h>                    // needed for PlatformIO
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Ramp.h>
#include <ArduinoJson.h>  //https://arduinojson.org/   https://arduinojson.org/v6/api/jsonarray/

#define SERVO_MIN_MILISEC   460                // https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
#define SERVO_MAX_MILISEC  2400                // value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
                                               // ..so that servos often respond to values between 700 and 2300.
#define SPEED         30.0                // not a [m/s] but [mm/s]
//#define DEBUG         //debug logging
#define BRIEF_LOG     //just a few logs

Buttons buttons;
InverseKinematics inverseKinematics;
RoboArmTurn roboArmTurn;
ServosManager servosManager;
LinearRampXYZ linearRampXYZ;

GripPositionXYZ currentGripPosition;
GripPositionXYZ targetGripPosition;

ArmServoAngles startArmAngles;
ArmServoAngles currentArmAngles;

ArmServoMicrosec currentArmMicrosec;


int speedDelay = 50;
int index = 0;
int runRobotArm = 0;
int initializationDone = 0;
boolean previousSensorVal = LOW;
bool partialMovementIsDone =  true;//false;

ArmServoAngles servosInitialPosition;
int servo1PPos = 90; //zakladna
int servo2PPos = 100;//spodne hnede rameno
int servo3PPos = 0; //horne  biela rameno
int servo4PPos = 100;//ruka nabok  100 = zhruba vodorovne
int servo5PPos = 85; //ruka hore
int servo6PPos = 80; //ruka otvorena= 100, zatvorena = 60
//----------------------setup-------------------------------------------
void setup() {
  // Open serial communications and wait for port to open:
    Serial.begin(115200);
  //Serial.begin( 19200);
  //Serial.begin(  9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  #ifdef DEBUG 
    Serial.println("RobotArm_servoSweep  Started--------------------------------------------------------");
  #endif
  #ifdef DEBUG 
    Serial.println(); 
  #endif
  //configure pin 2 as an input and enable the internal pull-up resistor
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  inverseKinematics.begin(SERVO_MIN_MILISEC, SERVO_MAX_MILISEC);
  Serial.println("setup: Setup DONE. System is ready to Initialize servos. Press Button to start...");

}
//----------------------end of setup-----------------------------------




//-----------------loop-------------------------------------------------
void loop() {
  boolean Btn2Clicked = buttons.ButtonHandle(2);
  if(Btn2Clicked == true) {
    runRobotArm = buttons.onButton2Clicked(runRobotArm) ;
  }// end of if (Btn2Clicked)
  
  if(runRobotArm == 1) {
    if (initializationDone == 0) {
      #ifdef DEBUG 
		    Serial.println("loop: Servo Initialization. runRobotArm == 0");
      #endif
      startArmAngles = servosManager.ServoInitialization(servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos,SERVO_MIN_MILISEC ,SERVO_MAX_MILISEC );
      currentGripPosition = inverseKinematics.convertAngleToPosXYZ(startArmAngles);      
      currentArmAngles = inverseKinematics.moveToPosXYZ(currentGripPosition);
      servosManager.updateServos(currentArmAngles);   // send pulses to servos.  update servos according to InverseKinematics Values
      initializationDone = 1;
      delay(100);
      Serial.println("loop: Servo Initialization  DONE. Press Button to start...-----------------------------------------");
    }
  }
  if(runRobotArm == 2) {
      if(initializationDone == 1) {
        if(partialMovementIsDone==true) {
          #ifdef DEBUG 
			      Serial.println("loop: partialMovementIsDone==true, starting  roboArmTurn.takeNextRoboArmPosition()");
          #endif
          targetGripPosition = roboArmTurn.takeNextRoboArmPosition(); //read next target-position from 'moving script'       
          if(!targetGripPosition.movesScriptEnd) {
            partialMovementIsDone = false;
            
            #ifdef DEBUG 
				      Serial.println("loop: before linearRampXYZ.begin(...) currentGripPosition = {"+ String(currentGripPosition.gripX)+", "+ String(currentGripPosition.gripY)+", "+ String(currentGripPosition.gripZ)+", "+ String(currentGripPosition.gripSpinAngle)+", "+ String(currentGripPosition.gripTiltAngle)+", "+ String(currentGripPosition.gripWidth)+","+ String(currentGripPosition.movesScriptEnd)+"}.");
            #endif
            linearRampXYZ.begin(currentGripPosition, targetGripPosition);
            linearRampXYZ.setup();

            currentArmAngles = inverseKinematics.moveToPosXYZ(currentGripPosition);
          } else {
            Serial.println("loop: movesScriptEnd!!!");
            runRobotArm = runRobotArm - 1;  // = 2
          }
          
        } else {
          currentGripPosition = linearRampXYZ.update();
          currentArmAngles = inverseKinematics.moveToPosXYZ(currentGripPosition);
          #ifdef DEBUG 
			      Serial.println("loop: after linearRampXYZ.update() currentGripPosition = {"+ String(currentGripPosition.gripX)+", "+ String(currentGripPosition.gripY)+", "+ String(currentGripPosition.gripZ)+", "+ String(currentGripPosition.gripSpinAngle)+", "+ String(currentGripPosition.gripTiltAngle)+", "+ String(currentGripPosition.gripWidth)+","+ String(currentGripPosition.movesScriptEnd)+"}.");
          #endif

          if(linearRampXYZ.rampOnce.rampIsFinished) {
            Serial.println("loop: linearRampXYZ.rampOnce.rampIsFinished. partialMovementIsDone = true");
            partialMovementIsDone = true;
          }
        }
        if(!targetGripPosition.movesScriptEnd) {
          
          //servosManager.updateServos(currentArmMicrosec);   // send pulses to servos.  update servos according to InverseKinematics Values
          servosManager.updateServos(currentArmAngles);   // send pulses to servos.  update servos according to InverseKinematics Values
          //delay(10);       // lazy way to limit update to 100 Hz
        } else {
          #ifdef DEBUG 
			      Serial.println("loop: delay(100)  @ End.");
          #endif
          delay(100);
        }  
      }      
  }  
}// ----------end of loop-------------------------------------------
