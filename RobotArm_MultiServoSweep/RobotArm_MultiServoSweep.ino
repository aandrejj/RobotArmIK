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
//

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

#define SPEED         60                // degree/s


Servo servo01; //zakladna
Servo servo02; //spodne hnede rameno
Servo servo03; //horne  biela rameno
Servo servo04; //ruka nabok  100 = zhruba vodorovne
Servo servo05; //ruka hore
Servo servo06; //ruka otvorena= 100, zatvorena = 60
//Servo servos[7];

Buttons buttons;
InverseKinematics inverseKinematics;
RoboArmTurn roboArmTurn;
ServosManager servosManager;
LinearRampXYZ linearRampXYZ;

GripPositionXYZ currentGripPosition;
GripPositionXYZ targetGripPosition;
//ArmServoAngles currentArmAngles;
ArmServoAngles currentArmAngles;
ArmServoMicrosec currentArmMicrosec;


int speedDelay = 50;
int index = 0;
int runRobotArm = 0;
int initializationDone = 0;
boolean previousSensorVal = LOW;
bool partialMovementIsDone =  true;//false;
//boolean previousButtonsVals[3] = {LOW, LOW, LOW};

ArmServoAngles servosInitialPosition;
int servo1PPos = 90; //zakladna
int servo2PPos = 100;//spodne hnede rameno
int servo3PPos = 35; //horne  biela rameno
int servo4PPos = 100;//ruka nabok  100 = zhruba vodorovne
int servo5PPos = 85; //ruka hore
int servo6PPos = 80; //ruka otvorena= 100, zatvorena = 60
//----------------------setup-------------------------------------------
void setup() {
  // Open serial communications and wait for port to open:
  //Serial.begin(19200);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("RobotArm_servoSweep  Started--------------------------------------------------------");
  Serial.println();
  //configure pin 2 as an input and enable the internal pull-up resistor
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  /*
  servosInitialPosition.baseAngle      = pservo1Pos; //zakladna
  servosInitialPosition.arm1Angle      = pservo2Pos; //spodne hnede rameno
  servosInitialPosition.arm2Angle      = pservo3Pos; //horne  biela rameno
  servosInitialPosition.gripSpinAngle  = pservo4Pos; //ruka nabok
  servosInitialPosition.gripTiltAngle  = pservo5Pos; //ruka hore
  servosInitialPosition.gripAngle      = pservo6Pos; //ruka otvorena
  servosInitialPosition.movesScriptEnd = false;
  */

 //servosManager.setServos(servo01, servo02, servo03, servo04, servo05, servo06);
 
  //char moves[] = "[{\"servoNumber\": 1, \"positionFrom\": 0, \"positionTo\": 180},{\"servoNumber\": 2, \"positionFrom\": 0, \"positionTo\": 180}]";
  //servoSweepJson(moves,"Method2 text.");

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
      Serial.println("loop: Servo Initialization. runRobotArm == 0");
      currentArmAngles = servosManager.ServoInitialization(servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos);
      currentGripPosition = inverseKinematics.convertAngleToPosXYZ(currentArmAngles);      
      currentArmMicrosec = inverseKinematics.moveToPosXYZ(currentGripPosition);
      initializationDone = 1;
      delay(999);
    }
  }
  if(runRobotArm == 2) {
      Serial.println("loop: Servo movement Start. runRobotArm == 2 ");
      
      if(initializationDone == 1) {
        if(partialMovementIsDone==true) {
          targetGripPosition = roboArmTurn.takeNextRoboArmPosition();        
          partialMovementIsDone = false;
          currentArmMicrosec = inverseKinematics.moveToPosXYZ(currentGripPosition);
        } else {
          currentArmAngles = servosManager.updateCurrentAngles(currentArmAngles);
          currentArmMicrosec = inverseKinematics.moveToAngle((double)currentArmAngles.baseAngle, (double)currentArmAngles.arm1Angle, (double)currentArmAngles.arm2Angle, (int)currentArmAngles.gripSpinAngle, (int)currentArmAngles.gripTiltAngle, (double)currentArmAngles.gripAngle);
          //ToDo Here: add param like currentArmAngles or currentArmMiliseconds
        }
        servosManager.updateServos(currentArmMicrosec);   // update servos according to InverseKinematics Values
        delay(10);       // lazy way to limit update to 100 Hz
      }
      
      //mainRoboArmTurn();
      mainRoboArmUpdate();
      Serial.println("loop: Servo End.");
      runRobotArm = runRobotArm - 1;  // = 2
  }
  
  
}// ----------end of loop-------------------------------------------

//--------------------mainRoboArmUpdate-----------------------------
void mainRoboArmUpdate() {
  Serial.println("mainRoboArmUpdate Started. ");
  servosManager.updateServos(currentArmMicrosec);   // update servos according to InverseKinematics Values
  delay(10);       // lazy way to limit update to 100 Hz
  
  //ToDo Remove this old mainRoboArmTurn
  //roboArmTurn.mainRoboArmTurn_old();
  
  Serial.println("mainRoboArmUpdate End OK. ");
}
//--------------------End of mainRoboArmUpdate----------------------
