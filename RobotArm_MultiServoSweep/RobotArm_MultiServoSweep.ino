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
#include "BluetoothFactory.h"

#include "EasyTransfer.h"
#include "SoftwareSerial.h"
#include "RemoteController_dataStructures.h"
#include <Math.h>


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <Arduino.h>                    // needed for PlatformIO
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Ramp.h>
#include <ArduinoJson.h>  //https://arduinojson.org/   https://arduinojson.org/v6/api/jsonarray/

#define BT_ON  1 ///1 = Bluetooth is on.  0 = Bluetooth is off

/*
#define IS_HM_10 //uncomment this if module is HM-10


#define SERVO_MIN_MILISEC   460                // https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
#define SERVO_MAX_MILISEC  2400                // value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
#define SERVO_MIN   135                // https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
#define SERVO_MAX   615                // value of 135 is fully counter-clockwise, 615 is fully clockwise.
*/

// Outcomment line below for HM-10, HM-19 etc
//#define HIGHSPEED   // Most modules are only 9600, although you can reconfigure this
#define EN_PIN_HIGH   // You can use this for HC-05 so you don't have to hold the small button on power-up to get to AT-mode

#ifdef HIGHSPEED
  #define Baud 38400   // Serial monitor
  #define BTBaud 38400 // There is only one speed for configuring HC-05, and that is 38400.
#else
  #define Baud 9600    // Serial monitor
  #define BTBaud 9600  // HM-10, HM-19 etc
#endif
/*

#define STATE 11
#define BLUETOOTH_RX 9  // Bluetooth RX -> Arduino D9
#define BLUETOOTH_TX 10 // Bluetooth TX -> Arduino D10
//#define GND 13
//#define Vcc 12
#define ENABLE 8
*/


                                          // ..so that servos often respond to values between 700 and 2300.
#define SPEED         30.0                // not a [m/s] but [mm/s]

//#define DEBUG         //debug logging
#define BRIEF_LOG     //just a few logs

Buttons buttons;
InverseKinematics inverseKinematics;
RoboArmTurn roboArmTurn;
ServosManager servosManager;

BluetoothOutputData bluetoothOutputData;
BluetoothOutputData incrementsFromBtController;
BluetoothFactory bluetoothFactory;

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
int servo2PPos = 160;//spodne hnede rameno
int servo3PPos = 13; //horne  biela rameno
int servo4PPos = 90;//ruka nabok  100 = zhruba vodorovne
int servo5PPos = 72; //ruka hore
int servo6PPos = 110; //ruka otvorena= 100, zatvorena = 60

//SEND_DATA_STRUCTURE mydata_send;
//RECEIVE_DATA_STRUCTURE mydata_remote;


unsigned long currentMillis;
//unsigned long previousMillis;

unsigned long previousMillis = 0;
const long interval = 20;

unsigned long previousBtMillis=0;

unsigned long previousServoMillis=0;
const long servoInterval = 200;

long previousSafetyMillis;

//SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);

//----------------------setup-------------------------------------------
void setup() {
  // Open serial communications and wait for port to open:
  //Serial.begin(115200);
  //Serial.begin( 19200);
  //Serial.begin(  9600);
  Serial.begin(Baud, SERIAL_8N1);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  #ifdef DEBUG 
    Serial.println(" ");
    Serial.print("Sketch:   ");   Serial.println(__FILE__);
    Serial.print("Uploaded: ");   Serial.println(__DATE__);
  #endif

  #ifdef DEBUG 
    Serial.println("RobotArm_servoSweep  Started--------------------------------------------------------");
  #endif
  #ifdef DEBUG 
    Serial.println(); 
  #endif

  if(BT_ON ==1) {
    #ifdef DEBUG 
      Serial.println(" setup:  Bluetooth is on");
    #endif
    bluetoothFactory.begin();
    bluetoothFactory.BT_to_serial_prepare();
  } else {
    Serial.println(" setup: Bluetooth is off");
  }

  //configure pin 2 as an input and enable the internal pull-up resistor
  pinMode(53, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  servosManager.begin();
  
  inverseKinematics.begin(SERVO_MIN_MILISEC, SERVO_MAX_MILISEC);
  Serial.println("setup: Setup DONE. System is ready to Initialize servos. Press Button to start...");

}
//----------------------end of setup-----------------------------------




//-----------------loop-------------------------------------------------
void loop() {
  
  unsigned long currentMillis = millis();

  if(BT_ON ==1) {
    if ((currentMillis - previousBtMillis) >= interval) {  
      bluetoothOutputData = bluetoothFactory.BT_loop(currentMillis);

      runRobotArm = bluetoothDataHandler_by_loop(bluetoothOutputData);

      previousBtMillis = currentMillis;
    }
  }


  boolean Btn2Clicked = buttons.ButtonHandle(53);
  if(Btn2Clicked == true) {
    runRobotArm = buttons.onButton2Clicked(runRobotArm) ;
  }// end of if (Btn2Clicked)
  
  if(runRobotArm == 1) {
    //------------------------------------
    servo_initialization_by_loop();
  }

  if(runRobotArm == 2) {
      if(initializationDone == 1) {
        if(BT_ON ==1) {
        //--------------------------------------
        //--------------------
        } else {
        //-------------
          runRobotArm = sequence_movement_by_loop();
        //-------------
        }  //end of else BT_ON
      }      
  }  
}// ----------end of loop-------------------------------------------

int bluetoothDataHandler_by_loop(BluetoothOutputData bluetoothOutputData){
      if(bluetoothOutputData.dataReceived) {
        incrementsFromBtController.index_finger_knuckle_right = bluetoothOutputData.index_finger_knuckle_right - 512;
        incrementsFromBtController.pinky_knuckle_right = bluetoothOutputData.pinky_knuckle_right - 512;
        incrementsFromBtController.index_finger_fingertip = bluetoothOutputData.index_finger_fingertip - 512;
        incrementsFromBtController.index_finger_knuckle_left = bluetoothOutputData.index_finger_knuckle_left - 512;

        if(bluetoothOutputData.Select) {
          #ifdef DEBUG 
		        Serial.println("bluetoothDataHandler_by_loop: Servo Initialization by bluetoothOutputData.Select . bluetoothOutputData.Select="+String(bluetoothOutputData.Select));
          #endif
          runRobotArm = buttons.onButton2Clicked(runRobotArm) ;
        }
      }
  return runRobotArm;
}

//-------------------bluetooth_movement_by_loop---------------------
void bluetooth_movement_by_loop(){
  if(partialMovementIsDone==true) {
    //----------------------------------
    //----------------------------------
    targetGripPosition.gripX = targetGripPosition.gripX + (incrementsFromBtController.index_finger_knuckle_right/50);
    targetGripPosition.gripY = targetGripPosition.gripY + (incrementsFromBtController.pinky_knuckle_right/50);
    targetGripPosition.gripZ = targetGripPosition.gripZ + (incrementsFromBtController.index_finger_fingertip/50);
    //----------------------------------
    //----------------------------------
    partialMovementIsDone = false;
        linearRampXYZ.begin(currentGripPosition, targetGripPosition);
        linearRampXYZ.setup();

        currentArmAngles = inverseKinematics.moveToPosXYZ(currentGripPosition);

  } else {
    currentGripPosition = linearRampXYZ.update();
    currentArmAngles = inverseKinematics.moveToPosXYZ(currentGripPosition);
    #ifdef DEBUG 
      Serial.println("bluetooth_movement_by_loop: BT after linearRampXYZ.update() currentGripPosition = {"+ String(currentGripPosition.gripX)+", "+ String(currentGripPosition.gripY)+", "+ String(currentGripPosition.gripZ)+", "+ String(currentGripPosition.gripSpinAngle)+", "+ String(currentGripPosition.gripTiltAngle)+", "+ String(currentGripPosition.gripWidth)+","+ String(currentGripPosition.movesScriptEnd)+"}.");
    #endif

    if(linearRampXYZ.rampOnce.rampIsFinished) {
      Serial.println("bluetooth_movement_by_loop: linearRampXYZ.rampOnce.rampIsFinished. partialMovementIsDone = true");
      partialMovementIsDone = true;
    }            
  }
}
//-------------------end of bluetooth_movement_by_loop---------------------

//----------------------sequence_movement_by_loop--------------------------
int sequence_movement_by_loop(){
  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed loop
    previousMillis = currentMillis;
    
    if(partialMovementIsDone==true) {
      #ifdef DEBUG 
        Serial.println("sequence_movement_by_loop: partialMovementIsDone==true, starting  roboArmTurn.takeNextRoboArmPosition()");
      #endif
      targetGripPosition = roboArmTurn.takeNextRoboArmPosition(); //read next target-position from 'moving script'       
      if(!targetGripPosition.movesScriptEnd) {
        partialMovementIsDone = false;
        
        #ifdef DEBUG 
          Serial.println("sequence_movement_by_loop: before linearRampXYZ.begin(...) currentGripPosition = {"+ String(currentGripPosition.gripX)+", "+ String(currentGripPosition.gripY)+", "+ String(currentGripPosition.gripZ)+", "+ String(currentGripPosition.gripSpinAngle)+", "+ String(currentGripPosition.gripTiltAngle)+", "+ String(currentGripPosition.gripWidth)+","+ String(currentGripPosition.movesScriptEnd)+"}.");
        #endif
        linearRampXYZ.begin(currentGripPosition, targetGripPosition);
        linearRampXYZ.setup();

        currentArmAngles = inverseKinematics.moveToPosXYZ(currentGripPosition);
      } else {
        Serial.println("sequence_movement_by_loop: movesScriptEnd!!!");
        runRobotArm = runRobotArm - 1;  // = 2
        targetGripPosition = roboArmTurn.resetMovescript();
        
      }
      
    } 
    else 
    {
      currentGripPosition = linearRampXYZ.update();
      currentArmAngles = inverseKinematics.moveToPosXYZ(currentGripPosition);
      #ifdef DEBUG 
        Serial.println("sequence_movement_by_loop: after linearRampXYZ.update() currentGripPosition = {"+ String(currentGripPosition.gripX)+", "+ String(currentGripPosition.gripY)+", "+ String(currentGripPosition.gripZ)+", "+ String(currentGripPosition.gripSpinAngle)+", "+ String(currentGripPosition.gripTiltAngle)+", "+ String(currentGripPosition.gripWidth)+","+ String(currentGripPosition.movesScriptEnd)+"}.");
      #endif

      if(linearRampXYZ.rampOnce.rampIsFinished) {
        Serial.println("sequence_movement_by_loop: linearRampXYZ.rampOnce.rampIsFinished. partialMovementIsDone = true");
        partialMovementIsDone = true;
      }
    }
    if(!targetGripPosition.movesScriptEnd) {
      
      servosManager.updateServos(currentArmAngles);   // send pulses to servos.  update servos according to InverseKinematics Values
      //delay(10);       // lazy way to limit update to 100 Hz
    } else {
      #ifdef DEBUG 
        Serial.println("sequence_movement_by_loop: delay(100)  @ End.");
      #endif
      //delay(100);
    }
  } // end of if (currentMillis - previousMillis >= 10
  return runRobotArm;
}
//-------------------end of sequence_movement_by_loop----------------------------

//---------------------servo_initialization_by_loop-----------------------------
void servo_initialization_by_loop() {
  if (initializationDone == 0) {
    #ifdef DEBUG 
      Serial.println("servo_initialization_by_loop: Servo Initialization. runRobotArm == 0");
    #endif
    startArmAngles = servosManager.ServoInitialization(servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos,SERVO_MIN_MILISEC ,SERVO_MAX_MILISEC );
    currentGripPosition = inverseKinematics.convertAngleToPosXYZ(startArmAngles);
    #if defined(DEBUG) || defined(BRIEF_LOG)
      Serial.println("servo_initialization_by_loop():  currentGripPosition (X,Y,Z) = ("+String(currentGripPosition.gripX)+", "+String(currentGripPosition.gripY)+", "+String(currentGripPosition.gripZ)+" ), Angles (Spin, Tilt, Open) = ("+String(currentGripPosition.gripSpinAngle) +", "+String(currentGripPosition.gripTiltAngle)+", "+String(currentGripPosition.gripWidth)+"), duration="+String(currentGripPosition.duration)+", movesScriptEnd = "+String(currentGripPosition.movesScriptEnd));     
    #endif
    currentArmAngles = inverseKinematics.moveToPosXYZ(currentGripPosition);
    servosManager.updateServos(currentArmAngles);   // send pulses to servos.  update servos according to InverseKinematics Values
    initializationDone = 1;
    delay(100);+
    Serial.println("servo_initialization_by_loop: Servo Initialization  DONE. Press Button to start...-----------------------------------------");
  }
}
//------------------end of servo_initialization_by_loop--------------------------
