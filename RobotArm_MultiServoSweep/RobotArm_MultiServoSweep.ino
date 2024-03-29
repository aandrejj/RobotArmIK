/*        
       DIY Arduino Robot Arm 
       https://www.youtube.com/watch?v=_B3gWd3A_SI

*/

//
//
// NEXT is https://github.com/siteswapjuggler/RAMP
//         https://github.com/siteswapjuggler/RAMP/blob/master/examples/ramp_servo/ramp_servo.ino
//        
//         https://github.com/roTechnic/InverseKinematicArm
//         https://github.com/roTechnic/InverseKinematicArm/blob/main/inverse_kinematics.ino  <-origin of method moveToPosXYZ  is here as moveToPos
//  Easy  Inverse kinematics for robot armServoAngles
//    https://www.youtube.com/watch?v=Q-UeYEpwXXU     <-  pictures from here 
//
//  https://github.com/aandrejj/RobotArmIK/pulls

/*
RemoteController by James Bruton
Building Robot X #10  
        https://www.youtube.com/watch?v=B2bUlDf4aNE
        https://github.com/XRobots/RobotX/tree/master/CAD

Open Dog  Part1
  https://github.com/XRobots/openDog/tree/master/Part1/CAD

*/

#include "ServosManager.h"
#include "InverseKinematics.h"
#include "Buttons.h"
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

#define MIN_X -240
#define MAX_X  240
#define MIN_Y -240
#define MAX_Y  240
#define MIN_Z -110
#define MAX_Z  240
#define MIN_W -40
#define MAX_W  50

#define buttonStepX 1.0;
#define buttonStepY 1.0;
#define buttonStepZ 5.0;

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

int state;
int previous_state;
#define STATE 11
/*
#define BLUETOOTH_RX 9  // Bluetooth RX -> Arduino D9
#define BLUETOOTH_TX 10 // Bluetooth TX -> Arduino D10
//#define GND 13
//#define Vcc 12
#define ENABLE 8
*/


                                          // ..so that servos often respond to values between 700 and 2300.
#define SPEED         30.0                // not a [m/s] but [mm/s]

//#define DEBUG         //debug logging
//#define BRIEF_LOG     //just a few logs
//#define MOVEMENT_LOG

Buttons buttons;
InverseKinematics inverseKinematics;
RoboArmTurn roboArmTurn;
ServosManager servosManager;

BluetoothOutputData bluetoothOutputData;
BluetoothOutputData balancedBluetoothOutputData;
BluetoothOutputData previousBluetoothOutputData;
BluetoothOutputData changesFromBtController;


BluetoothFactory bluetoothFactory;

LinearRampXYZ linearRampXYZ;

GripPositionXYZ previousGripPosition;
GripPositionXYZ currentGripPosition;
GripPositionXYZ targetGripPosition;

ArmServoAngles startArmAngles;
ArmServoAngles currentArmAngles;

ArmServoMicrosec currentArmMicrosec;


int speedDelay = 50;
int index = 0;
int runRobotArm = 0;
int previous_runRobotArm = 0;
int initializationDone = 0;
boolean previousSensorVal = LOW;
bool partialMovementIsDone =  true;//false;

ArmServoAngles servosInitialPosition;
int servo1PPos = 85; //zakladna
int servo2PPos = 50;//160;//spodne hnede rameno
int servo3PPos = 13; //horne  biela rameno
int servo4PPos = 0;//ruka nabok  100 = zhruba vodorovne
int servo5PPos = 0; //ruka hore
int servo6PPos = 110; //ruka otvorena= 100, zatvorena = 60

unsigned long currentMillis;
//unsigned long previousMillis;

unsigned long previousMillis = 0;
const long interval = 20;

unsigned long previousBtMillis=0;

unsigned long previousServoMillis=0;
const long servoInterval = 200;

long previousSafetyMillis;

bool bt_State_Connected = false;
bool previous_bt_state = false;
bool newDataReceived;


unsigned long milisOfLastStateChanged;
unsigned long maxTimeOfNoChangeMillis = 1500;
unsigned long  currentStateDuration;
bool LedIsBlinking = true;
bool BtLedIsSteadyOn = false;

float xIncrement;
float yIncrement;
float zIncrement;
float wIncrement;

//----------------------setup-------------------------------------------
void setup() {
  // Open serial communications and wait for port to open:
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

  currentGripPosition.showLog = false;

  inverseKinematics.begin(SERVO_MIN_MILISEC, SERVO_MAX_MILISEC);
  Serial.println("setup: Setup DONE. System is ready to Initialize servos. Press Button to start...");

}
//----------------------end of setup-----------------------------------

//-----------------loop-------------------------------------------------
void loop() {
  
  unsigned long currentMillis = millis();

  if(BT_ON ==1) {
    if ((currentMillis - previousBtMillis) >= interval) 
    {
      if( (!bt_State_Connected) || (!previous_bt_state) )
      {
        loop_Handling_formBluetoothConnecting(currentMillis);
      }

      if(bt_State_Connected)
      {
        bluetoothOutputData = bluetoothFactory.BT_loop(currentMillis);

        runRobotArm = bluetoothButtonHandler_by_loop(bluetoothOutputData);

      }
      //previousBtMillis = currentMillis;
    }
  }

  if(runRobotArm!=2) {
    boolean Btn2Clicked = buttons.ButtonHandle(53);
    if(Btn2Clicked == true) {
      runRobotArm = buttons.onButton2Clicked(runRobotArm) ;
    }// end of if (Btn2Clicked)
  }

  if(previous_runRobotArm!= runRobotArm) {
    #if defined(DEBUG) || defined(BRIEF_LOG)
    Serial.println("loop: runRobotArm changed from value "+String(previous_runRobotArm)+" to value "+String(runRobotArm)+".");
    #endif
  }
  previous_runRobotArm = runRobotArm;
  
  switch (runRobotArm) {
    case 0:
      //------------------------------------
      #if defined(DEBUG)
        Serial.println("loop: runRobotArm =0.");
      #endif
      break;
      
    case 1:
      //------------------------------------
      servo_initialization_by_loop();
      break;
      
    case 2:
      
      if(initializationDone == 1) {
        if(BT_ON ==1) {
          //--------------------------------------
          if ((currentMillis - previousBtMillis) >= interval) 
          {
            bluetoothDataHandler_by_loop(bluetoothOutputData);
          
            #if defined(DEBUG) // || defined(BRIEF_LOG)
              Serial.println("loop: balancedBluetoothOutputData= ("+String(balancedBluetoothOutputData.index_finger_knuckle_right)+", "+String(balancedBluetoothOutputData.pinky_knuckle_right)+", "+String(balancedBluetoothOutputData.index_finger_fingertip)+", "+String(balancedBluetoothOutputData.index_finger_knuckle_left)+")");
            #endif

            previousBluetoothOutputData = bluetoothOutputData;

            bluetooth_movement_by_loop();

            previousBtMillis = currentMillis;
          }
          //--------------------
        } else {
        //-------------
          runRobotArm = sequence_movement_by_loop();
        //-------------
        }  //end of else BT_ON
      }
      break;

    case 3:
      //------------------------------------
      #if defined(DEBUG)
        Serial.println("loop: runRobotArm =3.");
      #endif
      break;
      
      
    default:
      Serial.println("loop: WARNING:   switch(runRobotArm) -> default: runRobotArm = "+String(runRobotArm)+"!!!");
      break;
      
  } // end of switch
}// ----------end of loop-------------------------------------------

BluetoothOutputData bluetoothDataHandler_by_loop(BluetoothOutputData bluetoothOutputData){
      if(bluetoothOutputData.dataReceived) {
        balancedBluetoothOutputData.dataReceived = bluetoothOutputData.dataReceived;
        balancedBluetoothOutputData.menuUp  = bluetoothOutputData.menuUp;

        balancedBluetoothOutputData.stick1_X = bluetoothOutputData.stick1_X - 518;
        balancedBluetoothOutputData.stick1_Y = bluetoothOutputData.stick1_Y - 516;
        balancedBluetoothOutputData.stick2_X = bluetoothOutputData.stick2_X - 516;
        balancedBluetoothOutputData.stick2_Y = bluetoothOutputData.stick2_Y - 514;

        balancedBluetoothOutputData.stick1_X = zero_value_stabiliser(balancedBluetoothOutputData.stick1_X);
        balancedBluetoothOutputData.stick1_Y = zero_value_stabiliser(balancedBluetoothOutputData.stick1_Y);
        balancedBluetoothOutputData.stick2_X = zero_value_stabiliser(balancedBluetoothOutputData.stick2_X);
        balancedBluetoothOutputData.stick2_Y = zero_value_stabiliser(balancedBluetoothOutputData.stick2_Y);

        #if defined(DEBUG) || defined(BRIEF_LOG) || defined(MOVEMENT_LOG)
          //Serial.print("bluetoothDataHandler_by_loop: NewDataReceived balancedBluetoothOutputData = ");
          //Serial.print("{"+ String(balancedBluetoothOutputData.stick1_X)+", "+ String(balancedBluetoothOutputData.stick1_Y)+", "+ String(balancedBluetoothOutputData.stick2_X)+", "+ String(balancedBluetoothOutputData.stick2_Y)+"}.");
        #endif
      } else {
        bluetoothOutputData.dataReceived = false;
      }
  return balancedBluetoothOutputData;
}

int zero_value_stabiliser(int inputValue) {
  if(inputValue > -10 && inputValue <10) {
    inputValue =0;
  }
  return inputValue;
}

int bluetoothButtonHandler_by_loop(BluetoothOutputData bluetoothOutputData){
      if(bluetoothOutputData.dataReceived) {
        if(bluetoothOutputData.Select) {
          #if defined(DEBUG) || defined(BRIEF_LOG)
		        Serial.println("bluetoothDataHandler_by_loop: Servo Initialization by bluetoothOutputData.Select . bluetoothOutputData.Select="+String(bluetoothOutputData.Select));
          #endif
          runRobotArm = buttons.onButton2Clicked(runRobotArm) ;
        }
        if(bluetoothOutputData.menuUp) {
          if (initializationDone == 1) {
          #if defined(DEBUG) || defined(BRIEF_LOG)
            Serial.println("bluetoothDataHandler_by_loop: Reset Position to initial");
          #endif  
          goTo_initialization_position();
          } else {
            Serial.println("bluetoothDataHandler_by_loop: SKIP SKIP. GoTo_initialization_position requested, but hand is not initialized!!!  Initialize first.");
          }
        }
      }
  return runRobotArm;
}

GripPositionXYZ addButtonsToPosition(GripPositionXYZ _currentGripPosition, BluetoothOutputData bluetoothOutputData, BluetoothOutputData previousBluetoothOutputData){
  bool changed = false;
  if(bluetoothOutputData.navKeyMiddle == 0) {
    if(bluetoothOutputData.navKeyUp == 0 && previousBluetoothOutputData.navKeyUp == 1 ) {
      //x -
      _currentGripPosition.gripX = _currentGripPosition.gripX - buttonStepX;
      changed =true;
    }
    if(bluetoothOutputData.navKeyDown == 0 && previousBluetoothOutputData.navKeyDown == 1 ) {
      //x +
      _currentGripPosition.gripX = _currentGripPosition.gripX + buttonStepX;
      changed =true;
    }
    if(bluetoothOutputData.navKeyLeft == 0 && previousBluetoothOutputData.navKeyLeft == 1 ) {
      //y-
      _currentGripPosition.gripY = _currentGripPosition.gripY - buttonStepY;
      changed =true;
    }
    if(bluetoothOutputData.navKeyRight == 0 && previousBluetoothOutputData.navKeyRight == 1 ) {
      //y+
      _currentGripPosition.gripY = _currentGripPosition.gripY + buttonStepY;
      changed =true;
    }
  } else {
    if(bluetoothOutputData.navKeyLeft == 0 && previousBluetoothOutputData.navKeyLeft == 1 ) {
      //Z-
      _currentGripPosition.gripZ = _currentGripPosition.gripZ - buttonStepZ;
      changed =true;
    }
    if(bluetoothOutputData.navKeyRight == 0 && previousBluetoothOutputData.navKeyRight == 1 ) {
      //Z+
      _currentGripPosition.gripZ = _currentGripPosition.gripZ + buttonStepZ;
      changed =true;
    }
  }

  if(bluetoothOutputData.navKeySet == 0 && previousBluetoothOutputData.navKeySet == 1 ) {
    //Z-
    _currentGripPosition.gripZ = _currentGripPosition.gripZ - buttonStepZ;
    changed =true;
  }
  if(bluetoothOutputData.navKeyReset == 0 && previousBluetoothOutputData.navKeyReset == 1 ) {
    //Z+
    _currentGripPosition.gripZ = _currentGripPosition.gripZ + buttonStepZ;
    changed =true;
  }

    #if defined(DEBUG)  || defined(BRIEF_LOG) || defined(MOVEMENT_LOG)
    if(changed==true) {

      _currentGripPosition.gripX     = constrain(_currentGripPosition.gripX,     MIN_X, MAX_X);
      _currentGripPosition.gripY     = constrain(_currentGripPosition.gripY,     MIN_Y, MAX_Y);
      _currentGripPosition.gripZ     = constrain(_currentGripPosition.gripZ,     MIN_Z, MAX_Z);
      _currentGripPosition.gripWidth = constrain(_currentGripPosition.gripWidth, MIN_W, MAX_W);
      
      //_currentGripPosition.showLog = true;
      Serial.println("addButtonsToPosition: currentGripPosition = {"+ String(_currentGripPosition.gripX)+", "+ String(_currentGripPosition.gripY)+", "+ String(_currentGripPosition.gripZ)+", "+ String(_currentGripPosition.gripSpinAngle)+", "+ String(_currentGripPosition.gripTiltAngle)+", "+ String(_currentGripPosition.gripWidth)+"}.");
    }
    #endif

  return _currentGripPosition;
}

#define positionDivider 80
GripPositionXYZ addMovementsToPosition(GripPositionXYZ _currentGripPosition, BluetoothOutputData balancedBluetoothOutputData){

  xIncrement = balancedBluetoothOutputData.stick1_X / positionDivider;
  yIncrement = balancedBluetoothOutputData.stick1_Y / positionDivider;
  zIncrement = balancedBluetoothOutputData.stick2_Y / positionDivider;
  wIncrement = balancedBluetoothOutputData.stick2_X / positionDivider;

  _currentGripPosition.gripX     = constrain((_currentGripPosition.gripX     + xIncrement), MIN_X, MAX_X);
  _currentGripPosition.gripY     = constrain((_currentGripPosition.gripY     + yIncrement), MIN_Y, MAX_Y);
  _currentGripPosition.gripZ     = constrain((_currentGripPosition.gripZ     + zIncrement), MIN_Z, MAX_Z);
  _currentGripPosition.gripWidth = constrain((_currentGripPosition.gripWidth + wIncrement), MIN_W, MAX_W);

  //_currentGripPosition.showLog = true;

  #if defined(DEBUG)  || defined(BRIEF_LOG) || defined(MOVEMENT_LOG)
    if(xIncrement!= 0  || yIncrement != 0 || zIncrement != 0 || wIncrement != 0) {
      //_currentGripPosition.showLog = true;
      Serial.println("addMovementsToPosition: currentGripPosition = {"+ String(_currentGripPosition.gripX)+", "+ String(_currentGripPosition.gripY)+", "+ String(_currentGripPosition.gripZ)+", "+ String(_currentGripPosition.gripSpinAngle)+", "+ String(_currentGripPosition.gripTiltAngle)+", "+ String(_currentGripPosition.gripWidth)+"}.");
    }
  #endif

  return _currentGripPosition;
}

//-------------------bluetooth_movement_by_loop---------------------
void bluetooth_movement_by_loop(){
  if(bluetoothOutputData.dataReceived) {
    #if defined(DEBUG) || defined(BRIEF_LOG) || defined(MOVEMENT_LOG)
      //Serial.print("bluetooth_movement_by_loop: after linearRampXYZ.update() OLD currentGripPosition = {"+ String(currentGripPosition.gripX)+", "+ String(currentGripPosition.gripY)+", "+ String(currentGripPosition.gripZ)+", "+ String(currentGripPosition.gripSpinAngle)+", "+ String(currentGripPosition.gripTiltAngle)+", "+ String(currentGripPosition.gripWidth)+","+ String(currentGripPosition.movesScriptEnd)+"}.");
    #endif

    currentGripPosition = addButtonsToPosition(currentGripPosition, balancedBluetoothOutputData, previousBluetoothOutputData);
    currentGripPosition = addMovementsToPosition(currentGripPosition, balancedBluetoothOutputData);

    if(currentGripPosition.errorOutOfWorkZone == true) {
      currentGripPosition = previousGripPosition;
      Serial.print("bluetooth_movement_by_loop: BackToPrevious position. | currentGripPosition = {"+ String(currentGripPosition.gripX)+", "+ String(currentGripPosition.gripY)+", "+ String(currentGripPosition.gripZ)+", "+ String(currentGripPosition.gripSpinAngle)+", "+ String(currentGripPosition.gripTiltAngle)+", "+ String(currentGripPosition.gripWidth)+"}.");
    }

    #if defined(DEBUG)  || defined(BRIEF_LOG) || defined(MOVEMENT_LOG)
      //Serial.println(" |  NEW currentGripPosition = {"+ String(currentGripPosition.gripX)+", "+ String(currentGripPosition.gripY)+", "+ String(currentGripPosition.gripZ)+", "+ String(currentGripPosition.gripSpinAngle)+", "+ String(currentGripPosition.gripTiltAngle)+", "+ String(currentGripPosition.gripWidth)+"}.");
    #endif

    currentArmAngles = inverseKinematics.moveToPosXYZ(currentGripPosition);

    if(currentArmAngles.errorOutOfWorkZone == true) {
      currentGripPosition = previousGripPosition;
      Serial.print("bluetooth_movement_by_loop: BackToPrevious position. | currentGripPosition = {"+ String(currentGripPosition.gripX)+", "+ String(currentGripPosition.gripY)+", "+ String(currentGripPosition.gripZ)+", "+ String(currentGripPosition.gripSpinAngle)+", "+ String(currentGripPosition.gripTiltAngle)+", "+ String(currentGripPosition.gripWidth)+"}.");
      
      currentArmAngles = inverseKinematics.moveToPosXYZ(currentGripPosition);
    }

    currentGripPosition.showLog = false;
    servosManager.updateServos(currentArmAngles);   // send pulses to servos.  update servos according to InverseKinematics Values      

    if(currentGripPosition.errorOutOfWorkZone == false) {
      previousGripPosition = currentGripPosition;
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
        ////currentGripPosition.showLog = false;
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
    //#if defined(DEBUG) || defined(BRIEF_LOG)
      Serial.println("servo_initialization_by_loop():  currentGripPosition (X,Y,Z) = ("+String(currentGripPosition.gripX)+", "+String(currentGripPosition.gripY)+", "+String(currentGripPosition.gripZ)+" ), Angles (Spin, Tilt, Open) = ("+String(currentGripPosition.gripSpinAngle) +", "+String(currentGripPosition.gripTiltAngle)+", "+String(currentGripPosition.gripWidth)+"), duration="+String(currentGripPosition.duration)+", movesScriptEnd = "+String(currentGripPosition.movesScriptEnd)+", errorOutOfWorkZone = "+String(currentGripPosition.errorOutOfWorkZone));
    //#endif
    currentGripPosition.showLog = true;
    currentArmAngles = inverseKinematics.moveToPosXYZ(currentGripPosition);
    servosManager.updateServos(currentArmAngles);   // send pulses to servos.  update servos according to InverseKinematics Values
    initializationDone = 1;
    delay(100);+
    Serial.println("servo_initialization_by_loop: Servo Initialization  DONE. (initializationDone =1) Press Button to start...-----------------------------------------");
  } else {
    Serial.println("servo_initialization_by_loop: SKIP. Allready  inintialised!!!! SKIP -----------------------------------------");
  }
}
//------------------end of servo_initialization_by_loop--------------------------

//------------------goTo_initialization_position--------------------------
void goTo_initialization_position() {
    startArmAngles = servosManager.ServoInitialization(servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos,SERVO_MIN_MILISEC ,SERVO_MAX_MILISEC );
    currentGripPosition = inverseKinematics.convertAngleToPosXYZ(startArmAngles);
    #if defined(DEBUG) || defined(BRIEF_LOG)
      Serial.println("goTo_initialization_position():  currentGripPosition (X,Y,Z) = ("+String(currentGripPosition.gripX)+", "+String(currentGripPosition.gripY)+", "+String(currentGripPosition.gripZ)+" ), Angles (Spin, Tilt, Open) = ("+String(currentGripPosition.gripSpinAngle) +", "+String(currentGripPosition.gripTiltAngle)+", "+String(currentGripPosition.gripWidth)+"), duration="+String(currentGripPosition.duration)+", movesScriptEnd = "+String(currentGripPosition.movesScriptEnd));     
    #endif
    currentArmAngles = inverseKinematics.moveToPosXYZ(currentGripPosition);
    servosManager.updateServos(currentArmAngles);   // send pulses to servos.  update servos according to InverseKinematics Values
  
}//------------------end of goTo_initialization_position--------------------------


//------------loop_Handling_formBluetoothConnecting----------------------------
void loop_Handling_formBluetoothConnecting(unsigned long currentMillis) {
  //bt_serial_async(currentMillis);
  if (check_bt_from_loop(currentMillis)==true) {
    Serial.println("Bluetooth connected");
  }
}
//--------end of loop_Handling_formBluetoothConnecting--------------------------

//---------------------check_bt_from_loop--------------------------
bool check_bt_from_loop(unsigned long currentMillis) {
  if(BT_ON ==1) {
      // check to see if BT is paired
      //state = bluetoothFactory.getBtState();
      state = digitalRead(STATE);
      
      previous_bt_state = bt_State_Connected;
      bt_State_Connected = Bt_state_checker(currentMillis, previous_state, state);

      previous_state = state;

      if (!bt_State_Connected) {
        if(previous_bt_state!= bt_State_Connected) {
          Serial.println("BT connecting...");
        }
      }
      else {
        if(previous_bt_state!= bt_State_Connected) {
          Serial.println("BT Paired to RemoteController");
        }
      }
    } else {
  }// end of if BT_ON ==1
  return bt_State_Connected;
}
//-----------------------end of   check_bt_from_loop---------------------------------

//-----------------------Bt_state_checker----------------------------------------------
bool Bt_state_checker(unsigned long currentMillis, bool previousState, bool newState) {
  if(previousState!=newState) {
    milisOfLastStateChanged = currentMillis;
    LedIsBlinking = true;
    BtLedIsSteadyOn = false;
  } else {
    currentStateDuration = currentMillis - milisOfLastStateChanged;
    if(currentStateDuration > maxTimeOfNoChangeMillis) {
      LedIsBlinking = false;
      if(newState) {
        BtLedIsSteadyOn = true;
      } else {
        BtLedIsSteadyOn = false;
      }
    } else {
      LedIsBlinking = true;
      BtLedIsSteadyOn = false;
    }
  }
  return BtLedIsSteadyOn;
}
//-----------------------Bt_state_checker----------------------------------------------

