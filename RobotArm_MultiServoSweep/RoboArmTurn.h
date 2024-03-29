/**
 * RoboArmTurn.h
 */
#ifndef RoboArmTurn_h
#define RoboArmTurn_h
#include "Arduino.h"
#include <Servo.h>
#include <ArduinoJson.h>  //https://arduinojson.org/   https://arduinojson.org/v6/api/jsonarray/
#include <SoftwareSerial.h>
#include "ServosManager.h"

//#define DEBUG         //debug logging
//#define BRIEF_LOG     //just a few logs

class RoboArmTurn {
  public:
	      //                grip  grip  grip
		  //{XXX, YYY, ZZZ, tilt, spin,  open }  
    int movesScriptSize = 5;
    int movesScript[5][7] = {
			 {  10,   0, 120,    0,    0, 60, 300}  // navrat na basePosition
			,{  10,   0, 200,    0,    0, 60, 2000}
      ,{  10,   0, 120,    0,    0, 60, 2000} // navrat na basePosition
      ,{  10,   0,  90,    0,    0, 60, 2000}
      ,{  10,   0, 120,    0,    0, 60, 2000} // navrat na basePosition
//      ,{  10,   5, 120,    0,    0, 60, 1000}
//      ,{  10,  -5, 120,    0,    0, 60, 2000}
//      ,{  10,   0, 120,    0,    0, 60, 1000} // navrat na basePosition
//      ,{  10,   0, 200,    0,    0, 60, 2000} 
//      ,{  10,   0, 200,    0,    0, 50, 2000}
//      ,{  10,   0, 200,    0,    0, 60, 2000}
//      ,{  10,   0, 120,    0,    0, 60, 2000} // navrat na basePosition

//			,{100, 100,  10,    0,    0, 20, 3000}
//			,{100, 100,  20,    0,    0, 20, 3000}
//			,{ 80,  80,  20,    0,    0, 20, 3000}
//			,{ 80,  80,  10,    0,    0, 20, 3000}
//			,{ 80,  80,  10,    0,    0, 50, 3000}
		};
		int movesScriptIndex = 0;
		bool movesScriptEnd = false;

    
		//int actualTargetPositionXYZ[6];

		RoboArmTurn();
		//GripPositionXYZ updateRoboArmPositionsByScript();
		GripPositionXYZ takeNextRoboArmPosition();
    GripPositionXYZ resetMovescript();
      
		void servoSweepJson(char moves[], int runRobotArm, String text );
		void mainRoboArmTurn_old();
		void servoSweep(int servoNumber,  int positionFrom, int positionTo, int moveDelay, String text );
		//void ServoInitialization();
		//ArmServoAngles ServoInitialization(int pservo1Pos, int pservo2Pos, int pservo3Pos, int pservo4Pos, int pservo5Pos, int pservo6Pos );
		

private:
		int servoCurrentPos[7];
		int runRobotArm = 0;
		
		void TurnGripSideBySide();
		void OpenCloseGrip();
		void TurnGripUpDown();
		void RoboArmSequence1();

};

#endif
