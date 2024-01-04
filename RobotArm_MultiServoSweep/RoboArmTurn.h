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
#define BRIEF_LOG     //just a few logs

class RoboArmTurn {
  public:
	      //                grip  grip  grip
		  //{XXX, YYY, ZZZ, tilt, spin,  open }
    int movesScriptSize = 4;
		int movesScript[4][7] = {
			 {  0,   0, 200,    0,    0, 80, 3000}
			,{  0,   0, 200,    0,    0, 60, 3000}
      ,{  0,   0, 200,    0,    0, 80, 3000}
			,{ 10,  10, 190,    0,    0, 80, 3000}
//			,{ 10,  10, 190,    0,    0, 80, 3000}
//			,{100, 100,  10,    0,    0, 20, 3000}
//			,{100, 100,  20,    0,    0, 20, 3000}
//			,{ 80,  80,  20,    0,    0, 20, 3000}
//			,{ 80,  80,  10,    0,    0, 20, 3000}
//			,{ 80,  80,  10,    0,    0, 50, 3000}
		};
		int movesScriptIndex = 0;
		bool movesScriptEnd = false;

		//int actualTargetPositionXYZ[6];
		
		Servo servo01; //zakladna
		Servo servo02; //spodne hnede rameno
		Servo servo03; //horne  biela rameno
		Servo servo04; //ruka nabok  100 = zhruba vodorovne
		Servo servo05; //ruka hore
		Servo servo06; //ruka otvorena= 100, zatvorena = 60
		
		int servo1PPos = 90; //zakladna
		int servo2PPos = 100;//spodne hnede rameno
		int servo3PPos = 35; //horne  biela rameno
		int servo4PPos = 100;//ruka nabok  100 = zhruba vodorovne
		int servo5PPos = 85; //ruka hore
		int servo6PPos = 80; //ruka otvorena= 100, zatvorena = 60
		
		RoboArmTurn();
		//GripPositionXYZ updateRoboArmPositionsByScript();
		GripPositionXYZ takeNextRoboArmPosition();
		
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
