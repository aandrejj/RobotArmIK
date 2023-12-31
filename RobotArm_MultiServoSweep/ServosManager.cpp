#include "Arduino.h"
#include "ServosManager.h"
#include <SoftwareSerial.h>


	ServosManager::ServosManager() {
	}
	//--------------------updateServos()--------------------------------
	//ToDo Here
	void ServosManager::updateServos(ArmServoMicrosec armServoMicrosec) {
	  //for (int servoNumber = 1; servoNumber <= 6; servoNumber++) {
		//rampServos[servoNumber].updateServo();
	  //}
	  //ToDo Here
	  servo01.writeMicroseconds(armServoMicrosec.baseMicrosec);
	  servo02.writeMicroseconds(armServoMicrosec.arm1Microsec);
	  servo03.writeMicroseconds(armServoMicrosec.arm2Microsec);
	  servo04.writeMicroseconds(armServoMicrosec.griperSpinMicrosec);
	  servo05.writeMicroseconds(armServoMicrosec.griperTiltMicrosec);
	  servo06.writeMicroseconds(armServoMicrosec.griperMicrosec);
	  
	}
	 //--------------------end of updateServo-----------------------------

	ArmServoAngles ServosManager::updateCurrentAngles(ArmServoAngles oldServoAngles) {
		Serial.println("updateCurrentAngles: Started");
		ArmServoAngles newServoAngles;

		
		return newServoAngles;
	}

	//---------------SevoInitialization----------------------------------
	ArmServoAngles ServosManager::ServoInitialization(int pservo1Pos, int pservo2Pos, int pservo3Pos, int pservo4Pos, int pservo5Pos, int pservo6Pos ) {
	  Serial.println("ServosManager::ServoInitialization: Initialization started");
	  delay(201);
	  servo01.attach( 5, 460 ,2400); //servo01.attach( 5,460 ,2400); 
	  servo02.attach( 6, 460 ,2400);
	  servo03.attach( 7, 460 ,2400);
	  servo04.attach( 8, 460 ,2400);
	  servo05.attach( 9, 460 ,2400);
	  servo06.attach(10, 460 ,2400);
	  Serial.println("ServosManager::ServoInitialization: Servos attached");
	  delay(20);
	  Serial.println("ServosManager::ServoInitialization:  Robot arm initial position");

	  //servo1PPos = pservo1Pos; //zakladna
	  //servo2PPos = pservo2Pos; //spodne hnede rameno
	  //servo3PPos = pservo3Pos; //horne  biela rameno
	  //servo4PPos = pservo4Pos; //ruka nabok  100 = zhruba vodorovne
	  //servo5PPos = pservo5Pos; //ruka hore
	  //servo6PPos = pservo6Pos; //ruka otvorena= 100, zatvorena = 60
	  
	  servo01.write(pservo1Pos);
	  //Serial.println("Servo01 positioned");
	  //delay(100);

	  servo02.write(pservo2Pos);
	  //Serial.println("Servo02 positioned");
	  //delay(100);
	  
	  servo03.write(pservo3Pos);
	  //Serial.println("Servo03 positioned");
	  //delay(100);
	  
	  servo04.write(pservo4Pos);
	  //Serial.println("Servo04 positioned");
	  //delay(100);
	  
	  servo05.write(pservo5Pos);
	  //Serial.println("Servo05 positioned");
	  //delay(100);
	  
	  servo06.write(pservo6Pos);
	  //Serial.println("Servo06 positioned");
	  //delay(100);
	  
	  Serial.println("ServosManager::ServoInitialization: Servos positioned to default positions");
	  
	  ArmServoAngles currentServoAngles;
	  
	  currentServoAngles.baseAngle      = pservo1Pos; //zakladna
	  currentServoAngles.arm1Angle      = pservo2Pos; //spodne hnede rameno
	  currentServoAngles.arm2Angle      = pservo3Pos; //horne  biela rameno
	  currentServoAngles.gripSpinAngle  = pservo4Pos; //ruka nabok
	  currentServoAngles.gripTiltAngle  = pservo5Pos; //ruka hore
	  currentServoAngles.gripAngle      = pservo6Pos; //ruka otvorena
	  currentServoAngles.movesScriptEnd = false;

	  // ToDo : Remove this (down)
	  //RoboArmTurn::servoCurrentPos[1] = pservo1Pos;
	  //RoboArmTurn::servoCurrentPos[2] = pservo2Pos;
	  //RoboArmTurn::servoCurrentPos[3] = pservo3Pos;
	  //RoboArmTurn::servoCurrentPos[4] = pservo4Pos;
	  //RoboArmTurn::servoCurrentPos[5] = pservo5Pos;
	  //RoboArmTurn::servoCurrentPos[6] = pservo6Pos;
	  
	  Serial.println("ServosManager::ServoInitialization: Initialization OK.");
	  return currentServoAngles;
	}

