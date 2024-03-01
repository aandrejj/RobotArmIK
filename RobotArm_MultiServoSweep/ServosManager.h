#ifndef ServosManager_h
#define ServosManager_h

#include "Arduino.h"
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

#define DEBUG         //debug logging
#define BRIEF_LOG     //just a few logs

typedef struct {
  double gripX;
  double gripY;
  double gripZ;
  double gripSpinAngle;
  double gripTiltAngle;
  double gripWidth;
  double duration;
  bool movesScriptEnd;
}GripPositionXYZ;

typedef struct {
  double baseAngle;
  double arm1Angle;
  double arm2Angle;
  double gripSpinAngle;
  double gripTiltAngle;
  double gripAngle;
  double duration;
  bool movesScriptEnd;
} ArmServoAngles;

typedef struct {
  int arm1Microsec;
  int arm2Microsec;
  int baseMicrosec;
  int gripSpinMicrosec;
  int gripTiltMicrosec;
  int gripMicrosec;
  double duration;
  bool movesScriptEnd;
} ArmServoMicrosec;




class ServosManager{

public:
	//Servo servo01; //zakladna
	//Servo servo02; //spodne hnede rameno
	//Servo servo03; //horne  biela rameno
	//Servo servo04; //ruka nabok  100 = zhruba vodorovne
	//Servo servo05; //ruka hore
	//Servo servo06; //ruka otvorena= 100, zatvorena = 60

  ArmServoAngles previousArmServoAngles;

	ServosManager();
  
  void begin();

  //void updateServos_msec(ArmServoMicrosec armServoMicrosec);
  void updateServos(ArmServoAngles armServoAngles);
	
	ArmServoAngles updateCurrentAngles(ArmServoAngles oldServoAngles);
	
	ArmServoAngles ServoInitialization(int pservo1Pos, int pservo2Pos, int pservo3Pos, int pservo4Pos, int pservo5Pos, int pservo6Pos, int pServo_Min_milisec, int pServo_Max_milisec );

private:

};
#endif
