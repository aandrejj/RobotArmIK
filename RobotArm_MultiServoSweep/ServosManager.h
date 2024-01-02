#ifndef ServosManager_h
#define ServosManager_h

#include "Arduino.h"
#include <Servo.h>
//#define DEBUG         //extensive logging

typedef struct {
  double gripX;
  double gripY;
  double gripZ;
  double gripSpinAngle;
  double gripTiltAngle;
  double gripOpen;
  bool movesScriptEnd;
}GripPositionXYZ;

typedef struct {
  int baseAngle;
  int arm1Angle;
  int arm2Angle;
  int gripSpinAngle;
  int gripTiltAngle;
  int gripAngle;
  bool movesScriptEnd;
} ArmServoAngles;

typedef struct {
  int arm1Microsec;
  int arm2Microsec;
  int baseMicrosec;
  int griperSpinMicrosec;
  int griperTiltMicrosec;
  int griperMicrosec;
  bool movesScriptEnd;
} ArmServoMicrosec;




class ServosManager{

public:
	Servo servo01; //zakladna
	Servo servo02; //spodne hnede rameno
	Servo servo03; //horne  biela rameno
	Servo servo04; //ruka nabok  100 = zhruba vodorovne
	Servo servo05; //ruka hore
	Servo servo06; //ruka otvorena= 100, zatvorena = 60

	ServosManager();

  void updateServos_msec(ArmServoMicrosec armServoMicrosec);
  void updateServos(ArmServoAngles armServoAngles);
	
	ArmServoAngles updateCurrentAngles(ArmServoAngles oldServoAngles);
	
	ArmServoAngles ServoInitialization(int pservo1Pos, int pservo2Pos, int pservo3Pos, int pservo4Pos, int pservo5Pos, int pservo6Pos, int pServo_Min_milisec, int pServo_Max_milisec );

private:

};
#endif
