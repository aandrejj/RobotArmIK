#ifndef ServosManager_h
#define ServosManager_h

#include "Arduino.h"
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

//#define DEBUG         //debug logging
#define BRIEF_LOG     //just a few logs

//#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN1  130
#define SERVOMAX1  630

#define SERVOMIN2  180
#define SERVOMAX2  610

#define SERVOMIN3  120
#define SERVOMAX3  620

#define SERVOMIN4  120
#define SERVOMAX4  620

#define SERVOMIN5  150
#define SERVOMAX5  620

#define SERVOMIN6  155
#define SERVOMAX6  575


typedef struct {
  double gripX;
  double gripY;
  double gripZ;
  double gripSpinAngle;
  double gripTiltAngle;
  double gripWidth;
  double duration;
  bool movesScriptEnd;
  bool showLog;
  bool errorOutOfWorkZone;
  String errorMsg;
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
  bool errorOutOfWorkZone;
  String errorMsg;
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

  int angleToPulse(int ang, long servoMin, long servoMax);
private:

};
#endif
