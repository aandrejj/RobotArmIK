/**
 * InverseKinematics.h
 */

#ifndef InverseKinematics_h
#define InverseKinematics_h
#include "Arduino.h"
//#include "ArmServos.h"
#include "ServosManager.h"
/*
 #include "RoboArmTurn.h"
*/
#include "RampOnce.h"
//#define DEBUG         //debug logging
//#define BRIEF_LOG_IKM     //just a few logs

#define MATH_PI     3.14159265
#define M_180_DIV_PI 57.29577951

class InverseKinematics {
public:
  InverseKinematics();

  void begin(int pServo_Min_milisec, int pServo_Max_milisec);
  
  RampOnce rampOnce;
  
  int angleToMicroseconds(double angle);
  
  int microsecondsToAngle(double microseconds);

  ArmServoAngles   moveToAngle     (double b, double a1, double a2, double gripSpinAngle, double gripTiltAngle, double g, double duration, bool showLog);

  ArmServoAngles moveToPos(double x, double y, double z, double gripSpinAngle, double gripTiltAngle, double g, bool movesScriptEnd, double duration);

  ArmServoAngles moveToPosXYZ(GripPositionXYZ positionXYZ);
  
  GripPositionXYZ convertAngleToPosXYZ(ArmServoAngles armServoAngles);
  
private:
  double Servo_Min_milisec;
  double Servo_Max_milisec;
};
#endif
