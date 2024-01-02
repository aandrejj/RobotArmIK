/**
 * InverseKinematics.h
 */

#ifndef InverseKinematics_h
#define InverseKinematics_h
#include "Arduino.h"
//#include "ArmServos.h"
#include "ServosManager.h"
#include "RoboArmTurn.h"
#include "RampOnce.h"
//#define DEBUG         //extensive logging

class InverseKinematics {
public:
  InverseKinematics();

  void begin(int pServo_Min_milisec, int pServo_Max_milisec);
  
  RampOnce rampOnce;
  
  int angleToMicroseconds(double angle);
  
  int microsecondsToAngle(double microseconds);

  ArmServoMicrosec moveToAngle_msec(double b, double a1, double a2, int gripSpinAngle, int gripTiltAngle, double g);
  ArmServoAngles   moveToAngle     (double b, double a1, double a2, int gripSpinAngle, int gripTiltAngle, double g);

  //ArmServoMicrosec moveToAngle2(ArmServoAngles armServoAngles);

  ArmServoAngles moveToPos(double x, double y, double z, int gripSpinAngle, int gripTiltAngle, double g, bool movesScriptEnd);

  ArmServoAngles moveToPosXYZ(GripPositionXYZ positionXYZ);
  
  GripPositionXYZ convertAngleToPosXYZ(ArmServoAngles armServoAngles);
  
private:
  double Servo_Min_milisec;
  double Servo_Max_milisec;
};
#endif
