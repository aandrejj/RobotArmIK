/**
 * InverseKinematics.h
 */

#ifndef InverseKinematics_h
#define InverseKinematics_h
#include "Arduino.h"
//#include "ArmServos.h"
#include "ServosManager.h"
#include "RoboArmTurn.h"


class InverseKinematics {
public:
  InverseKinematics();
  
  int angleToMicroseconds(double angle);

  ArmServoMicrosec moveToAngle(double b, double a1, double a2, int gripSpinAngle, int gripTiltAngle, double g);

  //ArmServoMicrosec moveToAngle2(ArmServoAngles armServoAngles);

  ArmServoMicrosec moveToPos(double x, double y, double z, int gripSpinAngle, int gripTiltAngle, double g, bool movesScriptEnd);

  ArmServoMicrosec moveToPosXYZ(GripPositionXYZ positionXYZ);
  
  GripPositionXYZ convertAngleToPosXYZ(ArmServoAngles armServoAngles);
  
private:
};
#endif
