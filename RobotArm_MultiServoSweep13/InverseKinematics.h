/**
 * InverseKinematics.h
 */

#ifndef InverseKinematics_h
#define InverseKinematics_h
#include "Arduino.h"

class InverseKinematics {
public:
  InverseKinematics();
  
  int angleToMicroseconds(double angle);

  void moveToAngle(double b, double a1, double a2, double g);

  void moveToPos(double x, double y, double z, double g);
private:
};
#endif
