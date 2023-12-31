#include "InverseKinematics.h"
#include "Arduino.h"
//#include <ArmServos.h>

InverseKinematics::InverseKinematics() {

}

int InverseKinematics::angleToMicroseconds(double angle) {
  double val = 460.0 + (((2400.0 - 460.0) / 180.0) * angle);
  return (int)val;
}

//ArmServoMicrosec InverseKinematics::moveToAngle2(ArmServoAngles armServoAngles) {
//	return InverseKinematics::moveToAngle((double)armServoAngles.baseAngle, (double)armServoAngles.arm1Angle, (double)armServoAngles.arm2Angle, armServoAngles.gripSpinAngle, armServoAngles.gripTiltAngle, (double)armServoAngles.gripAngle, armServoAngles.movesScriptEnd);
//}

ArmServoMicrosec InverseKinematics::moveToAngle(double b, double a1, double a2, int gripSpinAngle, int gripTiltAngle, double g) {
  
  ArmServoMicrosec armServoMicrosec; 
  
  armServoMicrosec.arm1Microsec   = angleToMicroseconds(188 - a1);
  armServoMicrosec.arm2Microsec   = angleToMicroseconds(a2+101);
  armServoMicrosec.baseMicrosec   = angleToMicroseconds(b+90);
  armServoMicrosec.griperMicrosec = angleToMicroseconds(g);
  armServoMicrosec.griperSpinMicrosec = angleToMicroseconds(gripSpinAngle+90);
  armServoMicrosec.griperTiltMicrosec = angleToMicroseconds(gripTiltAngle);
  //armServoMicrosec.movesScriptEnd = movesScriptEnd;
  return armServoMicrosec;
}


GripPositionXYZ InverseKinematics::convertAngleToPosXYZ(ArmServoAngles armServoAngles) {
	GripPositionXYZ gripPosition;
	Serial.println("convertAngleToPosXYZ: Started");

    gripPosition.gripX = 0;
    gripPosition.gripY = 0;
    gripPosition.gripZ = 120;
    gripPosition.gripSpinAngle = 0;
    gripPosition.gripTiltAngle = 0;
    gripPosition.gripOpen = 10;
    gripPosition.movesScriptEnd = false;
	
	//ToDo Add math to evaluate all params correctly!!!
	Serial.println("convertAngleToPosXYZ: ToDo: Implement Math stuff here correctly");
	
	//double phi = armServoAngles.arm1Angle - armServoAngles.arm2Angle;
	//double theta =
	
	Serial.println("convertAngleToPosXYZ:  gripPosition.gripX          = "+String(gripPosition.gripX));
    Serial.println("convertAngleToPosXYZ:  gripPosition.gripY          = "+String(gripPosition.gripY));
    Serial.println("convertAngleToPosXYZ:  gripPosition.gripZ          = "+String(gripPosition.gripZ));
    Serial.println("convertAngleToPosXYZ:  gripPosition.gripSpinAngle  = "+String(gripPosition.gripSpinAngle));
    Serial.println("convertAngleToPosXYZ:  gripPosition.gripTiltAngle  = "+String(gripPosition.gripTiltAngle));
    Serial.println("convertAngleToPosXYZ:  gripPosition.gripOpen       = "+String(gripPosition.gripOpen));
    Serial.println("convertAngleToPosXYZ:  gripPosition.movesScriptEnd = "+String(gripPosition.movesScriptEnd));
	Serial.println("convertAngleToPosXYZ: End");
	return gripPosition;
}

ArmServoMicrosec InverseKinematics::moveToPosXYZ(GripPositionXYZ positionXYZ) {
	return InverseKinematics::moveToPos((double)positionXYZ.gripX, (double)positionXYZ.gripY, (double)positionXYZ.gripZ, (int)positionXYZ.gripSpinAngle, (int)positionXYZ.gripTiltAngle, (double)positionXYZ.gripOpen, (bool)positionXYZ.movesScriptEnd);
}
ArmServoMicrosec InverseKinematics::moveToPos(double x, double y, double z, int gripSpinAngle, int gripTiltAngle, double g, bool movesScriptEnd) {
  double b = atan2(y,x) * (180 / 3.1415); // base angle

  double l = sqrt(x*x + y*y); // x and y extension 
  
  l = l - 100; // 100mm = length of gripper when is in horizontal (flat) position

  double h = sqrt (l*l + z*z);

  double phi = atan(z/l) * (180 / 3.1415);

  double theta = acos((h/2)/120) * (180 / 3.1415);    //120 mm = length of one part of arm
  
  double a1 = phi + theta; // angle for first part of the arm
  double a2 = phi - theta; // angle for second part of the arm

  return moveToAngle(b, a1, a2, gripSpinAngle, gripTiltAngle, g);
}
