#include "InverseKinematics.h"
#include "Arduino.h"
//#include <ArmServos.h>

InverseKinematics::InverseKinematics() {
}

void InverseKinematics::begin(int pServo_Min_milisec, int pServo_Max_milisec){
  InverseKinematics::Servo_Min_milisec = (double)pServo_Min_milisec;
  InverseKinematics::Servo_Max_milisec = (double)pServo_Max_milisec;
}

int InverseKinematics::angleToMicroseconds(double angle) {
  double val = InverseKinematics::Servo_Min_milisec + (((InverseKinematics::Servo_Max_milisec - InverseKinematics::Servo_Min_milisec) / 180.0) * angle);
  return (int)val;
}

int InverseKinematics::microsecondsToAngle(double microseconds) {
//from 460 to 2400   //from 0 to 180
  double val = (microseconds - InverseKinematics::Servo_Min_milisec)/(InverseKinematics::Servo_Max_milisec - InverseKinematics::Servo_Min_milisec)*180;
  return (int)val;
}

//ArmServoMicrosec InverseKinematics::moveToAngle2(ArmServoAngles armServoAngles) {
//	return InverseKinematics::moveToAngle((double)armServoAngles.baseAngle, (double)armServoAngles.arm1Angle, (double)armServoAngles.arm2Angle, armServoAngles.gripSpinAngle, armServoAngles.gripTiltAngle, (double)armServoAngles.gripAngle, armServoAngles.movesScriptEnd);
//}

ArmServoMicrosec InverseKinematics::moveToAngle_msec(double b, double a1, double a2, int gripSpinAngle, int gripTiltAngle, double g) {
  
  //https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
  //value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
  //so that servos often respond to values between 700 and 2300.

  ArmServoMicrosec armServoMicrosec; 
  armServoMicrosec.baseMicrosec       = angleToMicroseconds(b+90);
  armServoMicrosec.arm1Microsec       = angleToMicroseconds(100-a1); //angleToMicroseconds(188-a1);
  armServoMicrosec.arm2Microsec       = angleToMicroseconds(a2+101);
  armServoMicrosec.griperMicrosec     = angleToMicroseconds(g + 90);
  armServoMicrosec.griperSpinMicrosec = angleToMicroseconds(gripSpinAngle + 100);
  armServoMicrosec.griperTiltMicrosec = angleToMicroseconds(gripTiltAngle + 90);
  //armServoMicrosec.movesScriptEnd   = movesScriptEnd;
  
  #ifdef DEBUG 
    Serial.println("InverseKinematics::moveToAngle(): b             = "+String(b)            +", baseMicrosec       = "+String(armServoMicrosec.baseMicrosec)+", _angle = "+String(InverseKinematics::microsecondsToAngle(armServoMicrosec.baseMicrosec))+".");
    Serial.println("InverseKinematics::moveToAngle(): a1            = "+String(a1)           +", arm1Microsec       = "+String(armServoMicrosec.arm1Microsec)+", _angle = "+String(InverseKinematics::microsecondsToAngle(armServoMicrosec.arm1Microsec))+".");
    Serial.println("InverseKinematics::moveToAngle(): a2            = "+String(a2)           +", arm2Microsec       = "+String(armServoMicrosec.arm2Microsec)+", _angle = "+String(InverseKinematics::microsecondsToAngle(armServoMicrosec.arm2Microsec))+".");
    Serial.println("InverseKinematics::moveToAngle(): gripSpinAngle = "+String(gripSpinAngle)+", griperSpinMicrosec = "+String(armServoMicrosec.griperSpinMicrosec)+", _angle = "+String(InverseKinematics::microsecondsToAngle(armServoMicrosec.griperSpinMicrosec))+".");
    Serial.println("InverseKinematics::moveToAngle(): gripTiltAngle = "+String(gripTiltAngle)+", griperTiltMicrosec = "+String(armServoMicrosec.griperTiltMicrosec)+", _angle = "+String(InverseKinematics::microsecondsToAngle(armServoMicrosec.griperTiltMicrosec))+".");
    Serial.println("InverseKinematics::moveToAngle(): g             = "+String(g)            +", griperMicrosec     = "+String(armServoMicrosec.griperMicrosec)+", _angle = "+String(InverseKinematics::microsecondsToAngle(armServoMicrosec.griperMicrosec))+".");
  #endif

  return armServoMicrosec;
}

ArmServoAngles InverseKinematics::moveToAngle(double b, double a1, double a2, int gripSpinAngle, int gripTiltAngle, double g) {
  
  //https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
  //value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
  //so that servos often respond to values between 700 and 2300.

  ArmServoAngles armServoAngles; 
  armServoAngles.baseAngle     = (b + 90);
  armServoAngles.arm1Angle     = (100 - a1);
  armServoAngles.arm2Angle     = (a2 + 101);
  armServoAngles.gripAngle     = (g);
  armServoAngles.gripSpinAngle = (gripSpinAngle + 100);
  armServoAngles.gripTiltAngle = (gripTiltAngle + 90);
  //armServoAngles.movesScriptEnd = movesScriptEnd;

  #ifdef DEBUG 
    Serial.println("InverseKinematics::moveToAngle(): b             = "+String(b)            +", baseAngle       = "+String(armServoAngles.baseAngle)+".");
    Serial.println("InverseKinematics::moveToAngle(): a1            = "+String(a1)           +", arm1Angle       = "+String(armServoAngles.arm1Angle)+".");
    Serial.println("InverseKinematics::moveToAngle(): a2            = "+String(a2)           +", arm2Angle       = "+String(armServoAngles.arm2Angle)+".");
    Serial.println("InverseKinematics::moveToAngle(): gripSpinAngle = "+String(gripSpinAngle)+", griperSpinAngle = "+String(armServoAngles.gripSpinAngle)+".");
    Serial.println("InverseKinematics::moveToAngle(): gripTiltAngle = "+String(gripTiltAngle)+", griperTiltAngle = "+String(armServoAngles.gripTiltAngle)+".");
    Serial.println("InverseKinematics::moveToAngle(): g             = "+String(g)            +", griperAngle     = "+String(armServoAngles.gripAngle)+".");
  #endif  
  return armServoAngles;
}


GripPositionXYZ InverseKinematics::convertAngleToPosXYZ(ArmServoAngles armServoAngles) {
	GripPositionXYZ gripPosition;
	  #ifdef DEBUG
	    Serial.println("InverseKinematics::convertAngleToPosXYZ: Started");
    #endif

    gripPosition.gripX = 10;
    gripPosition.gripY = 0;
    gripPosition.gripZ = 120;
    gripPosition.gripSpinAngle = 0;
    gripPosition.gripTiltAngle = 0;
    gripPosition.gripOpen = 80;
    gripPosition.movesScriptEnd = false;
	
	  //ToDo Add math to evaluate all params correctly!!!
	  #ifdef DEBUG  
	    Serial.println("InverseKinematics::convertAngleToPosXYZ: ToDo: Implement Math stuff here correctly");
    #endif
  
	
	//double phi = armServoAngles.arm1Angle - armServoAngles.arm2Angle;
	//double theta =
	  #ifdef DEBUG 
	    Serial.print("InverseKinematics::convertAngleToPosXYZ:  gripPosition (X,Y,Z) = ("+String(gripPosition.gripX)+", "+String(gripPosition.gripY)+", "+String(gripPosition.gripZ)+" ), ");
      Serial.print("Angles (Spin, Tilt, Open) = ("+String(gripPosition.gripSpinAngle) +", "+String(gripPosition.gripTiltAngle)+", "+String(gripPosition.gripOpen)+"), ");
      Serial.println("movesScriptEnd = "+String(gripPosition.movesScriptEnd));
    #endif
	  #ifdef DEBUG 
	    Serial.println("convertAngleToPosXYZ: End");
    #endif
	return gripPosition;
}

ArmServoAngles InverseKinematics::moveToPosXYZ(GripPositionXYZ positionXYZ) {
	return InverseKinematics::moveToPos((double)positionXYZ.gripX, (double)positionXYZ.gripY, (double)positionXYZ.gripZ, (int)positionXYZ.gripSpinAngle, (int)positionXYZ.gripTiltAngle, (double)positionXYZ.gripOpen, (bool)positionXYZ.movesScriptEnd);
}
ArmServoAngles InverseKinematics::moveToPos(double x, double y, double z, int gripSpinAngle, int gripTiltAngle, double g, bool movesScriptEnd) {
  double b = atan2(y,x) * (180 / 3.1415); // base angle

  double l = sqrt(x*x + y*y); // x and y extension 
  
  l = l - 100; // 100mm = length of gripper when is in horizontal (flat) position

  double h = sqrt (l*l + z*z);

  double phi = atan(z/l) * (180 / 3.1415);

  double theta = acos((h/2)/120) * (180 / 3.1415);    //120 mm = length of one part of arm
  
  double a1 = phi + theta; // angle for first part of the arm
  double a2 = phi - theta; // angle for second part of the arm

  //double newGripTiltAngle = gripTiltAngle - a1 + (180 -a2);
  double newGripTiltAngle = gripTiltAngle;

  return moveToAngle(b, a1, a2, gripSpinAngle, newGripTiltAngle, g);
}
