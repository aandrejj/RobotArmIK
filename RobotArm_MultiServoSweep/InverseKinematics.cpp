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

ArmServoMicrosec InverseKinematics::moveToAngle_msec(double b, double a1, double a2, double gripSpinAngle, double gripTiltAngle, double g, double duration) {
  
  //https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
  //value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
  //so that servos often respond to values between 700 and 2300.

  //---------unused yet---------------------------------------
  ArmServoMicrosec armServoMicrosec; 
  armServoMicrosec.baseMicrosec       = angleToMicroseconds(b+90);
  armServoMicrosec.arm1Microsec       = angleToMicroseconds(100-a1); //angleToMicroseconds(188-a1);
  armServoMicrosec.arm2Microsec       = angleToMicroseconds(a2+101);
  armServoMicrosec.gripMicrosec     = angleToMicroseconds(g + 90);
  armServoMicrosec.gripSpinMicrosec = angleToMicroseconds(gripSpinAngle + 100);
  armServoMicrosec.gripTiltMicrosec = angleToMicroseconds(gripTiltAngle + 90);
  armServoMicrosec.duration = duration;
  //armServoMicrosec.movesScriptEnd   = movesScriptEnd;
  
  #ifdef DEBUG 
    Serial.println("InverseKinematics::moveToAngle(): b             = "+String(b)            +", baseMicrosec       = "+String(armServoMicrosec.baseMicrosec)+".");
    Serial.println("InverseKinematics::moveToAngle(): a1            = "+String(a1)           +", arm1Microsec       = "+String(armServoMicrosec.arm1Microsec)+".");
    Serial.println("InverseKinematics::moveToAngle(): a2            = "+String(a2)           +", arm2Microsec       = "+String(armServoMicrosec.arm2Microsec)+".");
    Serial.println("InverseKinematics::moveToAngle(): gripSpinAngle = "+String(gripSpinAngle)+", gripSpinMicrosec = "+String(armServoMicrosec.gripSpinMicrosec)+".");
    Serial.println("InverseKinematics::moveToAngle(): gripTiltAngle = "+String(gripTiltAngle)+", gripTiltMicrosec = "+String(armServoMicrosec.gripTiltMicrosec)+".");
    Serial.println("InverseKinematics::moveToAngle(): g             = "+String(g)            +", gripMicrosec     = "+String(armServoMicrosec.gripMicrosec)+".");
    Serial.println("InverseKinematics::moveToAngle(): duration      = "+String(duration)     +", duration         = "+String(armServoMicrosec.duration)+".");
    
  #endif

  return armServoMicrosec;
}

ArmServoAngles InverseKinematics::moveToAngle(double b, double a1, double a2, double gripSpinAngle, double gripTiltAngle, double g, double duration) {
  
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
  armServoAngles.duration = duration;
  //armServoAngles.movesScriptEnd = movesScriptEnd;
  #if defined(BRIEF_LOG) 
    Serial.println("InverseKinematics::moveToAngle(): armServoAngles ={"+String(armServoAngles.baseAngle)+", "+String(armServoAngles.arm1Angle)+", "+String(armServoAngles.arm2Angle)+". Angles="+String(armServoAngles.gripSpinAngle)+","+String(armServoAngles.gripTiltAngle)+","+String(armServoAngles.gripAngle)+"},  g = "+String(g)+".");
  #elif DEBUG 
    Serial.println("InverseKinematics::moveToAngle(): b             = "+String(b)            +", baseAngle       = "+String(armServoAngles.baseAngle)+".");
    Serial.println("InverseKinematics::moveToAngle(): a1            = "+String(a1)           +", arm1Angle       = "+String(armServoAngles.arm1Angle)+".");
    Serial.println("InverseKinematics::moveToAngle(): a2            = "+String(a2)           +", arm2Angle       = "+String(armServoAngles.arm2Angle)+".");
    Serial.println("InverseKinematics::moveToAngle(): gripSpinAngle = "+String(gripSpinAngle)+", griperSpinAngle = "+String(armServoAngles.gripSpinAngle)+".");
    Serial.println("InverseKinematics::moveToAngle(): gripTiltAngle = "+String(gripTiltAngle)+", griperTiltAngle = "+String(armServoAngles.gripTiltAngle)+".");
    Serial.println("InverseKinematics::moveToAngle(): g             = "+String(g)            +", griperAngle     = "+String(armServoAngles.gripAngle)+".");
    Serial.println("InverseKinematics::moveToAngle(): duration      = "+String(duration)     +", duration        = "+String(armServoAngles.duration)+".");
  #endif  
  return armServoAngles;
}

//----------------------------------------------------------------------------------------
GripPositionXYZ InverseKinematics::convertAngleToPosXYZ(ArmServoAngles armServoAngles) {
	GripPositionXYZ gripPosition;
	  #if defined(DEBUG) || defined(BRIEF_LOG) 
	    Serial.println("InverseKinematics::convertAngleToPosXYZ'back': Started"); 
      Serial.println("InverseKinematics::convertAngleToPosXYZ'back': input params: armServoAngles:  baseAngle = "+String(armServoAngles.baseAngle)+", arm1Angle = "+ String(armServoAngles.arm1Angle)+", arm2Angle = "+String(armServoAngles.arm2Angle)+", gripSpinAngle = "+ String(armServoAngles.gripSpinAngle)+", gripTiltAngle = "+String(armServoAngles.gripTiltAngle)+", gripAngle = "+String(armServoAngles.gripAngle)+", duration = "+String(armServoAngles.duration)+"." );
    #endif
    
    gripPosition.gripWidth = 2 * (sin(armServoAngles.gripAngle * (3.1415926/180)) * 30);

    
    gripPosition.gripX = 10;
    gripPosition.gripY = 0;
    gripPosition.gripZ = 120;
    gripPosition.gripSpinAngle = 0;
    gripPosition.gripTiltAngle = 0;
    /* gripPosition.gripWidth = 80; */
    gripPosition.duration = armServoAngles.duration;
    gripPosition.movesScriptEnd = false;
	
	  //ToDo Add math to evaluate all params correctly!!!
	  #ifdef DEBUG  
	    Serial.println("InverseKinematics::convertAngleToPosXYZ: ToDo: Implement Math stuff here correctly");
    #endif
  
	
	//double phi = armServoAngles.arm1Angle - armServoAngles.arm2Angle;
	//double theta =
	  #if defined(DEBUG) || defined(BRIEF_LOG)  
	    Serial.println("InverseKinematics::convertAngleToPosXYZ 'back' output:  gripPosition (X,Y,Z) = ("+String(gripPosition.gripX)+", "+String(gripPosition.gripY)+", "+String(gripPosition.gripZ)+" ), Angles (Spin, Tilt, Open) = ("+String(gripPosition.gripSpinAngle) +", "+String(gripPosition.gripTiltAngle)+", "+String(gripPosition.gripWidth)+"), duration="+String(gripPosition.duration)+", movesScriptEnd = "+String(gripPosition.movesScriptEnd));
    #endif
	  #ifdef DEBUG 
	    Serial.println("convertAngleToPosXYZ: End");
    #endif
	return gripPosition;
}
//----------------------------------------------------------------------------------------

ArmServoAngles InverseKinematics::moveToPosXYZ(GripPositionXYZ positionXYZ) {
//	return InverseKinematics::moveToPos((double)positionXYZ.gripX, (double)positionXYZ.gripY, (double)positionXYZ.gripZ, (double)positionXYZ.gripSpinAngle, (double)positionXYZ.gripTiltAngle, (double)positionXYZ.gripWidth, (double)positionXYZ.duration, (bool)positionXYZ.movesScriptEnd);
//}
//----------------------------------------------------------------------------------------
//ArmServoAngles InverseKinematics::moveToPos(double x, double y, double z, double gripSpinAngle, double gripTiltAngle, double gripWidth, bool movesScriptEnd,  double duration) {
  double b = atan2(positionXYZ.gripY,positionXYZ.gripX) * (180 / MATH_PI); // base angle

  double l = sqrt(positionXYZ.gripX * positionXYZ.gripX + positionXYZ.gripY * positionXYZ.gripY); // x and y extension 
  
  l = l - 100; // 100mm = length of gripper when is in horizontal (flat) position

  double h = sqrt (l*l + positionXYZ.gripZ * positionXYZ.gripZ);

  double phi = atan(positionXYZ.gripZ/l) * (180 / MATH_PI);

  double theta = acos((h/2)/125) * (180 / MATH_PI);    //120 mm = length of first and second part of arm (120 = brown and 120 = white arm with black bracket)
  
  double a1 = phi + theta; // angle for first part of the arm
  //double a2 = phi - theta; // angle for second part of the arm
  double a2 =  (0 - a1) + theta;

  ////double newGripTiltAngle = gripTiltAngle - a1 + (180 -a2);
  //double newGripTiltAngle = positionXYZ.gripTiltAngle;

  double gripAngle = asin((positionXYZ.gripWidth/2)/30) * (180 / MATH_PI);

  #ifdef DEBUG 
    Serial.println("InverseKinematics::moveToPos(): b             = "+String(b )+".");
    Serial.println("InverseKinematics::moveToPos(): a1            = "+String(a1)+".");
    Serial.println("InverseKinematics::moveToPos(): a2            = "+String(a2)+".");
    Serial.println("InverseKinematics::moveToPos(): gripSpinAngle = "+String(gripSpinAngle)+".");
    Serial.println("InverseKinematics::moveToPos(): gripTiltAngle = "+String(gripTiltAngle)+".");
    Serial.println("InverseKinematics::moveToPos(): gripAngle     = "+String(gripAngle)+".");
  #endif
  
  return moveToAngle(b, a1, a2, positionXYZ.gripSpinAngle, positionXYZ.gripTiltAngle, gripAngle, positionXYZ.duration);
}
