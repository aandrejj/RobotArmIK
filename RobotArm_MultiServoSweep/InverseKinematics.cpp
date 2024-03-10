#include "InverseKinematics.h"
#include "Arduino.h"
//#include <ArmServos.h>
#define OneArmLength  120

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
//----------------------------------------------------------------------------------------
GripPositionXYZ InverseKinematics::convertAngleToPosXYZ(ArmServoAngles armServoAngles) {
	GripPositionXYZ gripPosition;
	  //#if defined(DEBUG) || defined(BRIEF_LOG_IKM) 
	    //Serial.println("InverseKinematics::convertAngleToPosXYZ'back': Started"); 
      Serial.println("InverseKinematics::convertAngleToPosXYZ(ForwardKinematics): input params: armServoAngles:  baseAngle = "+String(armServoAngles.baseAngle)+", arm1Angle = "+ String(armServoAngles.arm1Angle)+", arm2Angle = "+String(armServoAngles.arm2Angle)+", gripSpinAngle = "+ String(armServoAngles.gripSpinAngle)+", gripTiltAngle = "+String(armServoAngles.gripTiltAngle)+", gripAngle = "+String(armServoAngles.gripAngle)+", duration = "+String(armServoAngles.duration)+"." );
    //#endif
    // based on  https://www.youtube.com/watch?v=1-FJhmey7vk   goto video-time: 5:53
    double l0 = (OneArmLength * cos((armServoAngles.arm1Angle)*(3.1415926/180))) ;
    double l1 = (OneArmLength * cos((armServoAngles.arm1Angle+armServoAngles.arm2Angle+armServoAngles.gripTiltAngle)*(3.1415926/180)));
    double l = l0 + l1;
    gripPosition.gripX = l* cos((armServoAngles.baseAngle)*(3.1415926/180));
    gripPosition.gripY = l* sin((armServoAngles.baseAngle)*(3.1415926/180));
    gripPosition.gripZ = (OneArmLength * cos((armServoAngles.arm1Angle)*(3.1415926/180))) + (OneArmLength * sin((armServoAngles.arm1Angle + armServoAngles.arm2Angle + armServoAngles.gripTiltAngle)*(3.1415926/180)));
    
    gripPosition.gripTiltAngle = (armServoAngles.arm1Angle + armServoAngles.arm2Angle + armServoAngles.gripTiltAngle)*(3.1415926/180);
    gripPosition.gripSpinAngle = armServoAngles.gripSpinAngle;
    gripPosition.gripWidth = 2 * (sin(armServoAngles.gripAngle * (3.1415926/180)) * 30);
    
    /*
    gripPosition.gripX = 10;
    gripPosition.gripY = 0;
    gripPosition.gripZ = 120;
    gripPosition.gripSpinAngle = 0;
    gripPosition.gripTiltAngle = 0;
    //gripPosition.gripWidth = 80; 
    */
    gripPosition.duration = armServoAngles.duration;
    gripPosition.movesScriptEnd = false;
    gripPosition.showLog = false;
		
	  //#if defined(DEBUG) || defined(BRIEF_LOG_IKM)  
	    Serial.print("InverseKinematics::convertAngleToPosXYZ (ForwardKinematics) output: ");
      Serial.print("l0 = "+String(l0)+", ");
      Serial.print("l1 = "+String(l1)+", ");
      Serial.print("l = "+String(l)+", ");
      Serial.println("gripPosition (X,Y,Z) = ("+String(gripPosition.gripX)+", "+String(gripPosition.gripY)+", "+String(gripPosition.gripZ)+" ), Angles (Spin, Tilt, Open) = ("+String(gripPosition.gripSpinAngle) +", "+String(gripPosition.gripTiltAngle)+", "+String(gripPosition.gripWidth)+"), duration="+String(gripPosition.duration)+", movesScriptEnd = "+String(gripPosition.movesScriptEnd));
	     Serial.print("InverseKinematics::convertAngleToPosXYZ (ForwardKinematics) output: l0 = "+String(l0)+", l1 = "+String(l1)+", l = "+String(l)+", gripPosition (X,Y,Z) = ("+String(gripPosition.gripX)+", "+String(gripPosition.gripY)+", "+String(gripPosition.gripZ)+" ), Angles (Spin, Tilt, Open) = ("+String(gripPosition.gripSpinAngle) +", "+String(gripPosition.gripTiltAngle)+", "+String(gripPosition.gripWidth)+"), duration="+String(gripPosition.duration)+", movesScriptEnd = "+String(gripPosition.movesScriptEnd));
	    Serial.println("InverseKinematics::convertAngleToPosXYZ (ForwardKinematics) output: l0 = "+String(l0)+", l1 = "+String(l1)+", l = "+String(l)+", gripPosition (X,Y,Z) = ("+String(gripPosition.gripX)+", "+String(gripPosition.gripY)+", "+String(gripPosition.gripZ)+" ), Angles (Spin, Tilt, Open) = ("+String(gripPosition.gripSpinAngle) +", "+String(gripPosition.gripTiltAngle)+", "+String(gripPosition.gripWidth)+"), duration="+String(gripPosition.duration)+", movesScriptEnd = "+String(gripPosition.movesScriptEnd));
    //#endif
	  #ifdef DEBUG 
	    Serial.println("convertAngleToPosXYZ: End");
    #endif
	return gripPosition;
}
//--------------------------------------------------------------------------------------------------------
ArmServoAngles InverseKinematics::moveToAngle(double b, double a1, double a2, double gripSpinAngle, double gripTiltAngle, double g, double duration, bool showLog) {
  
  //https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
  //value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
  //so that servos often respond to values between 700 and 2300.
  #if defined(DEBUG)  || defined(BRIEF_LOG_IKM) 
    Serial.print("InverseKinematics::moveToAngle()Input: (b, a1, a2), grip(SpinAngle,TiltAngle,g) = (" + String(b)+", "+String(a1)+", "+String(a2)+"), grip("+String(gripSpinAngle)+", "+String(gripTiltAngle)+", "+String(g)+"). ");
  #endif
  ArmServoAngles armServoAngles; 
  armServoAngles.baseAngle     = (b + 90);
  armServoAngles.arm1Angle     = a1; //(180 - a1);
  armServoAngles.arm2Angle     = (-40 + a2);

  armServoAngles.gripSpinAngle = (gripSpinAngle + 90);
  armServoAngles.gripTiltAngle = (gripTiltAngle + 90);
  armServoAngles.gripAngle     = (g + 30);
  armServoAngles.duration = duration;
  //armServoAngles.movesScriptEnd = movesScriptEnd;
  #if defined(DEBUG) || defined(BRIEF_LOG_IKM) 
    Serial.println(" |   Outut: armServoAngles=("+String(armServoAngles.baseAngle)+", "+String(armServoAngles.arm1Angle)+", "+String(armServoAngles.arm2Angle)+") . Angles = ("+String(armServoAngles.gripSpinAngle)+","+String(armServoAngles.gripTiltAngle)+","+String(armServoAngles.gripAngle)+"),  g = "+String(g)+".");
  #endif
  if (showLog==true){ 
    Serial.println(" |   moveToAngle:Outut: armServoAngles=("+String(armServoAngles.baseAngle)+", "+String(armServoAngles.arm1Angle)+", "+String(armServoAngles.arm2Angle)+") . Angles = ("+String(armServoAngles.gripSpinAngle)+","+String(armServoAngles.gripTiltAngle)+","+String(armServoAngles.gripAngle)+"),  g = "+String(g)+".");
  }
  return armServoAngles;
}

//----------------------------------------------------------------------------------------
ArmServoAngles InverseKinematics::moveToPosXYZ(GripPositionXYZ positionXYZ) {
  #if defined(DEBUG)
    Serial.println("InverseKinematics::moveToPosXYZ: InputParams positionXYZ  (X,Y,Z) = ("+String(positionXYZ.gripX)+", "+String(positionXYZ.gripY)+", "+String(positionXYZ.gripZ)+" ), Grip (Spin, Tilt, Width) = ("+String(positionXYZ.gripSpinAngle) +", "+String(positionXYZ.gripTiltAngle)+", "+String(positionXYZ.gripWidth)+"), duration="+String(positionXYZ.duration)+", movesScriptEnd = "+String(positionXYZ.movesScriptEnd));
  #endif
  double b = atan2(positionXYZ.gripY,positionXYZ.gripX) * M_180_DIV_PI; //(180 / MATH_PI); // base angle

  double l = sqrt(positionXYZ.gripX * positionXYZ.gripX + positionXYZ.gripY * positionXYZ.gripY); // x and y extension 
  
  l = l; // 100mm = length of gripper when is in horizontal (flat) position

  if(l==0) {
    l=-0.2;
    Serial.print("InverseKinematics::moveToPosXYZ: WARNING l=0!!! Unable to use l as divider. l changed to value '"+String(l)+"'. ");
  }

  double h = sqrt (l*l + positionXYZ.gripZ * positionXYZ.gripZ);

  double phi = atan(positionXYZ.gripZ/l) * M_180_DIV_PI;//(180 / MATH_PI);

  double theta = acos((h/2)/OneArmLength) * M_180_DIV_PI; //(180 / MATH_PI);    //120 mm = length of first and second part of arm (120 = brown and 120 = white arm with black bracket)
  
  double a1 = phi + theta; // angle for first part of the arm
  //double a2 = phi - theta; // angle for second part of the arm
  double a2 =  (180 - a1) + theta;
  //double a2 =  theta - a1;
  //pictures here :  https://www.youtube.com/watch?v=Q-UeYEpwXXU


  double newGripTiltAngle = positionXYZ.gripTiltAngle + (0.7 * a1) -(0.9 * a2) + 30;
  //double newGripTiltAngle = positionXYZ.gripTiltAngle;

  double gripAngle = asin((positionXYZ.gripWidth/2)/30) * M_180_DIV_PI;//(180 / MATH_PI);

  #ifdef DEBUG 
    Serial.println("InverseKinematics::moveToPos(): b             = "+String(b )+".");
    Serial.println("InverseKinematics::moveToPos(): a1            = "+String(a1)+".");
    Serial.println("InverseKinematics::moveToPos(): a2            = "+String(a2)+".");
    //Serial.println("InverseKinematics::moveToPos(): gripSpinAngle = "+String(gripSpinAngle)+".");
    Serial.println("InverseKinematics::moveToPos(): newGripTiltAngle = "+String(newGripTiltAngle)+".");
    Serial.println("InverseKinematics::moveToPos(): gripAngle     = "+String(gripAngle)+".");
  #endif
  if (positionXYZ.showLog) {
    Serial.print("InverseKinematics::moveToPosXYZ() b:"+String(b)+", l:"+String(l)+", h:"+String(h)+", phi:"+String(phi)+", theta:"+String(theta)+", a1:"+String(a1)+", a2:"+String(a2)+", g:"+String(gripAngle)+".");
  }
  return moveToAngle(b, a1, a2, positionXYZ.gripSpinAngle, newGripTiltAngle, gripAngle, positionXYZ.duration, positionXYZ.showLog);
}
