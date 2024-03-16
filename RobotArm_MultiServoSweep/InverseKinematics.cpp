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
//----------------------------------------------------------------------------------------
GripPositionXYZ InverseKinematics::convertAngleToPosXYZ(ArmServoAngles armServoAngles) {
	GripPositionXYZ gripPosition;
  gripPosition.errorOutOfWorkZone = false;
	  //#if defined(DEBUG) || defined(BRIEF_LOG_IKM) 
	    //Serial.println("IK::convertAngleToPosXYZ'back': Started"); 
      Serial.println("IK::convertAngleToPosXYZ(ForwardKinematics): input params: armServoAngles:  baseAngle = "+String(armServoAngles.baseAngle)+", arm1Angle = "+ String(armServoAngles.arm1Angle)+", arm2Angle = "+String(armServoAngles.arm2Angle)+", gripSpinAngle = "+ String(armServoAngles.gripSpinAngle)+", gripTiltAngle = "+String(armServoAngles.gripTiltAngle)+", gripAngle = "+String(armServoAngles.gripAngle)+", duration = "+String(armServoAngles.duration)+"." );
    //#endif
    // based on  https://www.youtube.com/watch?v=1-FJhmey7vk   goto video-time: 5:53
    double l = (OneArmLength * cos((armServoAngles.arm1Angle)*(3.1415926/180))) + (OneArmLength * cos((armServoAngles.arm1Angle+armServoAngles.arm2Angle+armServoAngles.gripTiltAngle)*(3.1415926/180)));
    if(l<0) {
      gripPosition.errorOutOfWorkZone = true;
      gripPosition.errorMsg ="Negative l";
    } 
    //else {
      gripPosition.gripX = l* cos((armServoAngles.baseAngle)*(3.1415926/180));
      gripPosition.gripY = l* sin((armServoAngles.baseAngle)*(3.1415926/180));
      gripPosition.gripZ = (OneArmLength * cos((armServoAngles.arm1Angle)*(3.1415926/180))) + (OneArmLength * sin((armServoAngles.arm1Angle + armServoAngles.arm2Angle + armServoAngles.gripTiltAngle)*(3.1415926/180)));
      
      gripPosition.gripTiltAngle = (armServoAngles.arm1Angle + armServoAngles.arm2Angle + armServoAngles.gripTiltAngle)*(3.1415926/180);
      gripPosition.gripSpinAngle = armServoAngles.gripSpinAngle;
      gripPosition.gripWidth = 2 * (sin(armServoAngles.gripAngle * (3.1415926/180)) * 30);
      
      gripPosition.duration = armServoAngles.duration;
      gripPosition.movesScriptEnd = false;
      gripPosition.showLog = false;
    //}
	  //#if defined(DEBUG) || defined(BRIEF_LOG_IKM)  
	    Serial.print("IK::convertAngleToPosXYZ (ForwardKinematics) output: ");
      if(gripPosition.errorOutOfWorkZone==true){
        Serial.print(" errorOutOfWorkZone. errorMsg="+String(gripPosition.errorMsg)+" .");
      }
      //Serial.print("l0 = "+String(l0)+", ");
      //Serial.print("l1 = "+String(l1)+", ");
      Serial.print("l = "+String(l)+", ");
      Serial.println("gripPosition (X,Y,Z) = ("+String(gripPosition.gripX)+", "+String(gripPosition.gripY)+", "+String(gripPosition.gripZ)+" ), Angles (Spin, Tilt, Open) = ("+String(gripPosition.gripSpinAngle) +", "+String(gripPosition.gripTiltAngle)+", "+String(gripPosition.gripWidth)+"), duration="+String(gripPosition.duration)+", movesScriptEnd = "+String(gripPosition.movesScriptEnd));
	      //Serial.print("IK::convertAngleToPosXYZ (ForwardKinematics) output: l = "+String(l)+", gripPosition (X,Y,Z) = ("+String(gripPosition.gripX)+", "+String(gripPosition.gripY)+", "+String(gripPosition.gripZ)+" ), Angles (Spin, Tilt, Open) = ("+String(gripPosition.gripSpinAngle) +", "+String(gripPosition.gripTiltAngle)+", "+String(gripPosition.gripWidth)+"), duration="+String(gripPosition.duration)+", movesScriptEnd = "+String(gripPosition.movesScriptEnd));
	    Serial.println("IK::convertAngleToPosXYZ (ForwardKinematics) output: l = "+String(l)+", gripPosition (X,Y,Z) = ("+String(gripPosition.gripX)+", "+String(gripPosition.gripY)+", "+String(gripPosition.gripZ)+" ), Angles (Spin, Tilt, Open) = ("+String(gripPosition.gripSpinAngle) +", "+String(gripPosition.gripTiltAngle)+", "+String(gripPosition.gripWidth)+"), duration="+String(gripPosition.duration)+", movesScriptEnd = "+String(gripPosition.movesScriptEnd));
    //#endif
	  #ifdef DEBUG 
	    Serial.println("convertAngleToPosXYZ: End");
    #endif
	return gripPosition;
}
//--------------------------------------------------------------------------------------------------------
ArmServoAngles InverseKinematics::moveToAngle(double b, double a1, double beta, double gripSpinAngle, double gripTiltAngle, double g, double duration, bool showLog) {
  return InverseKinematics::moveToAngle (b, a1, beta, gripSpinAngle, gripTiltAngle, g, duration, showLog,false, "");
}  
ArmServoAngles InverseKinematics::moveToAngle(double b, double a1, double beta, double gripSpinAngle, double gripTiltAngle, double g, double duration, bool showLog, bool errorOutOfWorkZone, String errorMsg) {
  
  //https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
  //value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
  //so that servos often respond to values between 700 and 2300.
  ArmServoAngles armServoAngles;
    //#if defined(DEBUG)  || defined(BRIEF_LOG_IKM) 
    if(showLog==true){
      Serial.print("IK::moveToAngle()Input: (b, a1, beta) = ");
      Serial.print("(" + String(b)+", "+String(a1)+", "+String(beta)+")| ");
      Serial.print(" grip(SpinAngle,TiltAngle,g) = ");
      Serial.print("grip("+String(gripSpinAngle)+", "+String(gripTiltAngle)+", "+String(g)+")| ");
      Serial.print("errOutOfWorkZone="+String(errorOutOfWorkZone)+", errMsg='"+String(errorMsg)+"'. ");
    }
    //#endif
    
    armServoAngles.errorOutOfWorkZone = errorOutOfWorkZone;
    armServoAngles.errorMsg = errorMsg;
  if(errorOutOfWorkZone == true) {
    Serial.print("IK::moveToAngle() errorOutOfWorkZone: errorMsg = "+errorMsg +".  SKIP");
  } else {
    armServoAngles.baseAngle     = (b);
    armServoAngles.arm1Angle     = (180 - a1);
    armServoAngles.arm2Angle     = beta; //(-40 + a2);

    armServoAngles.gripSpinAngle = (gripSpinAngle + 90);
    armServoAngles.gripTiltAngle = (gripTiltAngle); //(gripTiltAngle + 90);
    armServoAngles.gripAngle     = (g + 30);
    armServoAngles.duration = duration;
    //armServoAngles.movesScriptEnd = movesScriptEnd;
    #if defined(DEBUG) || defined(BRIEF_LOG_IKM) 
      Serial.println(" | Outut: armServoAngles=("+String(armServoAngles.baseAngle)+", "+String(armServoAngles.arm1Angle)+", "+String(armServoAngles.arm2Angle)+") . Angles = ("+String(armServoAngles.gripSpinAngle)+","+String(armServoAngles.gripTiltAngle)+","+String(armServoAngles.gripAngle)+"),  g = "+String(g)+".");
    #endif
    if (showLog==true){ 
      Serial.print(" |   moveToAngle:Outut: armServoAngles=("+String(armServoAngles.baseAngle)+", "+String(armServoAngles.arm1Angle)+", "+String(armServoAngles.arm2Angle)+") . Angles = ("+String(armServoAngles.gripSpinAngle)+","+String(armServoAngles.gripTiltAngle)+","+String(armServoAngles.gripAngle)+"),  g = "+String(g)+", ");
      Serial.println("errorOutOfWorkZone = "+String(armServoAngles.errorOutOfWorkZone)+", errorMsg = '"+String(armServoAngles.errorMsg)+"'. ");
    }
  }
  return armServoAngles;
}

//----------------------------------------------------------------------------------------
ArmServoAngles InverseKinematics::moveToPosXYZ(GripPositionXYZ positionXYZ) {
  if(positionXYZ.errorOutOfWorkZone == true) {
    Serial.println("IK::moveToPosXYZ: @1  positionXYZ.errorOutOfWorkZone = true. fast return. End");
    return moveToAngle(0, 0, 0, positionXYZ.gripSpinAngle, 0, 0, positionXYZ.duration, positionXYZ.showLog, positionXYZ.errorOutOfWorkZone , positionXYZ.errorMsg);
  } else {
    #if defined(DEBUG)
      Serial.println("IK::moveToPosXYZ: InputParams positionXYZ  (X,Y,Z) = ("+String(positionXYZ.gripX)+", "+String(positionXYZ.gripY)+", "+String(positionXYZ.gripZ)+" ), Grip (Spin, Tilt, Width) = ("+String(positionXYZ.gripSpinAngle) +", "+String(positionXYZ.gripTiltAngle)+", "+String(positionXYZ.gripWidth)+"), duration="+String(positionXYZ.duration)+", movesScriptEnd = "+String(positionXYZ.movesScriptEnd));
    #endif
    double b = 0;
    double l = 0;
    double h = 0;
    double phi = 0;
    double theta = 0;
    double a1 = 0;
    double a2 = 0;
    double  beta =0 ;  //angle between arms (bones).  0=straight (full length) , 180 = fully bended (minimal distance between base and endpoint)
    double newGripTiltAngle = 0;
    double gripAngle = 0;

    if(positionXYZ.gripZ < -110) {
      Serial.print("IK::moveToPosXYZ: WARNING. Z is smaller than minimum !!!");
      positionXYZ.errorOutOfWorkZone = true;
      positionXYZ.errorMsg = "Z too Small";

    }
    //------------------
    b = atan2(positionXYZ.gripY,positionXYZ.gripX) * M_180_DIV_PI; //(180 / MATH_PI); // base angle

    l = sqrt(positionXYZ.gripX * positionXYZ.gripX + positionXYZ.gripY * positionXYZ.gripY); // x and y extension 
        
    if(l < 0) {
      Serial.print("IK::moveToPosXYZ: WARNING. l is smaller than zero !!!h Unable to use l. ");
      positionXYZ.errorOutOfWorkZone = true;
      positionXYZ.errorMsg = "l Negative";
    }
    if(positionXYZ.gripZ<55) {
      if( l < 30) {
        Serial.print("IK::moveToPosXYZ: WARNING. l is smaller than minimum !!!h Unable to use l. ");
        positionXYZ.errorOutOfWorkZone = true;
        positionXYZ.errorMsg = "l too small.1";
      }
    } else {
      if( l < 8) {
        Serial.print("IK::moveToPosXYZ: WARNING. l is smaller than minimum !!!h Unable to use l. ");
        positionXYZ.errorOutOfWorkZone = true;
        positionXYZ.errorMsg = "l too small.2";
      }
    }
    if(l==0) {
      //l=0.2;
      Serial.print("IK::moveToPosXYZ: WARNING l=0!!! Unable to use l as divider. ");
      positionXYZ.errorOutOfWorkZone = true;
      positionXYZ.errorMsg = "l==0";
    }
    if(l > (2*OneArmLength)) {
      Serial.print("IK::moveToPosXYZ: WARNING. l is longer than arm Length !!!h Unable to use l. ");
      positionXYZ.errorOutOfWorkZone = true;
      positionXYZ.errorMsg = "l> 2*ArmLength";
    }
    if(positionXYZ.errorOutOfWorkZone == true) {

    } else {
      h = sqrt (l*l + positionXYZ.gripZ * positionXYZ.gripZ);

      if(h > (2*OneArmLength)) {
        Serial.print("IK::moveToPosXYZ: WARNING. h is longer than arm Length !!!h Unable to use h. ");
        positionXYZ.errorOutOfWorkZone = true;
        positionXYZ.errorMsg = "h> 2*ArmLength";
      }

      phi = atan(positionXYZ.gripZ/l) * M_180_DIV_PI;//(180 / MATH_PI);

      theta = acos((h/2)/OneArmLength) * M_180_DIV_PI; //(180 / MATH_PI);    //120 mm = length of first and second part of arm (120 = brown and 120 = white arm with black bracket)
    
      a1 = phi + theta; // angle for first part of the arm
      //a1 = 74;
      a2 = phi - theta; // angle for second part of the arm against horizont level.
      beta = 180 - theta - theta;
      //a2 =  (180 - a1) + theta;
    //double a2 =  theta - a1;
    //pictures here :  https://www.youtube.com/watch?v=Q-UeYEpwXXU

      //newGripTiltAngle = (positionXYZ.gripTiltAngle - (1.1 * a1)) +(0.8 * beta);
      newGripTiltAngle = (positionXYZ.gripTiltAngle + (a1 + (beta - 180))) + 70;
      //double newGripTiltAngle = positionXYZ.gripTiltAngle;

      gripAngle = asin((positionXYZ.gripWidth/2)/30) * M_180_DIV_PI;//(180 / MATH_PI);    
    }
    if (positionXYZ.showLog) {
      Serial.print("IK::moveToPosXYZ() @END_OK b:"+String(b)+", l:"+String(l)+", h:"+String(h)+", phi:"+String(phi)+", theta:"+String(theta)+", a1:"+String(a1)+", beta:"+String(beta)+", a2:"+String(a2)+", g:"+String(gripAngle)+".");
    }
  
    return moveToAngle(b, a1, beta, positionXYZ.gripSpinAngle, newGripTiltAngle, gripAngle, positionXYZ.duration, positionXYZ.showLog, positionXYZ.errorOutOfWorkZone , positionXYZ.errorMsg);
  }
}
