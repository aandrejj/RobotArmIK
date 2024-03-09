#include "Arduino.h"
#include "ServosManager.h"
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

	ServosManager::ServosManager() {
	}
    
  void ServosManager::begin(){

    #ifdef DEBUG 
	    Serial.println("ServosManager.begin: PCA9685 initialization. pwm.begin");
    #endif

    pwm.begin(); //pwm.begin(0);   0 = driver_ID
    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

    #ifdef DEBUG 
	    Serial.println("ServosManager.begin: OK. PCA9685 initialized.");
    #endif
  }
  //--------------------updateServos()--------------------------------
  void ServosManager::updateServos(ArmServoAngles armServoAngles) {
    #ifdef DEBUG 
	  Serial.println("ServosManager::updateServos(): armServoAngles:  baseAngle = "+String(armServoAngles.baseAngle)+", arm1Angle = "+ String(armServoAngles.arm1Angle)+", arm2Angle = "+String(armServoAngles.arm2Angle)+", gripSpinAngle = "+ String(armServoAngles.gripSpinAngle)+", gripTiltAngle = "+String(armServoAngles.gripTiltAngle)+", gripAngle = "+String(armServoAngles.gripAngle)+"." );
    #endif
        //pwm.setPWM( 1, 0, armServoAngles.baseAngle);
        //pwm.setPWM( 2, 0, armServoAngles.arm1Angle);

        //pwm.setPWM( 3, 0, armServoAngles.arm2Angle);
        //pwm.setPWM( 4, 0, armServoAngles.gripSpinAngle);
        //pwm.setPWM( 5, 0, armServoAngles.gripTiltAngle);

        //pwm.setPWM( 6, 0, armServoAngles.gripAngle);

    if(armServoAngles.baseAngle != previousArmServoAngles.baseAngle) {
      pwm.setPWM( 1, 0, angleToPulse(armServoAngles.baseAngle));
    }
    if(armServoAngles.arm1Angle != previousArmServoAngles.arm1Angle) {
      pwm.setPWM( 2, 0, angleToPulse(armServoAngles.arm1Angle));
    }
    if(armServoAngles.arm2Angle != previousArmServoAngles.arm2Angle) {
      pwm.setPWM( 3, 0, angleToPulse(armServoAngles.arm2Angle));
    }
    if(armServoAngles.gripSpinAngle != previousArmServoAngles.gripSpinAngle) {
      pwm.setPWM( 4, 0, angleToPulse(armServoAngles.gripSpinAngle));
    }
    if(armServoAngles.gripTiltAngle != previousArmServoAngles.gripTiltAngle) {
      pwm.setPWM( 5, 0, angleToPulse(armServoAngles.gripTiltAngle));
    }
    if(armServoAngles.gripAngle != previousArmServoAngles.gripAngle) {
      pwm.setPWM( 6, 0, angleToPulse(armServoAngles.gripAngle));
    }
    previousArmServoAngles = armServoAngles;
  }
  //--------------------end of updateServo-----------------------------

	ArmServoAngles ServosManager::updateCurrentAngles(ArmServoAngles oldServoAngles) {
		#ifdef DEBUG 
		  Serial.println("updateCurrentAngles: Started");
        #endif
		ArmServoAngles newServoAngles;

		return newServoAngles;
	}

	//---------------SevoInitialization----------------------------------
	ArmServoAngles ServosManager::ServoInitialization(int pservo1Pos, int pservo2Pos, int pservo3Pos, int pservo4Pos, int pservo5Pos, int pservo6Pos, int pServo_Min_milisec, int pServo_Max_milisec) {
	  #if defined(DEBUG) || defined(BRIEF_LOG) 
	    Serial.println("ServosManager::ServoInitialization: Initialization started");
      #endif
	  //delay(200);
   //https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
   //value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
   //so that servos often respond to values between 700 and 2300.
   
	  #ifdef DEBUG 
      Serial.println("ServosManager::ServoInitialization:  Robot arm initial position");
    #endif
	  
	  delay(20);
    pwm.setPWM( 1, 0, angleToPulse(pservo1Pos));
    pwm.setPWM( 2, 0, angleToPulse(pservo2Pos));
    pwm.setPWM( 3, 0, angleToPulse(pservo3Pos));
    pwm.setPWM( 4, 0, angleToPulse(pservo4Pos));
    pwm.setPWM( 5, 0, angleToPulse(pservo5Pos));
    pwm.setPWM( 6, 0, angleToPulse(pservo6Pos));
      
	  #if defined(DEBUG) || defined(BRIEF_LOG) 
	    //Serial.println("ServosManager::ServoInitialization: Servos positioned to default positions");
    #endif
    
    previousArmServoAngles.baseAngle      = pservo1Pos; //zakladna
	  previousArmServoAngles.arm1Angle      = pservo2Pos; //spodne hnede rameno
	  previousArmServoAngles.arm2Angle      = pservo3Pos; //horne  biela rameno
	  previousArmServoAngles.gripSpinAngle  = pservo4Pos; //ruka nabok
	  previousArmServoAngles.gripTiltAngle  = pservo5Pos; //ruka hore
	  previousArmServoAngles.gripAngle      = pservo6Pos; //ruka otvorena
    previousArmServoAngles.duration       = 200;
	  previousArmServoAngles.movesScriptEnd = false;

	  ArmServoAngles currentServoAngles;
	  
	  currentServoAngles.baseAngle      = pservo1Pos; //zakladna
	  currentServoAngles.arm1Angle      = pservo2Pos; //spodne hnede rameno
	  currentServoAngles.arm2Angle      = pservo3Pos; //horne  biela rameno
	  currentServoAngles.gripSpinAngle  = pservo4Pos; //ruka nabok
	  currentServoAngles.gripTiltAngle  = pservo5Pos; //ruka hore
	  currentServoAngles.gripAngle      = pservo6Pos; //ruka otvorena
    currentServoAngles.duration       = 200;
	  currentServoAngles.movesScriptEnd = false;

    #if defined(DEBUG) || defined(BRIEF_LOG) 
      Serial.println("ServosManager::ServoInitialization: currentServoAngles:  baseAngle = "+String(currentServoAngles.baseAngle)+", arm1Angle = "+ String(currentServoAngles.arm1Angle)+", arm2Angle = "+String(currentServoAngles.arm2Angle)+", gripSpinAngle = "+ String(currentServoAngles.gripSpinAngle)+", gripTiltAngle = "+String(currentServoAngles.gripTiltAngle)+", gripAngle = "+String(currentServoAngles.gripAngle)+"." );
	    Serial.println("ServosManager::ServoInitialization: Initialization OK.");
    #endif
	  return currentServoAngles;
	}


/*
 * angleToPulse(int ang)
 * gets angle in degree and returns the pulse width
 * also prints the value on seial monitor
 * written by Ahmad Nejrabi for Robojax, Robojax.com
 */
int ServosManager::angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   
   #if defined(DEBUG)
    Serial.print("Angle: ");Serial.print(ang);
    Serial.print(" pulse: ");Serial.println(pulse);
   #endif
   return pulse;
}
