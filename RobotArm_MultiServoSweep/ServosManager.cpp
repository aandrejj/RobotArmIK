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
    
        //pwm.setPWM( 1, 0, armServoAngles.baseAngle);
        //pwm.setPWM( 2, 0, armServoAngles.arm1Angle);

        //pwm.setPWM( 3, 0, armServoAngles.arm2Angle);
        //pwm.setPWM( 4, 0, armServoAngles.gripSpinAngle);
        //pwm.setPWM( 5, 0, armServoAngles.gripTiltAngle);

        //pwm.setPWM( 6, 0, armServoAngles.gripAngle);

    if(armServoAngles.errorOutOfWorkZone==true) {
      Serial.println("ServosManager::updateServos(): SKIPPED errorOutOfWorkZone, errorMsg ="+String(armServoAngles.errorMsg)+". Using previous position.");
      pwm.setPWM( 1, 0, angleToPulse(previousArmServoAngles.baseAngle, SERVOMIN1, SERVOMAX1));
      pwm.setPWM( 2, 0, angleToPulse(previousArmServoAngles.arm1Angle, SERVOMIN2, SERVOMAX2));

      pwm.setPWM( 3, 0, angleToPulse(previousArmServoAngles.arm2Angle, SERVOMIN3, SERVOMAX3));
      pwm.setPWM( 4, 0, angleToPulse(previousArmServoAngles.gripSpinAngle, SERVOMIN4, SERVOMAX4));
      pwm.setPWM( 5, 0, angleToPulse(previousArmServoAngles.gripTiltAngle, SERVOMIN5, SERVOMAX5));

      pwm.setPWM( 6, 0, angleToPulse(previousArmServoAngles.gripAngle, SERVOMIN6, SERVOMAX6));
    } 
    else 
    {
      #ifdef DEBUG 
	      Serial.println("ServosManager::updateServos(): armServoAngles:  baseAngle = "+String(armServoAngles.baseAngle)+", arm1Angle = "+ String(armServoAngles.arm1Angle)+", arm2Angle = "+String(armServoAngles.arm2Angle)+", gripSpinAngle = "+ String(armServoAngles.gripSpinAngle)+", gripTiltAngle = "+String(armServoAngles.gripTiltAngle)+", gripAngle = "+String(armServoAngles.gripAngle)+"." );
      #endif
      if(armServoAngles.baseAngle != previousArmServoAngles.baseAngle) {
        pwm.setPWM( 1, 0, angleToPulse(armServoAngles.baseAngle, SERVOMIN1, SERVOMAX1));
      }
      if(armServoAngles.arm1Angle != previousArmServoAngles.arm1Angle) {
        pwm.setPWM( 2, 0, angleToPulse(armServoAngles.arm1Angle, SERVOMIN2, SERVOMAX2));
      }
      if(armServoAngles.arm2Angle != previousArmServoAngles.arm2Angle) {
        pwm.setPWM( 3, 0, angleToPulse(armServoAngles.arm2Angle, SERVOMIN3, SERVOMAX3));
      }
      if(armServoAngles.gripSpinAngle != previousArmServoAngles.gripSpinAngle) {
        pwm.setPWM( 4, 0, angleToPulse(armServoAngles.gripSpinAngle, SERVOMIN4, SERVOMAX4));
      }
      if(armServoAngles.gripTiltAngle != previousArmServoAngles.gripTiltAngle) {
        pwm.setPWM( 5, 0, angleToPulse(armServoAngles.gripTiltAngle, SERVOMIN5, SERVOMAX5));
      }
      if(armServoAngles.gripAngle != previousArmServoAngles.gripAngle) {
        pwm.setPWM( 6, 0, angleToPulse(armServoAngles.gripAngle, SERVOMIN6, SERVOMAX6));
      }
      previousArmServoAngles = armServoAngles;
    }
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
    previousArmServoAngles.errorOutOfWorkZone = false;
    previousArmServoAngles.errorMsg = "";

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
int ServosManager::angleToPulse(int ang, long servoMin, long servoMax){
   int pulse = map(ang,0, 180, servoMin, servoMax);// SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   
   #if defined(DEBUG)
    Serial.print("Angle: ");Serial.print(ang);
    Serial.print(" pulse: ");Serial.println(pulse);
   #endif
   return pulse;
}
