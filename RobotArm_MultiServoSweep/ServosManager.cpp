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

        //pwm.setPWM( 0, 0, servo01_Angle);  
        pwm.setPWM( 1, 0, armServoAngles.baseAngle);  
        pwm.setPWM( 2, 0, armServoAngles.arm1Angle);

        pwm.setPWM( 3, 0, armServoAngles.arm2Angle);
        pwm.setPWM( 4, 0, armServoAngles.gripSpinAngle);
        pwm.setPWM( 5, 0, armServoAngles.gripTiltAngle);

        pwm.setPWM( 6, 0, armServoAngles.gripAngle);
        //pwm.setPWM( 7, 0, servo02_Angle);  //Servo 1
        //pwm.setPWM( 8, 0, servo03_Angle);  //Servo 2
        
    //servo01.write(armServoAngles.baseAngle);
    //servo02.write(armServoAngles.arm1Angle);
    //servo03.write(armServoAngles.arm2Angle);
    //servo04.write(armServoAngles.gripSpinAngle);
    //servo05.write(armServoAngles.gripTiltAngle);
    //servo06.write(armServoAngles.gripAngle);
  }
  //--------------------end of updateServo-----------------------------
  
  /*
	//--------------------updateServos_msec()--------------------------------
	//ToDo Here
	void ServosManager::updateServos_msec(ArmServoMicrosec armServoMicrosec) {
	  //ToDo Here
    #ifdef DEBUG 
	  Serial.println("ServosManager::updateServos_msec(): armServoMicrosec:  baseMicrosec = "+String(armServoMicrosec.baseMicrosec)+", arm1Microsec = "+ String(armServoMicrosec.arm1Microsec)+", arm2Microsec = "+String(armServoMicrosec.arm2Microsec)+", gripSpinMicrosec = "+ String(armServoMicrosec.gripSpinMicrosec)+", gripTiltMicrosec = "+String(armServoMicrosec.gripTiltMicrosec)+", gripMicrosec = "+String(armServoMicrosec.gripMicrosec)+"." );
    #endif

        //pwm.setPWM( 0, 0, servo01_Angle);  
        pwm.setPWM( 1, 0, armServoMicrosec.baseMicrosec);  
        pwm.setPWM( 2, 0, armServoMicrosec.arm1Microsec);

        pwm.setPWM( 3, 0, armServoMicrosec.arm2Microsec);
        pwm.setPWM( 4, 0, armServoMicrosec.gripSpinMicrosec);
        pwm.setPWM( 5, 0, armServoMicrosec.gripTiltMicrosec);

        pwm.setPWM( 6, 0, armServoMicrosec.gripMicrosec);
        //pwm.setPWM( 7, 0, servo02_Angle);  //Servo 1
        //pwm.setPWM( 8, 0, servo03_Angle);  //Servo 2

	  //servo01.writeMicroseconds(armServoMicrosec.baseMicrosec);
	  //servo02.writeMicroseconds(armServoMicrosec.arm1Microsec);
	  //servo03.writeMicroseconds(armServoMicrosec.arm2Microsec);
	  //servo04.writeMicroseconds(armServoMicrosec.gripSpinMicrosec);
	  //servo05.writeMicroseconds(armServoMicrosec.gripTiltMicrosec);
	  //servo06.writeMicroseconds(armServoMicrosec.gripMicrosec);
	  
	}
	//--------------------end of updateServo_msec-----------------------------
  */

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
	  delay(200);
   //https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
   //value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
   //so that servos often respond to values between 700 and 2300.
   
    //servo01.attach( 1); //zakladna
    //servo02.attach( 2);//spodne hnede rameno
    //servo03.attach( 3);//horne  biela rameno
    //servo04.attach( 4);//ruka nabok  100 = zhruba vodorovne
    //servo05.attach( 5);//ruka hore
    //servo06.attach( 6);//ruka otvorena= 100, zatvorena = 60

    /*
	  servo01.attach( 5, pServo_Min_milisec ,pServo_Max_milisec); //zakladna
	  servo02.attach( 6, pServo_Min_milisec ,pServo_Max_milisec);//spodne hnede rameno
	  servo03.attach( 7, pServo_Min_milisec ,pServo_Max_milisec);//horne  biela rameno
	  servo04.attach( 8, pServo_Min_milisec ,pServo_Max_milisec);//ruka nabok  100 = zhruba vodorovne
	  servo05.attach( 9, pServo_Min_milisec ,pServo_Max_milisec);//ruka hore
	  servo06.attach(10, pServo_Min_milisec ,pServo_Max_milisec);//ruka otvorena= 100, zatvorena = 60
    */
	  #ifdef DEBUG 
	    //Serial.println("ServosManager::ServoInitialization: Servos attached");
      Serial.println("ServosManager::ServoInitialization:  Robot arm initial position");
    #endif
	  
	  delay(20);
  
        //pwm.setPWM( 0, 0, servo01_Angle);  
        pwm.setPWM( 1, 0, pservo1Pos);  
        pwm.setPWM( 2, 0, pservo2Pos);

        pwm.setPWM( 3, 0, pservo3Pos);
        pwm.setPWM( 4, 0, pservo4Pos);
        pwm.setPWM( 5, 0, pservo5Pos);

        pwm.setPWM( 6, 0, pservo6Pos);
        //pwm.setPWM( 7, 0, servo02_Angle);  //Servo 1
        //pwm.setPWM( 8, 0, servo03_Angle);  //Servo 2
      
    /*
	  servo01.write(pservo1Pos);
	  //Serial.println("Servo01 positioned");
	  //delay(100);

	  servo02.write(pservo2Pos);
	  //Serial.println("Servo02 positioned");
	  //delay(100);
	  
	  servo03.write(pservo3Pos);
	  //Serial.println("Servo03 positioned");
	  //delay(100);
	  
	  servo04.write(pservo4Pos);
	  //Serial.println("Servo04 positioned");
	  //delay(100);
	  
	  servo05.write(pservo5Pos);
	  //Serial.println("Servo05 positioned");
	  //delay(100);
	  
	  servo06.write(pservo6Pos);
	  //Serial.println("Servo06 positioned");
	  //delay(100);
    */ 
	  #if defined(DEBUG) || defined(BRIEF_LOG) 
	    Serial.println("ServosManager::ServoInitialization: Servos positioned to default positions");
      #endif
    
	  
	  ArmServoAngles currentServoAngles;
	  
	  currentServoAngles.baseAngle      = pservo1Pos; //zakladna
	  currentServoAngles.arm1Angle      = pservo2Pos; //spodne hnede rameno
	  currentServoAngles.arm2Angle      = pservo3Pos; //horne  biela rameno
	  currentServoAngles.gripSpinAngle  = pservo4Pos; //ruka nabok
	  currentServoAngles.gripTiltAngle  = pservo5Pos; //ruka hore
	  currentServoAngles.gripAngle      = pservo6Pos; //ruka otvorena
    currentServoAngles.duration       = 2000;
	  currentServoAngles.movesScriptEnd = false;

#if defined(DEBUG) || defined(BRIEF_LOG) 
      Serial.println("ServosManager::ServoInitialization: currentServoAngles:  baseAngle = "+String(currentServoAngles.baseAngle)+", arm1Angle = "+ String(currentServoAngles.arm1Angle)+", arm2Angle = "+String(currentServoAngles.arm2Angle)+", gripSpinAngle = "+ String(currentServoAngles.gripSpinAngle)+", gripTiltAngle = "+String(currentServoAngles.gripTiltAngle)+", gripAngle = "+String(currentServoAngles.gripAngle)+"." );
	    Serial.println("ServosManager::ServoInitialization: Initialization OK.");
      #endif
	  return currentServoAngles;
	}
