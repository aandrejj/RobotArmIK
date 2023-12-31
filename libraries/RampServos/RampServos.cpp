#include "Arduino.h"
#include "RampServos.h"
#include "ServoRamp.h"
#include <SoftwareSerial.h>


RampServos::RampServos() {

}
/*
void RampServos::setRampServos(Servo &pServo01, Servo &pServo02, Servo &pServo03, Servo &pServo04, Servo &pServo05, Servo &pServo06) {
	Serial.println("RampServos::setRampServos: Started.");
	servo01.setServo(pServo01); //zakladna
	servo02.setServo(pServo02); //spodne hnede rameno
	servo03.setServo(pServo03); //horne  biela rameno
	servo04.setServo(pServo04); //ruka nabok  100 = zhruba vodorovne
	servo05.setServo(pServo05); //ruka hore
	servo06.setServo(pServo06); //ruka otvorena= 100, zatvorena = 60
	Serial.println("RampServos::setRampServos: End.");
}

//--------------------updateServos()--------------------------------
void RampServos::updateServos() {
  //for (int servoNumber = 1; servoNumber <= 6; servoNumber++) {
    //rampServos[servoNumber].updateServo();
  //}
  servo01.updateServo();
  servo02.updateServo();
  servo03.updateServo();
  servo04.updateServo();
  servo05.updateServo();
  servo06.updateServo();
  
}
 //--------------------end of updateServo-----------------------------

ArmServoAngles RampServos::updateCurrentAngles(ArmServoAngles oldServoAngles) {
	Serial.println("updateCurrentAngles: Started");
	ArmServoAngles newServoAngles;

	
	return newServoAngles;
}
*/