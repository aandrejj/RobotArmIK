#ifndef RampServos_h
#define RampServos_h

#include "Arduino.h"
#include <Ramp.h>
#include <Servo.h>
#include <ServoRamp.h>
#include "RoboArmTurn.h"

class RampServos{

public:
	ServoRamp servo01; //zakladna
	ServoRamp servo02; //spodne hnede rameno
	ServoRamp servo03; //horne  biela rameno
	ServoRamp servo04; //ruka nabok  100 = zhruba vodorovne
	ServoRamp servo05; //ruka hore
	ServoRamp servo06; //ruka otvorena= 100, zatvorena = 60
	//Servo servos[7];
	//RampServos(Servo [], int servosCount);
	//RampServos(ServoRamp pRampServo01, ServoRamp pRampServo02, ServoRamp pRampServo03, ServoRamp pRampServo04, ServoRamp pRampServo05, ServoRamp pRampServo06);
	//RampServos(Servo pServo01, Servo pServo02, Servo pServo03, Servo pServo04, Servo pServo05, Servo pServo06);
	RampServos();
	//void setRampServos(Servo &pServo01, Servo &pServo02, Servo &pServo03, Servo &pServo04, Servo &pServo05, Servo &pServo06);
	//ArmServoAngles updateCurrentAngles(ArmServoAngles oldServoAngles);
	//void updateServos();
	
private:

};
#endif