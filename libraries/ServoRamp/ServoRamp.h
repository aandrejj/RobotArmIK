/**
*ServoRamp
**/
#ifndef ServoRamp_h
#define ServoRamp_h

#include "Arduino.h"
#include <Ramp.h>
#include <Servo.h>

#define JOYSTICK_PIN  A0                // Joystick analog input
#define SPEED         60                // degree/s

class ServoRamp{
public:

	Servo servo;
	
	rampInt  myAngle;                       // create a byte ramp object to interpolate servo position
	byte  previousAngle = 0;
	ServoRamp();
	void setServo(Servo pServo);
	int updateServo();
	void updateJoystick(bool init = false);
private:
};
#endif
