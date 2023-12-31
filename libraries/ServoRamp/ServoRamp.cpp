#include "Arduino.h"
#include "ServoRamp.h"

/*
ServoRamp::ServoRamp(Servo pRampServo01, Servo pRampServo02, Servo pRampServo03, Servo pRampServo04, Servo pRampServo05, Servo pRampServo06) {
  rampServo01 = pRampServo01; //zakladna
  rampServo02 = pRampServo02; //spodne hnede rameno
  rampServo03 = pRampServo03; //horne  biela rameno
  rampServo04 = pRampServo04; //ruka nabok  100 = zhruba vodorovne
  rampServo05 = pRampServo05; //ruka hore
  rampServo06 = pRampServo06; //ruka otvorena= 100, zatvorena = 60
}
*/

ServoRamp::ServoRamp() {
	//servo = pServo;
}

void ServoRamp::setServo(Servo pServo) {
	servo = pServo;
}

int ServoRamp::updateServo() {
  int val = myAngle.update();           // update ramp value
  Serial.println(myAngle.getValue());   // prompt angle value

  //Serial.print("updateServo: servoNumber = ");
  //Serial.print(servoNumber);
  Serial.print(" , myAngle = ");
  Serial.print(myAngle.getValue());   // prompt angle value

  //mysrvo.write(myAngle.getValue());    // transmit it to the servo
  return myAngle.getValue();
}

void ServoRamp::updateJoystick(bool init = false) {
  int  val = map(analogRead(JOYSTICK_PIN), 0, 1023, 0, 180);              // read and map joystick value
  uint32_t duration = 1000.* (float)abs(myAngle.getValue()-val)/SPEED;    // calculate ramp duration in ms according to speed
  
  if (init) {                                                             // start with going to the initial position
    myAngle.go(val);
    updateServo();
    delay(duration);
    return;
  } 
  
  if (abs(val-previousAngle) >=2) {
    myAngle.go(val, duration);                                            // set next ramp interpolation in ms
    previousAngle = val;                                                  // save previous val
  }
}
