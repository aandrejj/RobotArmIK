#include "Arduino.h"
#include "RampServos.h"


RampServos::RampServos() {
  
}

void RampServos::updateServo() {
  int val = myAngle.update();           // update ramp value
  Serial.println(myAngle.getValue());   // prompt angle value
  myservo.write(myAngle.getValue());    // transmit it to the servo
}

void RampServos::updateJoystick(bool init = false) {
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
