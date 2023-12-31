#include "Arduino.h"
#include "RampOnce.h"
 //https://github.com/siteswapjuggler/RAMP/tree/master
 	
	void RampOnce::RampOnce() {
	}
	
	void RampOnce::rampSetup(double duration, int stepsCount) {
		if(stepsCount==0) {
			RampOnce::stepsCount = 100;
		} else {
			RampOnce::stepsCount = stepsCount;
		}		
		RampOnce::duration = duration;
		
	  myRamp.setGrain(1);                         // set grain to 1 ms
	  //myRamp.go(new_value, (ramp_duration_in_ms), (ramp_mode), (loop_mode));
	  //myRamp.go(5000);                            // set value to directly to 5000
	  //myRamp.go(0, 15, LINEAR, LOOPFORWARD);      // go to 0 in 15 ms

	  myRamp.go(0);                            // set value to 0
	  myRamp.go(RampOnce::stepsCount, RampOnce::duration, ONCEFORWARD, BOUNCE_INOUT);
	}
	
	int RampOnce::rampLoop() {
	  int val = myRamp.update();                  // store updated value
	  if (val != lastValue) {                     // print updated value and completion percentage
		Serial.print(val);
		Serial.print("\t|\t");
		Serial.print(myRamp.getCompletion());
		Serial.print(" %");
		Serial.println();
	  }
	  lastValue = val;
	  return val;
	}




