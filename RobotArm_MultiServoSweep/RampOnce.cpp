#include "Arduino.h"
#include "RampOnce.h"
 //https://github.com/siteswapjuggler/RAMP/tree/master
 	
	RampOnce::RampOnce() {
	}
	
	void RampOnce::rampSetup(double duration, int stepsCount) {
		if(stepsCount==0) {
			RampOnce::stepsCount = 100;
		} else {
			RampOnce::stepsCount = stepsCount;
		}		
  Serial.println("RampOnce::rampSetup(): RampOnce::stepsCount = "+String(RampOnce::stepsCount)+".");

		RampOnce::duration = duration;
  Serial.println("RampOnce::rampSetup(): RampOnce::duration = "+String(RampOnce::duration)+".");
		
	  myRamp.setGrain(2);                         // set grain to 1 ms
    myRamp.go(0);
    Serial.print("RampOnce::rampSetup: Value start at: ");         //
    Serial.println(myRamp.getValue());        // accessing start value
	  ////myRamp.go(new_value, (ramp_duration_in_ms), (ramp_mode), (loop_mode));
	  ////myRamp.go(5000);                            // set value to directly to 5000
	  ////myRamp.go(0, 15, NONE, LOOPFORWARD);      // go to 0 in 15 ms

	  //myRamp.go(0);                            // set value to 0
    //myRamp.go((double)RampOnce::stepsCount, (unsigned long)RampOnce::duration, ONCEFORWARD, BOUNCE_INOUT);
    myRamp.go((double)RampOnce::stepsCount, (unsigned long)RampOnce::duration, LINEAR, ONCEFORWARD);
	}
	
	double RampOnce::update() {
	  double val = myRamp.update();                  // store updated value
	  if (val != lastValue) {                     // print updated value and completion percentage
      RampOnce::rampCompletion = myRamp.getCompletion();
      ////Serial.println("RampOnce::update: val ="+String(val)+" \t|\t rampCompletion = "+String(rampCompletion)+" %");
      Serial.println("RampOnce::update: rampCompletion = "+String(rampCompletion)+" %");
	  }
	  lastValue = val;
    RampOnce::rampIsFinished = myRamp.isFinished();                          // is the ramp finished
    RampOnce::rampIsRunning = myRamp.isRunning();                           // is the ramp running

	  return val;
	}
