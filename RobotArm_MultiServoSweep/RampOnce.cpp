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
		#if defined(DEBUG) || defined(BRIEF_LOG) 
		  Serial.println("RampOnce::rampSetup(): RampOnce::stepsCount = "+String(RampOnce::stepsCount)+".");
   #endif

		RampOnce::duration = duration;
		//#ifdef DEBUG 
    #if defined(DEBUG) || defined(BRIEF_LOG)
		   Serial.println("RampOnce::rampSetup(): RampOnce::duration = "+String(RampOnce::duration)+".");
    #endif
			
		myRamp.setGrain(1);                         // set grain to 1 ms
		myRamp.go(0);
		#ifdef DEBUG 
		  Serial.print("RampOnce::rampSetup: Value start at: ");         //
		  Serial.println(myRamp.getValue());        // accessing start value
    #endif
    
		//myRamp.go((double)RampOnce::stepsCount, (unsigned long)RampOnce::duration, ONCEFORWARD, BOUNCE_INOUT);
		myRamp.go((double)RampOnce::stepsCount, (unsigned long)RampOnce::duration, LINEAR, ONCEFORWARD);
	}
	
	double RampOnce::update() {
	  double val = myRamp.update();                  // store updated value
	  if (val != lastValue) {                     // print updated value and completion percentage
      RampOnce::rampCompletion = myRamp.getCompletion();
      #if defined(DEBUG) 
        //Serial.println("RampOnce::update: val ="+String(val)+" \t|\t rampCompletion = "+String(rampCompletion)+" %");
        Serial.println("RampOnce::update: rampCompletion = "+String(rampCompletion)+" %");
      #endif
	  }
	  lastValue = val;
    RampOnce::rampIsFinished = myRamp.isFinished();                          // is the ramp finished
    RampOnce::rampIsRunning = myRamp.isRunning();                           // is the ramp running

	  return val;
	}
