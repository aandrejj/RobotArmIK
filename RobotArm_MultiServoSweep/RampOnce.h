#ifndef RampOnce_h
#define RampOnce_h

#include "Arduino.h"
#include <Ramp.h>                             // include library
//#define DEBUG         //debug logging
#define BRIEF_LOG     //just a few logs

//https://github.com/siteswapjuggler/RAMP/tree/master

class RampOnce {

public:
		RampOnce();
		int lastValue;
		double duration;
    int stepsCount;

		double speed; //milimeters in miliseconds
		
		double trajectoryLength;
		ramp myRamp;                               // new int ramp object
    float rampCompletion;       // get current ramp completion percentage [0.-100.]
    bool rampIsFinished;                          // is the ramp finished
    bool rampIsRunning;                           // is the ramp running


		void rampSetup(double duration, int stepsCount);
		double update();


private:

};
#endif
