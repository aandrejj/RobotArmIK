#ifndef RampOnce_h
#define RampOnce_h

#include "Arduino.h"
#include <Ramp.h>                             // include library

//https://github.com/siteswapjuggler/RAMP/tree/master

class RampOnce {

public:
		RampOnce();
		int lastValue;
		double duration;
    int stepsCount;

		double speed; //milimeters in miliseconds
		
		double trajectoryLength;
		rampDouble myRamp;                               // new int ramp object

		void rampSetup(double duration, int stepsCount);
		int rampLoop();


private:

};
#endif
