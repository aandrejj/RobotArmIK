#ifndef LinearRampXYZ_h
#define LinearRampXYZ_h

#include "Arduino.h"
#include <Ramp.h>                             // include library

//https://github.com/siteswapjuggler/RAMP/tree/master

class LinearRampXYZ {

public:
		LinearRampXYZ();
		int lastValue;
		double duration;

		double speed; //milimeters in miliseconds
		double speedX;
		double speedY;
		double speedZ;
		
		
		double trajectoryLength;
		rampDouble myRamp;                               // new int ramp object

		double fromX;
		double fromY;
		double fromZ;

		double toX;
		double toY;
		double toZ;
		void begin(double fromX, double fromY, double fromZ, double toX, double toY, double toZ, double pSpeed);
		void rampSetup();
		void rampLoop();


private:

};
#endif
