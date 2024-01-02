#ifndef LinearRampXYZ_h
#define LinearRampXYZ_h

#include "Arduino.h"
#include <Ramp.h>                             // include library
#include "ServosManager.h"
#include "RampOnce.h"
//#define DEBUG         //debug logging
#define BRIEF_LOG     //just a few logs

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

    double lenX;
    double lenY;
    double lenZ;

		double trajectoryLength;

    //double incrementX;
    //double incrementY;
    //double incrementZ;
    GripPositionXYZ increment;

		
		RampOnce rampOnce;                               // new int ramp object
    
    
    int currentStep;
    GripPositionXYZ fromPos;
    GripPositionXYZ toPos;
    GripPositionXYZ currentPos;
    GripPositionXYZ distances;
    GripPositionXYZ speeds;
    
    void begin(GripPositionXYZ fromPos, GripPositionXYZ toPos, double pSpeed);
		void setup();
		GripPositionXYZ update();

private:

};
#endif
