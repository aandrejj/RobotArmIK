#include "Arduino.h"
#include "LinearRampXYZ.h"
 //https://github.com/siteswapjuggler/RAMP/tree/master
 
	LinearRampXYZ::LinearRampXYZ() {
	}
	
	void LinearRampXYZ::begin(double fromX, double fromY, double fromZ, double toX, double toY, double toZ, double pSpeed) {
		if(pSpeed==0) {
			LinearRampXYZ::speed = 1;
		} else {
			LinearRampXYZ::speed = pSpeed;
		}
		
		double lenX = abs(fromX - toX);
		double lenY = abs(fromY - toY);
		double lenZ = abs(fromZ - toZ);
		
		LinearRampXYZ::trajectoryLength = (int)sqrt( lenX*lenX + lenY*lenY + lenZ*lenZ);
		
		LinearRampXYZ::duration = LinearRampXYZ::trajectoryLength/LinearRampXYZ::speed;
		
		LinearRampXYZ::speedX = lenX/LinearRampXYZ::duration;
		LinearRampXYZ::speedY = lenY/LinearRampXYZ::duration;
		LinearRampXYZ::speedZ = lenZ/LinearRampXYZ::duration;
		
	}
	void LinearRampXYZ::rampSetup() {
	  myRamp.setGrain(1);                         // set grain to 1 ms
	  //myRamp.go(new_value, (ramp_duration_in_ms), (ramp_mode), (loop_mode));
	  //myRamp.go(5000);                            // set value to directly to 5000
	  //myRamp.go(0, 15, LINEAR, LOOPFORWARD);      // go to 0 in 15 ms

	  myRamp.go(0);                            // set value to 0
    //myRamp.go((double)LinearRampXYZ::trajectoryLength, (unsigned long)LinearRampXYZ::duration, ONCEFORWARD, BOUNCE_INOUT);
    myRamp.go((double)LinearRampXYZ::trajectoryLength, (unsigned long)LinearRampXYZ::duration);
	}
	
	void LinearRampXYZ::rampLoop() {
	  int val = myRamp.update();                  // store updated value
	  if (val != lastValue) {                     // print updated value and completion percentage
		Serial.print(val);
		Serial.print("\t|\t");
		Serial.print(myRamp.getCompletion());
		Serial.print(" %");
		Serial.println();
	  }
	  lastValue = val;
	}
