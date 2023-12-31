#include "Arduino.h"
#include "LinearRampXYZ.h"
 //https://github.com/siteswapjuggler/RAMP/tree/master

static volatile unsigned int rampTotalStepsCount = 100;              // number of steps needed to reach end-position.
 
	LinearRampXYZ::LinearRampXYZ() {
	}
	
	void LinearRampXYZ::begin(GripPositionXYZ fromPos, GripPositionXYZ toPos, double pSpeed) {
		if(pSpeed==0) {
			LinearRampXYZ::speed = 1;
		} else {
			LinearRampXYZ::speed = pSpeed;
		}

    Serial.println("LinearRampXYZ::begin(): LinearRampXYZ::speed = "+String(LinearRampXYZ::speed)+".");

    Serial.println("LinearRampXYZ::begin(): fromPos = {"+ String(fromPos.gripX)+", "+ String(fromPos.gripY)+", "+ String(fromPos.gripZ)+", "+ String(fromPos.gripSpinAngle)+", "+ String(fromPos.gripTiltAngle)+", "+ String(fromPos.gripOpen)+","+ String(fromPos.movesScriptEnd)+"}.");
    Serial.println("LinearRampXYZ::begin(): toPos   = {"+ String(toPos.gripX)+", "+ String(toPos.gripY)+", "+ String(toPos.gripZ)+", "+ String(toPos.gripSpinAngle)+", "+ String(toPos.gripTiltAngle)+", "+ String(toPos.gripOpen)+","+ String(toPos.movesScriptEnd)+"}.");

    
    LinearRampXYZ::fromPos = fromPos;
    LinearRampXYZ::toPos = toPos;
    LinearRampXYZ::currentPos = fromPos;

    //int gripSpinAngle;
    //int gripTiltAngle;
    //int gripOpen;
		LinearRampXYZ::distances.gripX = double(toPos.gripX - fromPos.gripX);
		LinearRampXYZ::distances.gripY = double(toPos.gripY - fromPos.gripY);
		LinearRampXYZ::distances.gripZ = double(toPos.gripZ - fromPos.gripZ);

    LinearRampXYZ::distances.gripSpinAngle = double(toPos.gripSpinAngle - fromPos.gripSpinAngle);
    LinearRampXYZ::distances.gripTiltAngle = double(toPos.gripTiltAngle - fromPos.gripTiltAngle);
    LinearRampXYZ::distances.gripOpen      = double(toPos.gripOpen      - fromPos.gripOpen);

    
    Serial.println("LinearRampXYZ::begin(): LinearRampXYZ::distances.grip X,Y,Z,Spin,Tilt,Open   = {"+ String(LinearRampXYZ::distances.gripX)+", "+ String(LinearRampXYZ::distances.gripY)+", "+ String(LinearRampXYZ::distances.gripZ)+",Angles: "+ String(LinearRampXYZ::distances.gripSpinAngle)+", "+ String(LinearRampXYZ::distances.gripTiltAngle)+", "+ String(LinearRampXYZ::distances.gripOpen)+"}.");
		
		LinearRampXYZ::trajectoryLength = (int)sqrt( (LinearRampXYZ::distances.gripX*LinearRampXYZ::distances.gripX) + (LinearRampXYZ::distances.gripY*LinearRampXYZ::distances.gripY) + (LinearRampXYZ::distances.gripZ*LinearRampXYZ::distances.gripZ));
    if(LinearRampXYZ::trajectoryLength == 0) {
          Serial.println("LinearRampXYZ::begin(): LinearRampXYZ::trajectoryLength was 0, so recomputing by GripValues ...");
          LinearRampXYZ::trajectoryLength = (int)sqrt( (LinearRampXYZ::distances.gripSpinAngle * LinearRampXYZ::distances.gripSpinAngle) + (LinearRampXYZ::distances.gripTiltAngle * LinearRampXYZ::distances.gripTiltAngle) + (LinearRampXYZ::distances.gripOpen * LinearRampXYZ::distances.gripOpen));
    }
    
    Serial.println("LinearRampXYZ::begin(): LinearRampXYZ::trajectoryLength = "+String(LinearRampXYZ::trajectoryLength)+".");
		
		LinearRampXYZ::duration = int(LinearRampXYZ::trajectoryLength/LinearRampXYZ::speed);
    Serial.println("LinearRampXYZ::begin(): LinearRampXYZ::duration = "+String(LinearRampXYZ::duration)+".");

		LinearRampXYZ::speeds.gripX = LinearRampXYZ::distances.gripX/LinearRampXYZ::duration;
		LinearRampXYZ::speeds.gripY = LinearRampXYZ::distances.gripY/LinearRampXYZ::duration;
		LinearRampXYZ::speeds.gripZ = LinearRampXYZ::distances.gripZ/LinearRampXYZ::duration;

    LinearRampXYZ::speeds.gripSpinAngle = LinearRampXYZ::distances.gripSpinAngle / LinearRampXYZ::duration;
    LinearRampXYZ::speeds.gripTiltAngle = LinearRampXYZ::distances.gripTiltAngle / LinearRampXYZ::duration;
    LinearRampXYZ::speeds.gripOpen      = LinearRampXYZ::distances.gripOpen      / LinearRampXYZ::duration;
    
    Serial.println("LinearRampXYZ::begin(): LinearRampXYZ::speeds.grip X,Y,Z,Spin,Tilt,Open   = {"+ String(LinearRampXYZ::speeds.gripX)+", "+ String(LinearRampXYZ::speeds.gripY)+", "+ String(LinearRampXYZ::speeds.gripSpinAngle)+"}. AnglesSpeed  "+ String(LinearRampXYZ::speeds.gripSpinAngle)+", "+ String(LinearRampXYZ::speeds.gripTiltAngle)+", "+ String(LinearRampXYZ::speeds.gripOpen)+".");
		
	}
	void LinearRampXYZ::setup() {
    LinearRampXYZ::rampOnce.rampSetup(LinearRampXYZ::duration, rampTotalStepsCount);  //void RampOnce::rampSetup(double duration, int stepsCount)
	}
	
	GripPositionXYZ LinearRampXYZ::update() {
	  LinearRampXYZ::currentStep = LinearRampXYZ::rampOnce.update();                  // store updated value

    LinearRampXYZ::incrementX = (double)(LinearRampXYZ::rampOnce.rampCompletion * LinearRampXYZ::distances.gripX/100.00);
    LinearRampXYZ::incrementY = (double)(LinearRampXYZ::rampOnce.rampCompletion * LinearRampXYZ::distances.gripY/100.00);
    LinearRampXYZ::incrementZ = (double)(LinearRampXYZ::rampOnce.rampCompletion * LinearRampXYZ::distances.gripZ/100.00);
    Serial.println("LinearRampXYZ::begin(): LinearRampXYZ::increment X,Y,Z   = {"+ String(LinearRampXYZ::incrementX)+", "+ String(LinearRampXYZ::incrementY)+", "+ String(LinearRampXYZ::incrementZ)+"}.");
    
    LinearRampXYZ::currentPos.gripX = (double)(LinearRampXYZ::fromPos.gripX + LinearRampXYZ::incrementX);
    LinearRampXYZ::currentPos.gripY = (double)(LinearRampXYZ::fromPos.gripY + LinearRampXYZ::incrementY);
    LinearRampXYZ::currentPos.gripZ = (double)(LinearRampXYZ::fromPos.gripZ + LinearRampXYZ::incrementZ);
    //LinearRampXYZ::currentStep/ rampTotalStepsCount
    return currentPos;
	}
