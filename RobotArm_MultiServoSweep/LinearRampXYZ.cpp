#include "Arduino.h"
#include "LinearRampXYZ.h"
 //https://github.com/siteswapjuggler/RAMP/tree/master

static volatile unsigned int rampTotalStepsCount = 100;              // number of steps needed to reach end-position.
 
	LinearRampXYZ::LinearRampXYZ() {
	}
	//------------------------------------begin----------------------------------------------
  void LinearRampXYZ::begin(GripPositionXYZ fromPos, GripPositionXYZ toPos) {
		LinearRampXYZ::duration = toPos.duration;
   
    #if defined(DEBUG) || defined(BRIEF_LOG) 
	  Serial.println("LinearRampXYZ::begin(): LinearRampXYZ::duration = "+String(LinearRampXYZ::duration)+".");
    #endif

    #if defined(DEBUG) || defined(BRIEF_LOG) 
      Serial.println("LinearRampXYZ::begin(): fromPos = {"+ String(fromPos.gripX)+", "+ String(fromPos.gripY)+", "+ String(fromPos.gripZ)+", "+ String(fromPos.gripSpinAngle)+", "+ String(fromPos.gripTiltAngle)+", "+ String(fromPos.gripWidth)+", end="+ String(fromPos.movesScriptEnd)+"}.");
      Serial.println("LinearRampXYZ::begin(): toPos   = {"+ String(toPos.gripX)+", "+ String(toPos.gripY)+", "+ String(toPos.gripZ)+", "+ String(toPos.gripSpinAngle)+", "+ String(toPos.gripTiltAngle)+", "+ String(toPos.gripWidth)+"}, duration="+ String(toPos.duration)+", end="+ String(toPos.movesScriptEnd)+".");
    #endif

    
    LinearRampXYZ::fromPos = fromPos;
    LinearRampXYZ::toPos = toPos;
    LinearRampXYZ::currentPos = fromPos;

    LinearRampXYZ::distances.gripX = double(toPos.gripX - fromPos.gripX);
    LinearRampXYZ::distances.gripY = double(toPos.gripY - fromPos.gripY);
    LinearRampXYZ::distances.gripZ = double(toPos.gripZ - fromPos.gripZ);

    LinearRampXYZ::distances.gripSpinAngle = double(toPos.gripSpinAngle - fromPos.gripSpinAngle);
    LinearRampXYZ::distances.gripTiltAngle = double(toPos.gripTiltAngle - fromPos.gripTiltAngle);
    LinearRampXYZ::distances.gripWidth      = double(toPos.gripWidth      - fromPos.gripWidth);

    
    #if defined(DEBUG) || defined(BRIEF_LOG) 
	    Serial.println("LinearRampXYZ::begin(): LinearRampXYZ::distances.grip X,Y,Z,Spin,Tilt,Open   = {"+ String(LinearRampXYZ::distances.gripX)+", "+ String(LinearRampXYZ::distances.gripY)+", "+ String(LinearRampXYZ::distances.gripZ)+",Angles: "+ String(LinearRampXYZ::distances.gripSpinAngle)+", "+ String(LinearRampXYZ::distances.gripTiltAngle)+", "+ String(LinearRampXYZ::distances.gripWidth)+"}.");
    #endif
		
		LinearRampXYZ::trajectoryLength = (double)sqrt( (LinearRampXYZ::distances.gripX*LinearRampXYZ::distances.gripX) + (LinearRampXYZ::distances.gripY*LinearRampXYZ::distances.gripY) + (LinearRampXYZ::distances.gripZ*LinearRampXYZ::distances.gripZ));
    if(LinearRampXYZ::trajectoryLength == 0) {
          #if defined(DEBUG) || defined(BRIEF_LOG)
            Serial.println("LinearRampXYZ::begin(): LinearRampXYZ::trajectoryLength was 0, so recomputing by GripValues ...");
          #endif
          LinearRampXYZ::trajectoryLength = (double)(sqrt( (LinearRampXYZ::distances.gripSpinAngle * LinearRampXYZ::distances.gripSpinAngle) + (LinearRampXYZ::distances.gripTiltAngle * LinearRampXYZ::distances.gripTiltAngle) + (LinearRampXYZ::distances.gripWidth * LinearRampXYZ::distances.gripWidth)));
    }
    
    #if defined(DEBUG) || defined(BRIEF_LOG) 
	  Serial.println("LinearRampXYZ::begin(): LinearRampXYZ::trajectoryLength = "+String(LinearRampXYZ::trajectoryLength)+".");
    #endif
		
		LinearRampXYZ::speeds.gripX = LinearRampXYZ::distances.gripX/LinearRampXYZ::duration;
		LinearRampXYZ::speeds.gripY = LinearRampXYZ::distances.gripY/LinearRampXYZ::duration;
		LinearRampXYZ::speeds.gripZ = LinearRampXYZ::distances.gripZ/LinearRampXYZ::duration;

    LinearRampXYZ::speeds.gripSpinAngle = LinearRampXYZ::distances.gripSpinAngle / LinearRampXYZ::duration;
    LinearRampXYZ::speeds.gripTiltAngle = LinearRampXYZ::distances.gripTiltAngle / LinearRampXYZ::duration;
    LinearRampXYZ::speeds.gripWidth     = LinearRampXYZ::distances.gripWidth      / LinearRampXYZ::duration;
    
    #if defined(DEBUG) || defined(BRIEF_LOG) 
      Serial.println("LinearRampXYZ::begin(): LinearRampXYZ::speeds.grip X,Y,Z,Spin,Tilt,Width   = {"+ String(LinearRampXYZ::speeds.gripX)+", "+ String(LinearRampXYZ::speeds.gripY)+", "+ String(LinearRampXYZ::speeds.gripSpinAngle)+"}, GripSpeed  "+ String(LinearRampXYZ::speeds.gripSpinAngle)+", "+ String(LinearRampXYZ::speeds.gripTiltAngle)+", "+ String(LinearRampXYZ::speeds.gripWidth)+".");
	  #endif
	}
  //------------------------------------end of begin----------------------------------------------
  //------------------------------------setup----------------------------------------------
	void LinearRampXYZ::setup() {
    LinearRampXYZ::rampOnce.rampSetup(LinearRampXYZ::duration, rampTotalStepsCount);  //void RampOnce::rampSetup(double duration, int stepsCount)
	}
  //------------------------------------end of setup----------------------------------------------
	
  //------------------------------------update----------------------------------------------
	GripPositionXYZ LinearRampXYZ::update() {
	  LinearRampXYZ::currentStep = LinearRampXYZ::rampOnce.update();                  // store updated value

    LinearRampXYZ::increment.gripX = (double)(LinearRampXYZ::rampOnce.rampCompletion * LinearRampXYZ::distances.gripX/100.00);  // /100 -> convert rampCompletion from % to interval [0-1]
    LinearRampXYZ::increment.gripY = (double)(LinearRampXYZ::rampOnce.rampCompletion * LinearRampXYZ::distances.gripY/100.00);
    LinearRampXYZ::increment.gripZ = (double)(LinearRampXYZ::rampOnce.rampCompletion * LinearRampXYZ::distances.gripZ/100.00);

    LinearRampXYZ::increment.gripSpinAngle = (double)(LinearRampXYZ::rampOnce.rampCompletion * LinearRampXYZ::distances.gripSpinAngle/100.00);
    LinearRampXYZ::increment.gripTiltAngle = (double)(LinearRampXYZ::rampOnce.rampCompletion * LinearRampXYZ::distances.gripTiltAngle/100.00);
    LinearRampXYZ::increment.gripWidth      = (double)(LinearRampXYZ::rampOnce.rampCompletion * LinearRampXYZ::distances.gripWidth     /100.00);
    

    #if defined(DEBUG) 
      Serial.println("LinearRampXYZ::update(): LinearRampXYZ::increment X,Y,Z,spin,tilt,Width   = {"+ String(LinearRampXYZ::increment.gripX)+", "+ String(LinearRampXYZ::increment.gripY)+", "+ String(LinearRampXYZ::increment.gripZ)+"}, Grip (Spin, Tilt, Width) = ("+String(LinearRampXYZ::increment.gripSpinAngle) +", "+String(LinearRampXYZ::increment.gripTiltAngle)+", "+String(LinearRampXYZ::increment.gripWidth)+"), ");

    #endif
    
    LinearRampXYZ::currentPos.gripX = (double)(LinearRampXYZ::fromPos.gripX + LinearRampXYZ::increment.gripX);
    LinearRampXYZ::currentPos.gripY = (double)(LinearRampXYZ::fromPos.gripY + LinearRampXYZ::increment.gripY);
    LinearRampXYZ::currentPos.gripZ = (double)(LinearRampXYZ::fromPos.gripZ + LinearRampXYZ::increment.gripZ);

    LinearRampXYZ::currentPos.gripSpinAngle = (double)(LinearRampXYZ::fromPos.gripSpinAngle + LinearRampXYZ::increment.gripSpinAngle);
    LinearRampXYZ::currentPos.gripTiltAngle = (double)(LinearRampXYZ::fromPos.gripTiltAngle + LinearRampXYZ::increment.gripTiltAngle);
    LinearRampXYZ::currentPos.gripWidth     = (double)(LinearRampXYZ::fromPos.gripWidth     + LinearRampXYZ::increment.gripWidth);
    
    //LinearRampXYZ::currentStep/ rampTotalStepsCount
    #if defined(DEBUG)
      Serial.print("LinearRampXYZ::update(): LinearRampXYZ::currentPos X,Y,Z   = {"+ String(LinearRampXYZ::currentPos.gripX)+", "+ String(LinearRampXYZ::currentPos.gripY)+", "+ String(LinearRampXYZ::currentPos.gripZ)+"}, ");
      Serial.println("Grip (Spin, Tilt, Width) = ("+String(LinearRampXYZ::currentPos.gripSpinAngle) +", "+String(LinearRampXYZ::currentPos.gripTiltAngle)+", "+String(LinearRampXYZ::currentPos.gripWidth)+"), ");

    #endif
    return currentPos;
	}
   //------------------------------------end of update----------------------------------------------
