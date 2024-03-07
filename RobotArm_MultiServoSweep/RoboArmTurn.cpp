#include "Arduino.h"
#include "RoboArmTurn.h"

	RoboArmTurn::RoboArmTurn() {
		
	}
	//-----------------takeNextRoboArmPosition------------------------------------
	GripPositionXYZ RoboArmTurn::takeNextRoboArmPosition() {
    #if defined(BRIEF_LOG)  
      Serial.println("--------------------------------------------------");
		#elif DEBUG 
      Serial.println("--------------------------------------------------");
		  Serial.println("takeNextRoboArmPosition: Started");
		  Serial.println("takeNextRoboArmPosition: movesScriptIndex = "+ String(movesScriptIndex)+".");
    #endif
		//Serial.println(movesScriptIndex);
		
		GripPositionXYZ newGripPosition;
		
		//actualTargetPositionXYZ = movesScript[movesScriptIndex];
		
		newGripPosition.gripX = movesScript[movesScriptIndex][0];
		newGripPosition.gripY = movesScript[movesScriptIndex][1];
		newGripPosition.gripZ = movesScript[movesScriptIndex][2];
		newGripPosition.gripSpinAngle = movesScript[movesScriptIndex][3];
		newGripPosition.gripTiltAngle = movesScript[movesScriptIndex][4];
		newGripPosition.gripWidth =  movesScript[movesScriptIndex][5];
    newGripPosition.duration =  movesScript[movesScriptIndex][6];
		newGripPosition.movesScriptEnd = false;
		
		movesScriptIndex++;
		if( movesScriptIndex > movesScriptSize) {
			movesScriptEnd = true;
			
			newGripPosition.gripX = 0;
			newGripPosition.gripY = 0;
			newGripPosition.gripZ = 0;
			newGripPosition.gripSpinAngle = 0;
			newGripPosition.gripTiltAngle = 0;
			newGripPosition.gripWidth =  0;
     newGripPosition.duration =  0;
			newGripPosition.movesScriptEnd = true;
			
			//return newGripPosition;
		}
   if(newGripPosition.gripWidth > 60) {
    Serial.println("takeNextRoboArmPosition:  WARNING: gripWidth = "+String(newGripPosition.gripWidth)+". Is bigger than maxGripWidth = 60. Truncating gripWith to 60...");
   }
    #if defined(DEBUG) || defined(BRIEF_LOG) 
	    Serial.println("takeNextRoboArmPosition: newGripPosition = {"+ String(newGripPosition.gripX)+", "+ String(newGripPosition.gripY)+", "+ String(newGripPosition.gripZ)+", "+ String(newGripPosition.gripSpinAngle)+", "+ String(newGripPosition.gripTiltAngle)+", "+ String(newGripPosition.gripWidth)+"}, duration = "+ String(newGripPosition.duration)+", end = "+ String(newGripPosition.movesScriptEnd)+".");
    #endif
    /*
    Serial.println("takeNextRoboArmPosition: newGripPosition.gripX          = "+ String(newGripPosition.gripX)+",");
		Serial.println("takeNextRoboArmPosition: newGripPosition.gripY          = "+ String(newGripPosition.gripY)+",");
		Serial.println("takeNextRoboArmPosition: newGripPosition.gripZ          = "+ String(newGripPosition.gripZ)+",");
		Serial.println("takeNextRoboArmPosition: newGripPosition.gripSpinAngle  = "+ String(newGripPosition.gripSpinAngle)+",");
		Serial.println("takeNextRoboArmPosition: newGripPosition.gripTiltAngle  = "+ String(newGripPosition.gripTiltAngle)+",");
		Serial.println("takeNextRoboArmPosition: newGripPosition.gripWidth       = "+ String(newGripPosition.gripWidth)+",");
		Serial.println("takeNextRoboArmPosition: newGripPosition.movesScriptEnd = "+ String(newGripPosition.movesScriptEnd)+".");
    */
		#ifdef DEBUG 
		  Serial.println("takeNextRoboArmPosition: End");
        #endif
		return newGripPosition;
	}//---------------------end of takeNextRoboArmPosition-----------------------

    //-----------------resetMovescript------------------------------------
  GripPositionXYZ RoboArmTurn::resetMovescript() {
    #if defined(BRIEF_LOG)  
      Serial.println("--------------------------------------------------");
    #elif DEBUG 
      Serial.println("--------------------------------------------------");
      Serial.println("resetMovescript: Started");
    #endif
    
    movesScriptIndex = 0;
    
    #if defined(DEBUG) || defined(BRIEF_LOG)  
      Serial.println("resetMovescript: movesScriptIndex = "+ String(movesScriptIndex)+".");
    #endif
    return RoboArmTurn::takeNextRoboArmPosition();
  }
