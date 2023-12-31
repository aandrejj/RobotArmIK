#include "Arduino.h"
#include "RoboArmTurn.h"
#include "ArmServos.h"

	RoboArmTurn::RoboArmTurn() {
		
	}
	//-----------------takeNextRoboArmPosition------------------------------------
	GripPositionXYZ RoboArmTurn::takeNextRoboArmPosition() {
		Serial.println("takeNextRoboArmPosition: Started");
		Serial.println("takeNextRoboArmPosition: movesScriptIndex = "+ String(movesScriptIndex)+".");
		//Serial.println(movesScriptIndex);
		
		GripPositionXYZ newGripPosition;
		
		//actualTargetPositionXYZ = movesScript[movesScriptIndex];
		
		newGripPosition.gripX = movesScript[movesScriptIndex][0];
		newGripPosition.gripY = movesScript[movesScriptIndex][1];
		newGripPosition.gripZ = movesScript[movesScriptIndex][2];
		newGripPosition.gripSpinAngle = movesScript[movesScriptIndex][3];
		newGripPosition.gripTiltAngle = movesScript[movesScriptIndex][4];
		newGripPosition.gripOpen =  movesScript[movesScriptIndex][5];
		newGripPosition.movesScriptEnd = false;
		
		movesScriptIndex++;
		if( movesScriptIndex >= movesScriptSize) {
			movesScriptEnd = true;
			
			newGripPosition.gripX = 0;
			newGripPosition.gripY = 0;
			newGripPosition.gripZ = 0;
			newGripPosition.gripSpinAngle = 0;
			newGripPosition.gripTiltAngle = 0;
			newGripPosition.gripOpen =  0;
			newGripPosition.movesScriptEnd = false;
			
			//return newGripPosition;
		}
		Serial.println("takeNextRoboArmPosition: newGripPosition.gripX          = "+ String(newGripPosition.gripX)+",");
		Serial.println("takeNextRoboArmPosition: newGripPosition.gripY          = "+ String(newGripPosition.gripY)+",");
		Serial.println("takeNextRoboArmPosition: newGripPosition.gripZ          = "+ String(newGripPosition.gripZ)+",");
		Serial.println("takeNextRoboArmPosition: newGripPosition.gripSpinAngle  = "+ String(newGripPosition.gripSpinAngle)+",");
		Serial.println("takeNextRoboArmPosition: newGripPosition.gripTiltAngle  = "+ String(newGripPosition.gripTiltAngle)+",");
		Serial.println("takeNextRoboArmPosition: newGripPosition.gripOpen       = "+ String(newGripPosition.gripOpen)+",");
		Serial.println("takeNextRoboArmPosition: newGripPosition.movesScriptEnd = "+ String(newGripPosition.movesScriptEnd)+".");
		Serial.println("takeNextRoboArmPosition: End");
		return newGripPosition;
	}//---------------------end of takeNextRoboArmPosition-----------------------
	

