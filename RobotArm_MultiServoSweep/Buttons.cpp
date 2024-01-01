#include "Arduino.h"
#include "Buttons.h"

//-----------Buttons  ctor----------------------------
Buttons::Buttons () {
	//previousButtonsVals[3] = {LOW, LOW, LOW};
}

//---------------------- onButton2Click---------------------
int Buttons::onButton2Clicked(int lastAction) {
  int newAction = lastAction;
  //if(lastAction==0) {
  //  newAction = 1;
  //  Serial.println(" Start: runRobotArm => 1");
  //} else {
      switch (lastAction) {
        case 0:
          newAction = 1;
          Serial.println("Buttons::onButton2Clicked: case 0: newAction => 1---------------------------------------------------------------------------");
          break;
          
        case 1:
          newAction = 2;
          Serial.println("Buttons::onButton2Clicked: case 1: newAction => 2---------------------------------------------------------------------------");
          break;
          
        case 2:
          newAction = 3;
          Serial.println("Buttons::onButton2Clicked: case 2: newAction => 3---------------------------------------------------------------------------");
          break;
          
        case 3:
          newAction = 3;
          Serial.println("Buttons::onButton2Clicked: case 3: newAction => 3---------------------------------------------------------------------------");
          break;
          
        default:
          //nic
          break;
          
      } // end of switch
    //} //end of else (lastAction > 0)
  return newAction;
}
//---------------------- end of onButton2Click---------------------
// ----------------------ButtonHandle------------------------------
boolean Buttons::ButtonHandle(int BtnPin) {
//read the pushbutton value into a variable
  boolean BtnClicked = false;
  int ButtonPressed = !digitalRead(BtnPin);
  if (ButtonPressed == LOW) {
    digitalWrite(13, LOW);
    if (previousButtonsVals[BtnPin] == HIGH) {
      Serial.print("Buttons::ButtonHandle: Button ");
      Serial.print(BtnPin);
      Serial.println(" released");
      BtnClicked = true;
    }

  } else {
    digitalWrite(13, HIGH);
    //Serial.print(" Button ");
    //Serial.print(BtnPin);
    //Serial.println(" pressed");
  }
  previousButtonsVals[BtnPin] = ButtonPressed;
  
  return BtnClicked;
}
//-------------------------------------------------------
