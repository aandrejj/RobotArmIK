/**
 * Buttons.h
 */
#ifndef Buttons_h
#define Buttons_h
#include "Arduino.h"

class Buttons {
  public:
    Buttons();
    boolean previousButtonsVals[3] = {LOW, LOW, LOW};
    int onButton2Clicked(int lastAction);
  
  boolean ButtonHandle(int BtnPin);
  
private:

};

#endif
