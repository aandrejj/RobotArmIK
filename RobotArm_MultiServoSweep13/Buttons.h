/**
 * Buttons.h
 */
#ifndef Buttons_h
#define Buttons_h
#include "Arduino.h"

class Buttons {
  public:
    Buttons();

  int onButton2Clicked(int lastAction);
  
  boolean ButtonHandle(int BtnPin);
  
private:

};

#endif
