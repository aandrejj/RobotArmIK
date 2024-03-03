#ifndef BluetoothFactory_h
#define BluetoothFactory_h

#include "Arduino.h"
#include "EasyTransfer.h"
#include "SoftwareSerial.h"
#include "RemoteController_dataStructures.h"

// Outcomment line below for HM-10, HM-19 etc
//#define HIGHSPEED   // Most modules are only 9600, although you can reconfigure this

#define SERVO_MIN_MILISEC   460                // https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
#define SERVO_MAX_MILISEC  2400                // value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
#define SERVO_MIN   135                // https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
#define SERVO_MAX   615                // value of 135 is fully counter-clockwise, 615 is fully clockwise.

#define EN_PIN_HIGH   // You can use this for HC-05 so you don't have to hold the small button on power-up to get to AT-mode

#ifdef HIGHSPEED
  #define Baud 38400   // Serial monitor
  #define BTBaud 38400 // There is only one speed for configuring HC-05, and that is 38400.
#else
  #define Baud 9600    // Serial monitor
  #define BTBaud 9600  // HM-10, HM-19 etc
#endif


#define STATE 11
#define BLUETOOTH_RX 9   // Bluetooth RX -> Arduino D9
#define BLUETOOTH_TX 10  // Bluetooth TX -> Arduino D10
//#define GND 13
//#define Vcc 12
#define ENABLE 8


typedef struct {
  int index_finger_knuckle_right;
  int pinky_knuckle_right;
  int index_finger_fingertip;
  int index_finger_knuckle_left;
  bool Select; 
  bool  dataReceived;
}BluetoothOutputData;



class BluetoothFactory {

public:
  BluetoothFactory();

  void begin();
  
  void BT_to_serial_prepare();
  BluetoothOutputData BT_loop(unsigned long currentMillis);

  //bool check_bt_from_loop(unsigned long currentMillis);

  //bool Bt_state_checker(unsigned long currentMillis, bool previousState, bool newState);

  private:

};

#endif 
