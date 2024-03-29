#include "Arduino.h"
#include "SoftwareSerial.h"
#include "BluetoothFactory.h"
#include "RemoteController_dataStructures.h"


  //unsigned long interval;
  //unsigned long previousServoMillis;
  //unsigned long servoInterval;
  //unsigned long previousSafetyMillis;

int mode;
int count;

  //unsigned long milisOfLastStateChanged;
  //unsigned long maxTimeOfNoChangeMillis = 1500;
  //unsigned long  currentStateDuration;
  //bool LedIsBlinking = true;
  //bool BtLedIsSteadyOn = false;

  //int state; // BT state
  //int previous_state;

bool bt_State = false;
//bool previous_bt_state = false;



//create object
EasyTransfer ET1;   // send serial
EasyTransfer ET2;   // rec serial

SEND_DATA_STRUCTURE mydata_send;
RECEIVE_DATA_STRUCTURE mydata_remote;

SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);

	BluetoothFactory::BluetoothFactory() {
  }
  
  void BluetoothFactory::begin() {
    //previous_state = 0;
    //previousBtMillis = 0;
    //interval = 20;
    //previousServoMillis=0;
    //servoInterval = 200;
	}

  void BluetoothFactory::BT_to_serial_prepare(){
    Serial.println("Bluetooth initialization....");

    // Setup BT module
    pinMode(BLUETOOTH_TX, INPUT);
    pinMode(BLUETOOTH_RX, OUTPUT);  
    pinMode(STATE, INPUT);
    pinMode(ENABLE, OUTPUT);
    #ifdef EN_PIN_HIGH  
      digitalWrite(ENABLE, HIGH);   // Used to force AT-mode for HC-05. More flexible is to press the button on the pcb
    #endif
    
    bluetooth.begin(BTBaud);
    ET1.begin(details(mydata_send), &bluetooth);
    ET2.begin(details(mydata_remote), &bluetooth);
    //bluetooth_initialized = true;
    Serial.println("Bluetooth available.");
    //previous_Bluetooth_State = bluetooth_On;
  }

  BluetoothOutputData BluetoothFactory::BT_loop(unsigned long currentMillis) {

      BluetoothOutputData bluetoothOutputData;
      bluetoothOutputData.dataReceived = false;

      if(ET2.receiveData()){                                        // main data receive

        mydata_send.mode  = mode;
        mydata_send.count = count;

        ET1.sendData();                                           // send data back to remote       
        /*
        Serial.println( "LX:"+String(mydata_remote.index_finger_knuckle_right)+
                      ", LY:"+String(mydata_remote.pinky_knuckle_right)+
                      ", RX:"+String(mydata_remote.index_finger_fingertip)+
                      ", RY:"+String(mydata_remote.index_finger_knuckle_left)+
                      ", count:"+String(count));
        */

        bluetoothOutputData.dataReceived = true;
        bluetoothOutputData.stick1_X = mydata_remote.stick1_X;
        bluetoothOutputData.stick1_Y = mydata_remote.stick1_Y;
        bluetoothOutputData.stick2_X = mydata_remote.stick2_X;
        bluetoothOutputData.stick2_Y = mydata_remote.stick2_Y;
        bluetoothOutputData.Select  = mydata_remote.Select;
        bluetoothOutputData.menuUp = mydata_remote.menuUp;

        bluetoothOutputData.navKeyUp = mydata_remote.navKeyUp;
        bluetoothOutputData.navKeyDown = mydata_remote.navKeyDown;
        bluetoothOutputData.navKeyLeft = mydata_remote.navKeyLeft;
        bluetoothOutputData.navKeyRight = mydata_remote.navKeyRight;
        bluetoothOutputData.navKeyMiddle = mydata_remote.navKeyMiddle;
        bluetoothOutputData.navKeySet = mydata_remote.navKeySet;
        bluetoothOutputData.navKeyReset = mydata_remote.navKeyReset;
        
        // end of receive data
      } 
      //else if(currentMillis - previousSafetyMillis > 200) {         // safeties
        //Serial.Println("No Data")
      //}

      count = count + 1;                                              // update count for remote monitoring
    return bluetoothOutputData;
}
