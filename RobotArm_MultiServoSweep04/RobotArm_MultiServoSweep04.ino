/*        
       DIY Arduino Robot Arm 
*/

#include <Arduino.h>                    // needed for PlatformIO
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Ramp.h>


#define SPEED         60                // degree/s


Servo servo01;
Servo servo02;
Servo servo03;
Servo servo04;
Servo servo05;
Servo servo06;

//SoftwareSerial Bluetooth(3, 4); // Arduino(RX, TX) - HC-05 Bluetooth (TX, RX)

int servo1Pos, servo2Pos, servo3Pos, servo4Pos, servo5Pos, servo6Pos; // current position
//int servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos; // previous position
//int servo01SP[50], servo02SP[50], servo03SP[50], servo04SP[50], servo05SP[50], servo06SP[50]; // for storing positions/steps
int speedDelay = 50;
int index = 0;
int runRobotArm = 0;
int initializationDone = 0;
boolean previousSensorVal = LOW;
boolean previousButtonsVals[7] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW};

int servo1PPos = 90;
int servo2PPos = 100;
int servo3PPos = 35;
int servo4PPos = 140;
int servo5PPos = 85;
int servo6PPos = 80;

//----------------------mainRoboArmTurn---------------------------
void mainRoboArmTurn() {
    int stepDelay= 500;
      servoSweep(servo01, servo1PPos, 180, 0, "S01 to 180. zakladna dolava"); //zakladna dolava
      delay(stepDelay);
      
      servoSweep(servo02, servo2PPos, 130, 0, "S02 to 130. hnede rameno kolmo hore (z 90 na 130)"); //hnede rameno kolmo hore (z 90 na 130)
      delay(stepDelay);
      
      servoSweep(servo03, servo3PPos, 90, 0, "S03 to 90. biele rameno rovnobezne s hnedym (z 35 na 90)"); //biele rameno rovnobezne s hnedym (z 35 na 90)
      delay(stepDelay);
      
      servoSweep(servo02, 130, 180, 0, "S02 to 130. hnede rameno dopredu (z 130 na 180)");
      delay(stepDelay);
      
      servoSweep(servo03, 90, 60, 0, "S03 to 90. biele rameno rovnobezne s hnedym (z 35 na 90)"); //biele rameno rovnobezne s hnedym (z 35 na 90)
      delay(stepDelay);
      
      servoSweep(servo01, 180, 90, 0, "S01 from 180 to 90. zakladna doprava"); //zakladna doprava 
      delay(stepDelay);
      servoSweep(servo01, 90, 0, 0, "S01 from 90 to 0. zakladna doprava");   //zakladna doprava
      delay(stepDelay);

      servoSweep(servo02, 180, servo2PPos, 0, "S02 from 130 to 100 hnede rameno dopredu"); //hnede rameno dopredu
      delay(stepDelay);

      servoSweep(servo03, 90, servo3PPos, 0, "S03 from 90 to 35. biele rameno hore (z 90 na 35)"); //biele rameno hore (z 90 na 35)
      delay(stepDelay);
      
      servoSweep(servo01, 0, servo1PPos, 0, "S01 from 0 to servo1PPos zakladna dolava (z 0 na 90)"); //zakladna dolava
      delay(stepDelay);
}
//----------------------end of mainRoboArmTurn---------------------------

//----------------------setup-------------------------------------------
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(19200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("RobotArm_servoSweep  Started--------------------------------------------------------");
  Serial.println();
  //configure pin 2 as an input and enable the internal pull-up resistor
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);
}
//----------------------end of setup-----------------------------------


//-----------------loop-------------------------------------------------
void loop() {
  boolean Btn2Clicked = ButtonHandle(2);
  if(Btn2Clicked == true) {
    runRobotArm = onButton2Clicked(runRobotArm) ;
  }// end of if (Btn2Clicked)
  
  if(runRobotArm == 1) {
    if (initializationDone == 0) {
      Serial.println("Servo Initialization. runRobotArm == 2");
      ServoInitialization();
      initializationDone = 1;
      delay(1000);
    }
  }
  if(runRobotArm == 2) {
      Serial.println("Servo movement Start. runRobotArm == 3 ");
      mainRoboArmTurn();
      Serial.println("Servo End.");
      runRobotArm = runRobotArm - 1;  // = 2
  }
}// ----------end of loop-------------------------------------------

//--------------------updateServo()--------------------------------
//void updateServo() {
//  int val = myAngle.update();           // update ramp value
//  Serial.println(myAngle.getValue());   // prompt angle value
//  myservo.write(myAngle.getValue());    // transmit it to the servo
//}
 //--------------------end of updateServo-----------------------------


//---------------------- onButton2Click---------------------
int onButton2Clicked(int lastAction) {
  int newAction = lastAction;
  //if(lastAction==0) {
  //  newAction = 1;
  //  Serial.println(" Start: runRobotArm => 1");
  //} else {
      switch (lastAction) {
        case 0:
          newAction = 1;
          Serial.println(" case 0: newAction => 1");
          break;
          
        case 1:
          newAction = 2;
          Serial.println(" case 1: newAction => 2");
          break;
          
        case 2:
          newAction = 3;
          Serial.println(" case 2: newAction => 3");
          break;
          
        case 3:
          newAction = 3;
          Serial.println(" case 3: newAction => 3");
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
boolean ButtonHandle(int BtnPin) {
//read the pushbutton value into a variable
  boolean BtnClicked = false;
  int ButtonPressed = !digitalRead(BtnPin);
  if (ButtonPressed == LOW) {
    digitalWrite(13, LOW);
    if (previousButtonsVals[BtnPin] == HIGH) {
      Serial.print(" Button ");
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


void servoSweep(Servo myservo, int positionFrom, int positionTo, int moveDelay, String text ) {
  Serial.print("Servo sweep from  \'");
  Serial.print(positionFrom);
  Serial.print("\' , to \'");
  Serial.print(positionTo);
  Serial.print("\'. ");
  Serial.println(text);
  if (moveDelay <=0) {
    moveDelay = 15;
    Serial.print("moveDelay was 0, changed to \'");
    Serial.print(moveDelay);
    Serial.println("\'. ");
    }
  int pos = positionFrom;    // variable to store the servo position
  int step = 1;
  //int direction = 0;
  if(positionFrom < positionTo)
  {
    step = 1;
  } 
  else 
  {
    step = -1;
  }
  Serial.print("Direction:  step =  \'");
  Serial.print(step);
  Serial.println("\'.");
  if(step == 1) {
    for (pos = positionFrom; pos <= positionTo; pos = pos + step) {
      //Serial.print("For 1, pos = ");
      //Serial.print(pos);
      //Serial.println(".");
      if (runRobotArm == 0) {
        Serial.println("Break 1");
        break;
      }
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(moveDelay);                       // waits 15ms for the servo to reach the position
    }
  }
  else
  {
    for (pos = positionFrom; pos >= positionTo; pos = pos + step) {
      //Serial.print("For 2, pos = ");
      //Serial.print(pos);
      //Serial.println(".");
      if (runRobotArm == 0) {
        Serial.println("Break 2");
        break;
      }
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(moveDelay);                       // waits 15ms for the servo to reach the position
    }
  }
  Serial.println("servoSweep: Done1.");  
}

void ServoInitialization() {
  servo01.attach(5);
  servo02.attach(6);
  servo03.attach(7);
  servo04.attach(8);
  servo05.attach(9);
  servo06.attach(10);
  delay(20);
  // Robot arm initial position
  servo01.write(servo1PPos);
  servo02.write(servo2PPos);
  servo03.write(servo3PPos);
  servo04.write(servo4PPos);
  servo05.write(servo5PPos);
  servo06.write(servo6PPos);  
}
