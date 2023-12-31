/*        
       DIY Arduino Robot Arm 
*/

//
//
// NEXT is https://github.com/siteswapjuggler/RAMP
//         https://github.com/siteswapjuggler/RAMP/blob/master/examples/ramp_servo/ramp_servo.ino
//        
//         https://github.com/roTechnic/InverseKinematicArm
//         https://github.com/roTechnic/InverseKinematicArm/blob/main/inverse_kinematics.ino
//
//

#include <Arduino.h>                    // needed for PlatformIO
#include <SoftwareSerial.h>
//#include <Encoder.h>
#include <Servo.h>
//#include <Ramp.h>
#include <ArduinoJson.h>  //https://arduinojson.org/   https://arduinojson.org/v6/api/jsonarray/

#define SPEED         60                // degree/s


Servo servo01; //zakladna
Servo servo02; //spodne hnede rameno
Servo servo03; //horne  biela rameno
Servo servo04; //ruka nabok  100 = zhruba vodorovne
Servo servo05; //ruka hore
Servo servo06; //ruka otvorena= 100, zatvorena = 60

int servoCurrentPos[7];
//int servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos; // previous position
//int servo01SP[50], servo02SP[50], servo03SP[50], servo04SP[50], servo05SP[50], servo06SP[50]; // for storing positions/steps
int speedDelay = 50;
int index = 0;
int runRobotArm = 0;
int initializationDone = 0;
boolean previousSensorVal = LOW;
boolean previousButtonsVals[3] = {LOW, LOW, LOW};

int servo1PPos = 90; //zakladna
int servo2PPos = 100;//spodne hnede rameno
int servo3PPos = 35; //horne  biela rameno
int servo4PPos = 100;//ruka nabok  100 = zhruba vodorovne
int servo5PPos = 85; //ruka hore
int servo6PPos = 80; //ruka otvorena= 100, zatvorena = 60
//----------------------setup-------------------------------------------
void setup() {
  // Open serial communications and wait for port to open:
  //Serial.begin(19200);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("RobotArm_servoSweep  Started--------------------------------------------------------");
  Serial.println();
  //configure pin 2 as an input and enable the internal pull-up resistor
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);


  //char moves[] = "[{\"servoNumber\": 1, \"positionFrom\": 0, \"positionTo\": 180},{\"servoNumber\": 2, \"positionFrom\": 0, \"positionTo\": 180}]";
  //servoSweepJson(moves,"Method2 text.");
  Serial.println("setup: Setup DONE. System is ready to Initialize servos. Press Button to start...");

}
//----------------------end of setup-----------------------------------


//---------------servoSweepJson----------------------------------------
void servoSweepJson(char moves[], String text ) {
  StaticJsonDocument<512> moves_doc;
  //DynamicJsonDocument doc(220);
 // JSON input string.
  //
  // Using a char[], as shown here, enables the "zero-copy" mode. This mode uses
  // the minimal amount of memory because the JsonDocument stores pointers to
  // the input buffer.
  // If you use another type of input, ArduinoJson must copy the strings from
  // the input to the JsonDocument, so you need to increase the capacity of the
  // JsonDocument.
  
  Serial.print("servoSweepJson: 2 Started with params : ");
  Serial.print("moves = \'");
  Serial.print(moves);  
  Serial.print("\', text =");
  Serial.println(text);
  
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(moves_doc, moves);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("servoSweepJson: deserializeJson() failed: "));
    Serial.print("moves= \'");
    Serial.print(moves);
    Serial.print("\'..., text =");
    Serial.println(text);
  
    Serial.println(error.f_str());
    return;
  }

  
  int servoNumber;
  int positionFrom;
  int positionTo;
  int moveDelay;
  

  JsonArray array = moves_doc.as<JsonArray>();
  //for (int i = 0; i <= arrLen; i++) {
  //for(JsonVariant v : array) {
  int i = 0; 
  for(JsonObject object : array) {
    servoNumber =  object["servoNumber"].as<int>();
    positionFrom = object["positionFrom"].as<int>();
    positionTo =   object["positionTo"].as<int>();
    //moveDelay =    object["moveDelay"].as<int>();

    Serial.print("i = ");
    Serial.print(i);
    // Print values.
    Serial.print(", servoNumber = ");
    Serial.print(servoNumber);
    
    Serial.print(", positionFrom = ");
    Serial.print(positionFrom);
    
    Serial.print(", positionTo = ");
    Serial.print(positionTo);
    
    //Serial.print(", moveDelay = ");
    //Serial.print(moveDelay);
    
    Serial.print(". ");
    Serial.println(text);
    
    servoSweep(servoNumber, positionFrom, positionTo, 0, "");
    i++;
  }
  Serial.println("servoSweepJson END.OK.");
  //StaticJsonDocument<200> doc;
  //DynamicJsonDocument doc(200);

}
//---------------end of servoSweepJson----------------------------------------

//----------------------mainRoboArmTurn---------------------------
void mainRoboArmTurn() {
  //char moves1[] = "{\"servo\":1,\"posFrom\":0,\"posTo\":180,\"Delay\":0,\"viem\":\"Nic\"}";
  //servoSweepJson(moves1,"Method2 text.");
    
    int stepDelay= 500;
      servoSweep(4, 0, 160, 0, ""); //otocit ruku nabok
      delay(stepDelay);
      servoSweep(4, 0, 0, 0, ""); //
      delay(stepDelay);      
      servoSweep(4, 0, 100, 0, ""); //
      delay(stepDelay);
      
      
      servoSweep(6, 0, 55, 0, ""); //zatvorit ruku (chytit)
      delay(stepDelay);
      servoSweep(6, 0, 110, 0, ""); //otvorit ruku (pustit)
      delay(stepDelay);      
      servoSweep(6, 0, 80, 0, ""); //pootvorit ruku ( stred)
      delay(stepDelay);
      
      servoSweep(5, 0, 105, 0, ""); //zdvihnut ruku hore?
      delay(stepDelay);
      servoSweep(5, 0, 65, 0, ""); // spustit ruhu dole?
      delay(stepDelay);      
      servoSweep(5, 0, 85, 0, ""); // vyska ruky 'v strede'??
      delay(stepDelay);
      

      // void servoSweep(int servoNumber,  int positionFrom, int positionTo, int moveDelay, String text )
      servoSweep(1, 0, 180, 0, "S01 to 180. zakladna dolava"); //zakladna dolava
      delay(stepDelay);
      
      
      //servoSweep(2, 0, 130, 0, "S02 to 130. hnede rameno kolmo hore (z 90 na 130)"); //hnede rameno kolmo hore (z 90 na 130)
      //delay(stepDelay);
      
      
      //servoSweep(3, 0, 90, 0, "S03 to 90. biele rameno rovnobezne s hnedym (z 35 na 90)"); //biele rameno rovnobezne s hnedym (z 35 na 90)
      //delay(stepDelay);
      
      servoSweep(2, 0, 180, 0, "S02 to 130. hnede rameno dopredu (z 130 na 180)");
      delay(stepDelay);
      
      
      //servoSweep(3, 0, 60, 0, "S03 to 60. biele rameno rovnobezne s hnedym (z 35 na 90)"); //biele rameno rovnobezne s hnedym (z 35 na 90)
      //delay(stepDelay);
      
      
      servoSweep(3, 0, 35, 0, "S03 to 35. biele rameno klesa pod hnede (z 35 na 90)"); //biele rameno rovnobezne s hnedym (z 35 na 90)
      delay(stepDelay);
      
      servoSweep(3, 0, 60, 0, "S03 to 60. biele rameno rovnobezne s hnedym (z 35 na 90)"); //biele rameno rovnobezne s hnedym (z 35 na 90)
      delay(stepDelay);
      
      servoSweep(1, 0, 90, 0, "S01 from 180 to 90. zakladna doprava"); //zakladna doprava 
      delay(stepDelay);
      servoSweep(3, 0, 35, 0, "S03 to 35. biele rameno rovnobezne s hnedym (z 35 na 90)"); //biele rameno rovnobezne s hnedym (z 35 na 90)
      delay(stepDelay);
      
      servoSweep(3, 0, 60, 0, "S03 to 60. biele rameno rovnobezne s hnedym (z 35 na 90)"); //biele rameno rovnobezne s hnedym (z 35 na 90)
      delay(stepDelay);
      
      servoSweep(1, 0, 0, 0, "S01 from 90 to 0. zakladna doprava");   //zakladna doprava
      delay(stepDelay);

      servoSweep(3, 0, 35, 0, "S03 to 35. biele rameno rovnobezne s hnedym (z 35 na 90)"); //biele rameno rovnobezne s hnedym (z 35 na 90)
      delay(stepDelay);
      
      servoSweep(3, 0, 60, 0, "S03 to 60. biele rameno rovnobezne s hnedym (z 35 na 90)"); //biele rameno rovnobezne s hnedym (z 35 na 90)
      delay(stepDelay);
      
      servoSweep(2, 0, servo2PPos, 0, "S02 from 130 to 100 hnede rameno dopredu"); //hnede rameno dopredu
      delay(stepDelay);

      servoSweep(3, 0, servo3PPos, 0, "S03 from 90 to 35. biele rameno hore (z 90 na 35)"); //biele rameno hore (z 90 na 35)
      delay(stepDelay);
      
      servoSweep(1, 0, servo1PPos, 0, "S01 from 0 to servo1PPos zakladna dolava (z 0 na 90)"); //zakladna dolava
      delay(stepDelay);
     
}
//----------------------end of mainRoboArmTurn---------------------------


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
      delay(100);
      Serial.println("loop: Servo Initialization  DONE. Press Button to start...-----------------------------------------");
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

//---------------servoSweep----------------------------------------
void servoSweep(int servoNumber,  int positionFrom, int positionTo, int moveDelay, String text ) {
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
  
  if(positionFrom == 0) {
    positionFrom = servoCurrentPos[servoNumber];
    Serial.print("positionFrom was 0, changed to \'");
    Serial.print(servoCurrentPos[servoNumber]);
    Serial.println("\'. ");
  }

  Servo servo;
  
switch(servoNumber) {
  case 1: {servo = servo01; break;}
  case 2: {servo = servo02; break;}
  case 3: {servo = servo03; break;}
  case 4: {servo = servo04; break;}
  case 5: {servo = servo05; break;}
  case 6: {servo = servo06; break;}
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
      if (runRobotArm == 0) {
        Serial.println("Break 1");
        break;
      }
      // in steps of 1 degree
      servo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(moveDelay);                       // waits 15ms for the servo to reach the position
    }
  }
  else
  {
    for (pos = positionFrom; pos >= positionTo; pos = pos + step) {
      if (runRobotArm == 0) {
        Serial.println("Break 2");
        break;
      }
      // in steps of 1 degree
      servo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(moveDelay);                       // waits 15ms for the servo to reach the position
    }
  }
  servoCurrentPos[servoNumber] = positionTo;
  switch(servoNumber) {
    case 1: servo01 = servo; break;
    case 2: servo02 = servo; break;
    case 3: servo03 = servo; break;
    case 4: servo04 = servo; break;
    case 5: servo05 = servo; break;
    case 6: servo06 = servo; break;
    default:
      Serial.print("servoSweep: servoNumber = ");
        Serial.print(servoNumber);
        Serial.println(". return");
        return;
    break;
  Serial.println("servoSweep: Done.");  
  }
}

//---------------SevoInitialization----------------------------------
void ServoInitialization() {
  Serial.println("Initialization started");
  delay(200);
  servo01.attach(5);//servo01.attach( 5,460 ,2400);
  servo02.attach(6);
  servo03.attach(7);
  servo04.attach(8);
  servo05.attach(9);
  servo06.attach(10); //,460 ,2400
  Serial.println("Servos attached");
  delay(20);
  Serial.println("Robot arm initial position");
  servo01.write(servo1PPos);
  //Serial.println("Servo01 positioned");
  //delay(100);

  servo02.write(servo2PPos);
  //Serial.println("Servo02 positioned");
  //delay(100);
  
  servo03.write(servo3PPos);
  //Serial.println("Servo03 positioned");
  //delay(100);
  
  servo04.write(servo4PPos);
  //Serial.println("Servo04 positioned");
  //delay(100);
  
  servo05.write(servo5PPos);
  //Serial.println("Servo05 positioned");
  //delay(100);
  
  servo06.write(servo6PPos);  
  //Serial.println("Servo06 positioned");
  //delay(100);
  
  Serial.println("Servos positioned to default positions");

  servoCurrentPos[1]=servo1PPos;
  servoCurrentPos[2]=servo1PPos;
  servoCurrentPos[3]=servo1PPos;
  servoCurrentPos[4]=servo1PPos;
  servoCurrentPos[5]=servo1PPos;
  servoCurrentPos[6]=servo1PPos;
  Serial.println("Initialization OK.");
  
}
