/*        
       DIY Arduino Robot Arm Smartphone Control  
        by Dejan, www.HowToMechatronics.com  
*/

#include <SoftwareSerial.h>
#include <Servo.h>

Servo servo01;
Servo servo02;
Servo servo03;
Servo servo04;
Servo servo05;
Servo servo06;

//SoftwareSerial Bluetooth(3, 4); // Arduino(RX, TX) - HC-05 Bluetooth (TX, RX)

int servo1Pos, servo2Pos, servo3Pos, servo4Pos, servo5Pos, servo6Pos; // current position
//int servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos; // previous position
int servo01SP[50], servo02SP[50], servo03SP[50], servo04SP[50], servo05SP[50], servo06SP[50]; // for storing positions/steps
int speedDelay = 50;
int index = 0;
int runRobotArm = 0;
int initializationDone = 0;
boolean previousSensorVal = LOW;

int servo1PPos = 90;
int servo2PPos = 100;
int servo3PPos = 35;
int servo4PPos = 140;
int servo5PPos = 85;
int servo6PPos = 80;


//String dataIn = "";


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("RobotArm_servoSweep  Started--------------------------------------------------------");
  Serial.println();
  //configure pin 2 as an input and enable the internal pull-up resistor
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);
}

void loop() {
  //read the pushbutton value into a variable
  
  int sensorVal = !digitalRead(2);
  if (sensorVal == LOW) {
    digitalWrite(13, LOW);
    if (previousSensorVal == HIGH) {
      Serial.println(" Button released");
    }

  } else {
    digitalWrite(13, HIGH);
    
    if (previousSensorVal == LOW) {
      Serial.println(" Button pressed");
      switch (runRobotArm) {
        case 0:
          runRobotArm = 1;
          Serial.println(" case 0: runRobotArm => 1");
          break;
          
        case 1:
          runRobotArm = 2;
          Serial.println(" case 1: runRobotArm => 2");
          break;
          
        case 2:
          runRobotArm = 2;
          Serial.println(" case 2: runRobotArm => 2");
          break;
          
        default:
          //nic
          break;
          
      }
    }
  }
  
  previousSensorVal = sensorVal;
  
//    if (runRobotArm == 0) {
//      runRobotArm = 1;
//    } else {
//      runRobotArm = 0;
//    }

  

  if(runRobotArm == 1) {
    if (initializationDone == 0) {
      Serial.println("Servo Initialization. runRobotArm == 1");
      ServoInitialization();
      initializationDone = 1;
      delay(1000);
    }
  }
  if(runRobotArm == 2) {
      Serial.println("Servo movement Start. runRobotArm == 2 ");
      mainRoboArmTurn();
      Serial.println("Servo End.");
      runRobotArm = 1;
  }
}

void mainRoboArmTurn() {
    int stepDelay= 500;
      servoSweep(servo01, servo1PPos, 180); //zakladna dolava
      delay(stepDelay);
      
      servoSweep(servo02, servo2PPos, 130); //hnede rameno dozadu
      delay(stepDelay);
      
      servoSweep(servo03, servo3PPos, 90); //biele rameno dolu (z 35 na 90)
      delay(stepDelay);
      
      servoSweep(servo01, 180, 90); //zakladna doprava 
      delay(stepDelay);
      servoSweep(servo01, 90, 0);   //zakladna doprava
      delay(stepDelay);

      servoSweep(servo02, servo2PPos, 130); //hnede rameno dopredu
      delay(stepDelay);

      servoSweep(servo03, 90, servo3PPos); //biele rameno hore (z 90 na 35)
      delay(stepDelay);
      
      servoSweep(servo01, 0, servo1PPos); //zakladna dolava
      delay(stepDelay);
  
}


void servoSweep(Servo myservo, int positionFrom, int positionTo) {
  Serial.print("Servo sweep from  \'");
  Serial.print(positionFrom);
  Serial.print("\' , to \'");
  Serial.print(positionTo);
  Serial.println("\'.");
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
      Serial.print("For 1, pos = ");
      Serial.print(pos);
      Serial.println(".");
      if (runRobotArm == 0) {
        Serial.println("Break 1");
        break;
      }
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(speedDelay);                       // waits 15ms for the servo to reach the position
    }
  }
  else
  {
    for (pos = positionFrom; pos >= positionTo; pos = pos + step) {
      Serial.print("For 2, pos = ");
      Serial.print(pos);
      Serial.println(".");
      if (runRobotArm == 0) {
        Serial.println("Break 2");
        break;
      }
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(speedDelay);                       // waits 15ms for the servo to reach the position
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
