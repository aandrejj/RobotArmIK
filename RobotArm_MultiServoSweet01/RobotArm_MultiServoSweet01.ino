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
int servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos; // previous position
int servo01SP[50], servo02SP[50], servo03SP[50], servo04SP[50], servo05SP[50], servo06SP[50]; // for storing positions/steps
int speedDelay = 20;
int index = 0;
//String dataIn = "";


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("RobotArm_servoSweep  Started");
  Serial.println();


  servo01.attach(5);
  servo02.attach(6);
  servo03.attach(7);
  servo04.attach(8);
  servo05.attach(9);
  servo06.attach(10);
  delay(20);
  // Robot arm initial position
  servo1PPos = 90;
  servo01.write(servo1PPos);
  servo2PPos = 150;
  servo02.write(servo2PPos);
  servo3PPos = 35;
  servo03.write(servo3PPos);
  servo4PPos = 140;
  servo04.write(servo4PPos);
  servo5PPos = 85;
  servo05.write(servo5PPos);
  servo6PPos = 80;
  servo06.write(servo6PPos);
}

void loop() {
  Serial.println("Delay started");
  delay( 2000);
  Serial.println("Delay end");
  
  Serial.println("Servo Start.");
  servoSweep(servo01, 0, 90);
  delay(1000);
  servoSweep(servo01, 90, 180);
  delay(1000);
  servoSweep(servo01, 180, 90);
  delay(1000);
  servoSweep(servo01, 90, 0);
  delay(1000);
  Serial.println("Servo End.");
  
}

void servoSweep(Servo myservo, int positionFrom, int positionTo) {
  Serial.print("Servo sweep from  \'");
  Serial.print(positionFrom);
  Serial.print("\' , to \'");
  Serial.print(positionTo);
  Serial.println("\'.");
  int pos = positionFrom;    // variable to store the servo position
  int step = 0;
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

  if(step =1) {
    for (pos = positionFrom; pos <= positionTo; pos += step) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(speedDelay);                       // waits 15ms for the servo to reach the position
    }
  }
  else
  {
    for (pos = positionFrom; pos >= positionTo; pos += step) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(speedDelay);                       // waits 15ms for the servo to reach the position
    }
  }
  Serial.println("  Done.");  
}
