#include "Arduino.h"
/*
 * Original sourse: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
 * This is the Arduino code PAC6985 16 channel servo controller
 * watch the video for details and demo http://youtu.be/y8X9X10Tn1k
 *  * 
 
 * Watch video for this code: 
 * 
 * Related Videos
V5 video of PCA9685 32 Servo with ESP32 with WiFi https://youtu.be/bvqfv-FrrLM
V4 video of PCA9685 32 Servo with ESP32 (no WiFi): https://youtu.be/JFdXB8Za5Os
V3 video of PCA9685 how to control 32 Servo motors https://youtu.be/6P21wG7N6t4
V2 Video of PCA9685 3 different ways to control Servo motors: https://youtu.be/bal2STaoQ1M
V1 Video introduction to PCA9685 to control 16 Servo  https://youtu.be/y8X9X10Tn1k

 * Written by Ahmad Shamshiri for Robojax Video channel www.Robojax.com
 * Date: Dec 16, 2017, in Ajax, Ontario, Canada
 * Permission granted to share this code given that this
 * note is kept with the code.
 * Disclaimer: this code is "AS IS" and for educational purpose only.
 * this code has been downloaded from http://robojax.com/learn/arduino/
 * 
 */
/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN1  130
#define SERVOMAX1  630

#define SERVOMIN2  180
#define SERVOMAX2  610

#define SERVOMIN3  120 //160 = 0    minimum 120
#define SERVOMAX3  610 //650 = 180

#define SERVOMIN4  120
#define SERVOMAX4  620

#define SERVOMIN5  150
#define SERVOMAX5  620

#define SERVOMIN6  155
#define SERVOMAX6  575

// our servo # counter
uint8_t servonum = 0;
int step = 15;
int wait =100;

#define Angle2Min 30

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  Serial.println("Step = "+String(step));
  //yield();
  wait = 500;//int(round(000 /(180/step)));
  Serial.println("Wait = "+String(wait));
}

// the code inside loop() has been updated by Robojax
void loop() {

Serial.println("---------forward----------------------------------------------------------------------");
int angle0 = 180;
Serial.println("Part1 Up, UP..........................................");
  for( int angle =0; angle<180; angle +=step){
    for( int angle2 =(180); angle2>=Angle2Min; angle2 -=step){
      callMoveSservo(angle, angle2);
    }
    Serial.println("Part2 Up, Down........................................");
    for( int angle2 =Angle2Min; angle2<(180); angle2 +=step){
      callMoveSservo(angle, angle2);
    }
  }
  Serial.println("---------back----------------------------------------------------------------------");

  //delay(wait);
  Serial.println("Part3 down, UP....................................");
  for( int angle =180; angle>=0; angle -=step){
    for( int angle2 =(180); angle2>=Angle2Min; angle2 -=step){
      callMoveSservo(angle, angle2);
    }
    Serial.println("Part4 Down, Down..................................");
    for( int angle2 =Angle2Min; angle2<(180); angle2 +=step){
      callMoveSservo(angle, angle2);
    }
  }
  Serial.println("----------end----------------------------------------------------------------------");

  delay(wait);
 
}

void callMoveSservo(int angle, int angle2) {
      moveServos(0, angle, angle2, 90, ((180 - angle) + (angle2 - 180)), 90);
      Serial.println("......");
      delay(wait);
}

void moveServos(int angle1, int angle2, int angle3, int angle4, int angle5, int angle6) {
      pwm.setPWM(1, 0, angleToPulse(angle1, SERVOMIN1, SERVOMAX1, false,"1 ") );
      pwm.setPWM(2, 0, angleToPulse(angle2, SERVOMIN2, SERVOMAX2, true ,"2 ") );
      pwm.setPWM(3, 0, angleToPulse(angle3, SERVOMIN3, SERVOMAX3, true ,"3 ") );
      pwm.setPWM(4, 0, angleToPulse(angle4, SERVOMIN4, SERVOMAX4, false,"4 ") );
      pwm.setPWM(5, 0, angleToPulse(angle5, SERVOMIN5, SERVOMAX5, true ,"5 ") );
      pwm.setPWM(6, 0, angleToPulse(angle6, SERVOMIN6, SERVOMAX6, false,"6 ") );

}

/*
 * angleToPulse(int ang)
 * gets angle in degree and returns the pulse width
 * also prints the value on seial monitor
 * written by Ahmad Nejrabi for Robojax, Robojax.com
 */
int angleToPulse(int ang, long servoMin, long servoMax, bool showLog, String TextToAdd){
   int ang2 = constrain(ang, 0, 180);
   int pulse = map(ang2 , 0, 180, servoMin, servoMax);// SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   if(showLog) {
    Serial.println(String(TextToAdd)+"Angle: "+String(ang)+", Angle2: "+String(ang2)+", Pulse: "+String(pulse));
   }
   return pulse;
}
