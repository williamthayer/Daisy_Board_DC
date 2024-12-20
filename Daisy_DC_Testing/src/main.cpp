#include <Arduino.h>

// put function declarations here:
void setupPins();
int driveMotor(bool, bool, int);
int getCurrent();

void setup() {
  // put your setup code here, to run once:
  delay(1000);
  driveMotor(true, false, 200);
  delay(1000);
  driveMotor(false, true, 200);
  delay(1000);
  driveMotor(true, true, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// control H-Bridge motor driver
int driveMotor(bool INA, bool INB, int PWM) 
{
  digitalWrite(3,INA);
  digitalWrite(6,INB);
  analogWrite(11,PWM);
  return getCurrent();
}

int getCurrent()
{
  static int rSize = 2200;


}

void setupPins()
{
pinMode(3,OUTPUT); //INA
pinMode(6,OUTPUT); //INB
pinMode(11,OUTPUT); //PWM
}