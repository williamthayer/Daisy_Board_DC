#include <Arduino.h>
#define PIN_INA 20
#define PIN_INB 8
#define PIN_PWM 3
#define PIN_RSENSE 5

// put function declarations here:
void setupPins();
int driveMotor(bool, bool, int);
int getCurrent();


void setup() {
  setupPins();
  
}

void loop() {
  delay(1000);
  driveMotor(true, false, 200);
  delay(1000);
  driveMotor(false, true, 200);
  delay(1000);
  driveMotor(true, true, 0);
}

// control H-Bridge motor driver
int driveMotor(bool INA, bool INB, int PWM)
{
  digitalWrite(PIN_INA,INA);
  digitalWrite(PIN_INB,INB);
  analogWrite(PIN_PWM,PWM);
  return getCurrent();
}

int getCurrent()
{
  static int rSize = 2200;
  int adjustment = rSize*100/293;
  return analogRead(PIN_RSENSE) * rSize / adjustment;
}

void setupPins()
{
pinMode(PIN_INA,OUTPUT); 
pinMode(PIN_INB,OUTPUT); 
pinMode(PIN_PWM,OUTPUT); 
}