#include <Arduino.h>
#include <PID_v1.h> // Include the PID library
// Static Values
#define anglePWMPeriod 10000
#define freqAngle 10      // PID timer interup frequency (Hz)
#define freqCurrent 100   // current timer interup frequency (Hz)
#define currentAVGSize 10 // How many samples to use in the current rolling average filter
#define angleAVGSize 10   // How many samples to use in the angle rolling average filter
#define rSize 2200        // Size of resistor used for current feedback in ohms

// Pin Labels
#define PIN_INA 20
#define PIN_INB 8
#define PIN_PWM 3
#define PIN_RSENSE 5
#define GPIO_A 7

// Modes:
bool angleTarget = true;
bool currentTarget = true;
bool angleAVG = true;
bool currentAVG = true;

// Globals:
volatile uint32_t riseMicroTime;
volatile uint32_t fallMicroTime;

// PID variables
double targetAngle = 180.0; // Desired target angle (0-360 degrees)
double pidInputAngle = 0;   // Current angle (feedback)
double pidOutputAngle = 0;  // PID output (motor control signal)
// PID variables
double targetCurrent = 500;  // Desired target current (mA)
double pidInputCurrent = 0;  // Current current (feedback)
double pidOutputCurrent = 0; // PID output (motor control signal)
// Timer for PID updates
hw_timer_t *timerAngle = NULL;
hw_timer_t *timerCurrent = NULL;

// PID Tuning Parameters
// PID values used for current control: Kp1 = 20.0, Ki1 = 5, Kd1 = 1;
// PID values used for angle control:   Kp1 = 2.0, Ki1 = 0.5, Kd1 = 0.1;
double Kp1 = 2.0, Ki1 = 0.5, Kd1 = 0.1;
PID motorAnglePID(&pidInputAngle, &pidOutputAngle, &targetAngle, Kp1, Ki1, Kd1, DIRECT);

double Kp1 = 2.0, Ki1 = 0.5, Kd1 = 0.1;
PID motorCurrentPID(&pidInputCurrent, &pidOutputCurrent, &targetCurrent, Kp1, Ki1, Kd1, DIRECT);

// put function declarations here:
void setupPins();                 // Pin declarations and mode setups
int getAngle(bool);               // Returns angle from 0 to 3600 (10/ths of a degree)
void driveMotor(bool, bool, int); // Drive motor with INA,INB,PWM signals
int getCurrent(bool);             // Returns current value in mA from Rsense
void IRAM_ATTR targetAnglePID();
void IRAM_ATTR targetCurrentPID();
void IRAM_ATTR angleMicroISR();

void setup()
{
  setupPins();
}

void loop()
{
  targetAngle = CANREAD();
  if (angleTarget)
  {
    driveMotorPID();
  }
}
void driveMotorPID() // Drive the motor with PID with angle or current feedback enabled
{
  if (angleTarget && currentTarget)
  {
    int motorPWM = constrain((int)pidOutputCurrent, -255, 255);
    if (motorPWM < -2)
    {
      driveMotor(false, true, motorPWM * -1);
    }
    else if (motorPWM > 2)
    {
      driveMotor(true, false, motorPWM);
    }
    else //motor breaking when in off control 
    {
      driveMotor(true, true, 0);
    }
  }
  else if (angleTarget)
  {
    int motorPWM = constrain((int)pidOutputAngle, -255, 255);
    if (motorPWM < -2)
    {
      driveMotor(false, true, motorPWM * -1);
    }
    else if (motorPWM > 2)
    {
      driveMotor(true, false, motorPWM);
    }
    else//motor breaking when in off control 
    {
      driveMotor(true, true, 0);
    }
  }
}

void driveMotor(bool INA, bool INB, int PWM)// control H-Bridge motor driver
{
  digitalWrite(PIN_INA, INA);
  digitalWrite(PIN_INB, INB);
  analogWrite(PIN_PWM, PWM);
}
int getAngle(bool avg = false)
{
  noInterrupts();
  uint32_t highPeriod = fallMicroTime - riseMicroTime;

  int bitTime = anglePWMPeriod / 4351;
  int dataPeriod = highPeriod - 128 * bitTime;
  int angle = 3600 * dataPeriod / (4095 * bitTime);
  interrupts();

  static long sum;
  static int currentIndex = 0;
  static int readings[angleAVGSize];
  if (avg)
  {
    sum -= readings[currentIndex];                    // Subtract the oldest value from the sum
    readings[currentIndex] = angle;                   // Replace it with the new value
    sum += angle;                                     // Add the new value to the sum
    currentIndex = (currentIndex + 1) % angleAVGSize; // Move to the next index
    return sum / angleAVGSize;
  }
  return angle;
}
int getCurrent(bool avg = false) // Return the current in mA with or without a 8 sample avg
{
  int adjustment = rSize * 100 / 293;
  long currentMa = analogRead(PIN_RSENSE) * rSize / adjustment;

  static long sum;
  static int currentIndex = 0;
  static int readings[currentAVGSize];
  if (avg)
  {
    sum -= readings[currentIndex];                      // Subtract the oldest value from the sum
    readings[currentIndex] = currentMa;                 // Replace it with the new value
    sum += currentMa;                                   // Add the new value to the sum
    currentIndex = (currentIndex + 1) % currentAVGSize; // Move to the next index
    return sum / currentAVGSize;
  }

  return currentMa;
}
void IRAM_ATTR angleMicroISR()
{
  static uint32_t riseTime;
  if (digitalRead(GPIO_A) == HIGH)
    riseTime = micros();
  else
  {
    fallMicroTime = micros();
    riseMicroTime = riseTime;
  }
}
void IRAM_ATTR targetAnglePID()
{
  if (angleTarget)
  {
    pidInputAngle = getAngle(angleAVG); // Update PID input with the current angle
    motorAnglePID.Compute();            // Calculate PID output
  }
}
void IRAM_ATTR targetCurrentPID()
{
  if (currentTarget)
  {
    targetCurrent = pidOutputAngle;         // This feeds the output of the angle PID into the current PID
    pidInputCurrent = getCurrent(currentAVG); // Update PID input with the current
    motorCurrentPID.Compute();                // Calculate PID output
  }
}
void setupPins()
{
  pinMode(PIN_INA, OUTPUT);
  pinMode(PIN_INB, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  attachInterrupt(GPIO_A, angleMicroISR, CHANGE);

  // Timer for Angle PID
  timerAngle = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 µs tick)
  timerAttachInterrupt(timerAngle, &targetAnglePID, true);
  timerAlarmWrite(timerAngle, 1000000 / freqAngle, true); // freqPID interval in Hz
  timerAlarmEnable(timerAngle);                           // Enable the timer

  // Timer for Current PID
  timerCurrent = timerBegin(1, 80, true); // Timer 1, prescaler 80 (1 µs tick)
  timerAttachInterrupt(timerCurrent, &targetCurrentPID, true);
  timerAlarmWrite(timerCurrent, 1000000 / freqCurrent, true); // freqPID interval in Hz
  timerAlarmEnable(timerCurrent);                             // Enable the timer
}