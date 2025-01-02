#include <Arduino.h>
#include <PID_v1.h> // Include the PID library
#include <CAN.h>    // Include the CAN library for ESP32
//#include <Adafruit_LSM6DS3TRC.h> // IMU library

// Static Values
#define anglePWMPeriod 10000
#define freqAngle 10      // PID timer interup frequency (Hz)
#define freqCurrent 100   // current timer interup frequency (Hz)
#define currentAVGSize 10 // How many samples to use in the current rolling average filter
#define angleAVGSize 10   // How many samples to use in the angle rolling average filter
#define rSize 2200        // Size of resistor used for current feedback in ohms
#define CANRXID 100       // ID for CAN receiving
#define CANTXID 101       // ID for CAN sending
#define CAN_SPEED 500E3   // Set CAN bus speed

// Pin Labels; this is for ESP32-C3 SuperMini from Alliexpress
#define PIN_GPIO2 0
#define PIN_CAN_TX 1
#define PIN_CAN_RX 2
#define PIN_PWM 3
#define PIN_SEL 4
#define PIN_RSENSE 5
#define PIN_DIP_V 6
#define PIN_GPIO_A 7
#define PIN_INB 8
#define PIN_SDA 9
#define PIN_SCL 10
#define PIN_INA 20
#define PIN_GPIO1 21

// Modes:
bool angleTargetEnable = true;
bool currentTargetEnable = true;
bool angleAVGEnable = true;
bool currentAVGEnable = true;

// Config Vars:
int deadzoneFW = 2;  // PWM control value limit to enable braking
int deadzoneBK = -2; // PWM control value limit to enable braking

// Globals:
volatile uint32_t riseMicroTime;
volatile uint32_t fallMicroTime;
byte msgCAN[8];
// MSG Globals:
char msgHeader;
int msgControl;
int msgVal1;
int msgVal2;

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

double Kp2 = 2.0, Ki2 = 0.5, Kd2 = 0.1;
PID motorCurrentPID(&pidInputCurrent, &pidOutputCurrent, &targetCurrent, Kp2, Ki2, Kd2, DIRECT);

// put function declarations here:
void updatePID(bool);
void 
void setupPins();   // Pin declarations and mode setups
int getAngle(bool); // Returns angle from 0 to 3600 (10/ths of a degree)
int setAnglePercentage(int, bool);
void driveMotor(bool, bool, int); // Drive motor with INA,INB,PWM signals
int getCurrent(bool);             // Returns current value in mA from Rsense
bool receiveCAN();
void calibrate(int);

void IRAM_ATTR targetAnglePID();
void IRAM_ATTR targetCurrentPID();
void IRAM_ATTR angleMicroISR();

void setup()
{
  setupPins();
}

void loop()
{
  if (receiveCAN())
  {
    switch (msgHeader)
    {
    case 'A':
      targetAngle = msgControl;
      break;
    case 'C':
      setAnglePercentage(msgControl, true); // Calibrate with current limit of msgControl (mA)
      break;
    case 'P':
      targetAngle = setAnglePercentage(msgControl); // Set target angle form msgControl
      break;
    case '1':
      updatePID(false); // update for outer PID loop (angle)
      break;
    case '2':
      updatePID(true); // update for inner PID loop (current)
      break;
    default:
      break;
    }
  }
  if (angleTargetEnable)
  {
    driveMotorPID();
  }
}
void updatePID(bool PID2) // Update PID1 or PID2 values
{
  // Add EEPROM value saves for theses
  if (PID2)
  {
    Kp2 = msgControl;
    Ki2 = msgVal1;
    Kd2 = msgVal2;
  }
  else
  {
    Kp1 = msgControl;
    Ki1 = msgVal1;
    Kd1 = msgVal2;
  }
}
void sendCAN()
{
  int angle = getAngle(angleAVGEnable); // Example integer
  int current = getCurrent(currentAVGEnable);
  CAN.beginPacket(CANTXID);
  CAN.write((angle >> 8) & 0xFF);
  CAN.write(angle & 0xFF);
  CAN.write((current >> 8) & 0xFF);
  CAN.write(current & 0xFF);
  CAN.endPacket();
}
bool receiveCAN()
{
  // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize)
  {
    if (CAN.packetId() == CANRXID)
    {

      int i = 0;
      while (CAN.available()) // Read all CAN data 8 bytes
      {
        msgCAN[i] = CAN.read();
        i++;
      }
      msgHeader = msgCAN[0];
      msgControl = (msgCAN[1] << 8) | msgCAN[2];
      msgVal1 = (msgCAN[3] << 8) | msgCAN[4];
      msgVal2 = (msgCAN[5] << 8) | msgCAN[6];
      return true;
    }
  }
  return false;
}
int setAnglePercentage(int angle, bool calibrate = false) // Return 0 to 100 val from the range of motion of the joint (after calibration)
{
  static int angle1;
  static int angle2;
  if (calibrate)
  {
    int currentLimit = angle; // Use control value for current limit mA
    driveMotor(true, false, 100);
    unsigned long timeout = millis();
    while (getCurrent() < currentLimit && millis() - timeout < 10000) // 10 second timeout if current limit not reached
      ;
    angle1 = getAngle();
    driveMotor(false, true, 100);
    timeout = millis();
    while (getCurrent() < currentLimit && millis() - timeout < 10000) // 10 second timeout if current limit not reached
      ;
    angle2 = getAngle();
    driveMotor(false, false, 0);

    // Add EEPROM value saves for theses
  }

  return map(angle, 0, 100, angle1, angle2);
}
void driveMotorPID() // Drive the motor with PID with angle or current feedback enabled
{
  if (angleTargetEnable && currentTargetEnable)
  {
    int motorPWM = constrain((int)pidOutputCurrent, -255, 255);
    if (motorPWM < deadzoneBK) // Contorl for backwards values
    {
      driveMotor(false, true, motorPWM * -1);
    }
    else if (motorPWM > deadzoneFW) // Control for fowards values
    {
      driveMotor(true, false, motorPWM);
    }
    else // Used to enable braking if control value is in the deadzone
    {
      driveMotor(true, true, 0);
    }
  }
  else if (angleTargetEnable)
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
    else // motor breaking when in off control
    {
      driveMotor(true, true, 0);
    }
  }
  else
  {
    for (int i = 0; i < 10; i++) // Shake motor to indicate setting error. No PID enabled and PID is called
    {
      driveMotor(false, true, 100);
      delay(100);
      driveMotor(true, false, 100);
      delay(100);
    }
  }
}
void driveMotor(bool INA, bool INB, int PWM) // control H-Bridge motor driver
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
void IRAM_ATTR angleMicroISR()//Triggered interrupt form angle signal
{
  static uint32_t riseTime;
  if (digitalRead(PIN_GPIO_A) == HIGH)
    riseTime = micros();
  else
  {
    fallMicroTime = micros();
    riseMicroTime = riseTime;
  }
}
void IRAM_ATTR targetAnglePID()//Timer ISR for calculating PID for angle
{
  if (angleTargetEnable)
  {
    pidInputAngle = getAngle(angleAVGEnable); // Update PID input with the current angle
    motorAnglePID.Compute();                  // Calculate PID output
  }
}
void IRAM_ATTR targetCurrentPID()//Timer ISR for calculating PID for current
{
  if (currentTargetEnable)
  {
    targetCurrent = pidOutputAngle;                 // This feeds the output of the angle PID into the current PID
    pidInputCurrent = getCurrent(currentAVGEnable); // Update PID input with the current
    motorCurrentPID.Compute();                      // Calculate PID output
  }
}
void setupPins()
{
  pinMode(PIN_INA, OUTPUT);
  pinMode(PIN_INB, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  attachInterrupt(PIN_GPIO_A, angleMicroISR, CHANGE);

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

  // Initialize CAN communication
  CAN.setPins(PIN_CAN_RX, PIN_CAN_TX);
  CAN.begin(CAN_SPEED);
}