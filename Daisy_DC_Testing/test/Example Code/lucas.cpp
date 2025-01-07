#include <Arduino.h>
#include <Wire.h> // I2C library
#include <driver/twai.h> // ESP32 TWAI driver for CAN communication
#include <EEPROM.h>
#include <PID_v1.h> // Include the PID library








// Pin definitions
#define CURRENT_SENSOR_PIN 5
#define CAN_RX_PIN 2
#define CAN_TX_PIN 1
#define MOTOR_PWM_PIN 3
#define MOTOR_INA_PIN 20
#define MOTOR_INB_PIN 8
#define AS5600_ADDRESS 0x36
#define AS5600_ANGLE_REGISTER 0x0E // High byte of the angle register
#define SDA_PIN 9
#define SCL_PIN 10




//CANbus/TWAI address
#define DEVICE_TWAI_ID 0x123 // Unique TWAI address for this board








float kp = 5.0; // Default proportional gain
float ki = 0.1; // Default integral gain
float kd = 2.0; // Default derivative gain




#define EEPROM_KP_ADDR 8  // EEPROM address for kp
#define EEPROM_KI_ADDR 12 // EEPROM address for ki
#define EEPROM_KD_ADDR 16 // EEPROM address for kd








int targetAngle = 0;
int currentAngle = 0;
int endPoint = 0; // Endpoint determined after zeroing
int zeroOffset = 0; // Offset determined during manual zeroing
float integral = 0.0, previousError = 0.0;
float deadband = 2.0;
int endpointSafezone = 5;
float pidOutput;




// EEPROM addresses for storing zero and endpoint
#define EEPROM_ZERO_ADDR 0
#define EEPROM_END_ADDR 4




// System state
bool isZeroed = false; // Flag to track if the system has been zeroed




void setupPins();
int readAngle();
float computePID(float target, float current);
void controlMotor(float pidOutput);
void receiveSerialInput();
void printLiveData(int currentAngle, int targetAngle, float pidOutput);
void saveToEEPROM();
void loadFromEEPROM();
void performZeroing();
void sendLiveDataTWAI(int target, int current, const char* direction, int pwm);
void receiveTWAIInput();






void setup() {
  setupPins();
  Serial.begin(115200);
  Serial.println("System initialized.");




  EEPROM.begin(512); // Initialize EEPROM with 512 bytes
  loadFromEEPROM();
   // Ensure valid default PID values
  if (kp <= 0 || ki < 0 || kd < 0) {
    kp = 5.0;
    ki = 0.1;
    kd = 2.0;
    Serial.println("Default PID values set: Kp=5.0, Ki=0.1, Kd=2.0");
  }


  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("I2C initialized.");




  // TWAI configuration
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();




  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI Driver installed.");
  } else {
    Serial.println("TWAI Driver installation failed.");
    while (1);
  }




  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("TWAI Driver started.");
  } else {
    Serial.println("Failed to start TWAI Driver.");
    while (1);
  }




 
}




void loop() {
  receiveSerialInput(); // Check for serial input
  receiveTWAIInput();   // Check for TWAI input




  if (!isZeroed || targetAngle == 0) {
    controlMotor(0); // Ensure motors stay off
    return;          // Wait for valid target
  }




  currentAngle = readAngle();
  pidOutput = computePID(targetAngle, currentAngle);
  controlMotor(pidOutput);




  // Update the display with live data
  printLiveData(currentAngle, targetAngle, pidOutput);




  delay(10); // Loop delay for stability
}








void setupPins() {
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  pinMode(MOTOR_INA_PIN, OUTPUT);
  pinMode(MOTOR_INB_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
}




int readAngle() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_ANGLE_REGISTER); // Start reading from the angle register
  if (Wire.endTransmission(false) != 0) { // Repeated start
    Serial.println("I2C communication failed!");
    return currentAngle; // Return last valid angle
  }




  Wire.requestFrom(AS5600_ADDRESS, 2); // Request 2 bytes (high and low)
  if (Wire.available() < 2) {
    Serial.println("I2C data unavailable!");
    return currentAngle; // Return last valid angle
  }




  // Combine high and low bytes into a raw 12-bit angle value
  int rawAngle = (Wire.read() << 8) | Wire.read();




  // Convert raw 12-bit angle to degrees
  int angleInDegrees = (rawAngle * 360) / 4096;




  // Apply zero offset
  int adjustedAngle = angleInDegrees - zeroOffset;




  // Handle rollover for extended angles
  if (adjustedAngle < 0) adjustedAngle += 360;
  if (adjustedAngle >= 360) adjustedAngle -= 360;




  return adjustedAngle;
}




float computePID(float target, float current) {
  // Ensure no movement before zeroing
  if (!isZeroed) {
    return 0.0;
  }




  // Adjust target if it is too close to zeroOffset or endPoint
  if (target < endpointSafezone) {
    //Serial.println("Warning: Target angle too close to zero. Adjusting target.");
    target = endpointSafezone;
  } else if (target > endPoint - endpointSafezone) {
    //Serial.println("Warning: Target angle too close to endpoint. Adjusting target.");
    target = endPoint - endpointSafezone;
  }




  // Correct for out-of-range angles
  if (current > 180) {
    return 255.0; // Drive forward at full speed to bring the angle back
  }




  // Calculate the error (distance to target)
  float error = target - current;




  // Define a deadband to prevent unnecessary motor activity near the target
  if (abs(error) < deadband) { // Deadband: no movement if error is within ±deadband
    return 0.0;
  }




  // Integral term with anti-windup
  integral += error;
  integral = constrain(integral, -1000, 1000); // Prevent integral windup




  // Derivative term
  float derivative = error - previousError;
  previousError = error;




  // Calculate PID output
  float output = (kp * error) + (ki * integral) + (kd * derivative);




  // Clamp the output to valid PWM range
  return constrain(output, -255, 255);
}




















void controlMotor(float pidOutput) {
  if (targetAngle == 0 || pidOutput == 0) {
    // Ensure motors are off if no target or no PID output
    digitalWrite(MOTOR_INA_PIN, LOW);
    digitalWrite(MOTOR_INB_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, 0);
    return;
  }




  if (pidOutput > 0) {
    digitalWrite(MOTOR_INA_PIN, HIGH);
    digitalWrite(MOTOR_INB_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, (int)pidOutput); // Forward PWM
  } else {
    digitalWrite(MOTOR_INA_PIN, LOW);
    digitalWrite(MOTOR_INB_PIN, HIGH);
    analogWrite(MOTOR_PWM_PIN, (int)(-pidOutput)); // Backward PWM
  }
}












void receiveSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any trailing whitespace




    if (input.equalsIgnoreCase("ZERO")) {
      performZeroing();
    } else if (input.toInt() >= 0 && input.toInt() <= 360) {
      if (!isZeroed) {
        Serial.println("System is not zeroed. Type 'ZERO' to set zero offset.");
        return;
      }




      targetAngle = input.toInt();
      Serial.print("New target angle: ");
      Serial.println(targetAngle);
    } else if (input.startsWith("KP=")) {
      kp = input.substring(3).toFloat();
      saveToEEPROM();
      Serial.print("Updated Kp and saved to EEPROM: ");
      Serial.println(kp);
    } else if (input.startsWith("KI=")) {
      ki = input.substring(3).toFloat();
      saveToEEPROM();
      Serial.print("Updated Ki and saved to EEPROM: ");
      Serial.println(ki);
    } else if (input.startsWith("KD=")) {
      kd = input.substring(3).toFloat();
      saveToEEPROM();
      Serial.print("Updated Kd and saved to EEPROM: ");
      Serial.println(kd);
    } else {
      Serial.println("Invalid input. Use 'ZERO', a number (0-360) for target angle, or 'KP=value', 'KI=value', 'KD=value' to set PID parameters.");
    }
  }
}




void performZeroing() {
  Serial.println("Zeroing process started...");
  int initialAngle = readAngle();
  int direction = initialAngle > 180 ? -1 : 1; // Determine direction to decrease the angle




  // Drive motor at 25% PWM for 5 seconds to find zero
  int pwmValue = 64; // 25% of 255
  if (direction == 1) {
    digitalWrite(MOTOR_INA_PIN, LOW);
    digitalWrite(MOTOR_INB_PIN, HIGH);
    analogWrite(MOTOR_PWM_PIN, pwmValue);
  } else {
    digitalWrite(MOTOR_INA_PIN, HIGH);
    digitalWrite(MOTOR_INB_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, pwmValue);
  }
  delay(2000); // Drive motor for 5 seconds




  // Set zero offset
  zeroOffset = readAngle();




  digitalWrite(MOTOR_INA_PIN, LOW);
  digitalWrite(MOTOR_INB_PIN, LOW);
  analogWrite(MOTOR_PWM_PIN, 0);




  // Find endpoint by driving in the opposite direction
  Serial.println("Finding endpoint...");
  direction *= -1;
  if (direction == 1) {
    digitalWrite(MOTOR_INA_PIN, LOW);
    digitalWrite(MOTOR_INB_PIN, HIGH);
    analogWrite(MOTOR_PWM_PIN, pwmValue);
  } else {
    digitalWrite(MOTOR_INA_PIN, HIGH);
    digitalWrite(MOTOR_INB_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, pwmValue);
  }
  delay(2000); // Drive motor for 5 seconds




  // Record endpoint
  endPoint = readAngle();




  digitalWrite(MOTOR_INA_PIN, LOW);
  digitalWrite(MOTOR_INB_PIN, LOW);
  analogWrite(MOTOR_PWM_PIN, 0);




  // Save to EEPROM
  saveToEEPROM();




  isZeroed = true;
  Serial.println("Zeroing complete. Zero and endpoint updated.");
}












void printLiveData(int currentAngle, int targetAngle, float pidOutput) {
  Serial.print("\033[2K"); // Clear the current line
  Serial.print("\r");      // Move the cursor to the beginning of the line




  Serial.print("Target: ");
  Serial.print(targetAngle);
  Serial.print(" | Current: ");
  Serial.print(currentAngle);
  Serial.print(" | PID Output: ");
  Serial.print(pidOutput);
  Serial.print(" | Direction: ");
  if (pidOutput > 0) {
    Serial.print("Forward");
  } else if (pidOutput < 0) {
    Serial.print("Reverse");
  } else {
    Serial.print("Stopped");
  }
  Serial.print(" | PWM: ");
  Serial.print(abs((int)pidOutput)); // Absolute value of PID output for PWM
}








void saveToEEPROM() {
  EEPROM.put(EEPROM_ZERO_ADDR, zeroOffset);
  EEPROM.put(EEPROM_END_ADDR, endPoint);
  EEPROM.put(EEPROM_KP_ADDR, kp);
  EEPROM.put(EEPROM_KI_ADDR, ki);
  EEPROM.put(EEPROM_KD_ADDR, kd);
  EEPROM.commit();
  Serial.println("Values saved to EEPROM.");
}








void loadFromEEPROM() {
  EEPROM.get(EEPROM_ZERO_ADDR, zeroOffset);
  EEPROM.get(EEPROM_END_ADDR, endPoint);
  // EEPROM.get(EEPROM_KP_ADDR, kp);
  // EEPROM.get(EEPROM_KI_ADDR, ki);
  // EEPROM.get(EEPROM_KD_ADDR, kd);




  // if (zeroOffset >= 0 && zeroOffset <= 360 && endPoint >= 0 && endPoint <= 360) {
  //   isZeroed = true;
  //   Serial.println("Zero and endpoint values loaded from EEPROM.");
  // } else {
  //   isZeroed = false; // If values are invalid, require re-zeroing
  //   Serial.println("EEPROM values invalid. Please zero the system.");
  // }




  // if (kp < 0 || ki < 0 || kd < 0 || kp > 10 || ki > 10 || kd > 10) {
  //   kp = 5.0;
  //   ki = 0.1;
  //   kd = 2.0;
  //   Serial.println("Invalid PID values in EEPROM. Resetting to defaults.");
  // } else {
  //   Serial.println("PID values loaded from EEPROM.");
  //   Serial.print("Kp: "); Serial.println(kp);
  //   Serial.print("Ki: "); Serial.println(ki);
  //   Serial.print("Kd: "); Serial.println(kd);
  // }
}








void sendLiveDataTWAI(int target, int current, const char* direction, int pwm) {
  twai_message_t message;
  message.identifier = DEVICE_TWAI_ID;
  message.extd = 0; // Standard frame
  message.rtr = 0;  // Data frame
  message.data_length_code = 8; // Up to 8 bytes




  // Pack data into the message payload
  message.data[0] = (uint8_t)(target & 0xFF);
  message.data[1] = (uint8_t)((target >> 8) & 0xFF);
  message.data[2] = (uint8_t)(current & 0xFF);
  message.data[3] = (uint8_t)((current >> 8) & 0xFF);
  message.data[4] = (strcmp(direction, "Forward") == 0) ? 1 : (strcmp(direction, "Reverse") == 0) ? 2 : 0;
  message.data[5] = (uint8_t)(pwm & 0xFF);
  message.data[6] = (uint8_t)((pwm >> 8) & 0xFF);
  message.data[7] = 0; // Reserved for future use




  if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.println("Live data sent over TWAI.");
  } else {
    Serial.println("Failed to send live data over TWAI.");
  }
}








void receiveTWAIInput() {
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    String input((char*)message.data);




    input.trim(); // Remove any trailing whitespace




    if (input.equalsIgnoreCase("ZERO")) {
      performZeroing();
    } else if (input.toInt() >= 0 && input.toInt() <= 360) {
      if (!isZeroed) {
        Serial.println("System is not zeroed. Type 'ZERO' to set zero offset.");
        return;
      }




      targetAngle = input.toInt();
      Serial.print("New target angle set via TWAI: ");
      Serial.println(targetAngle);




      // Send data back to the master
      const char* direction = (pidOutput > 0) ? "Forward" : (pidOutput < 0) ? "Reverse" : "Stopped";
      sendLiveDataTWAI(targetAngle, currentAngle, direction, abs((int)pidOutput));
    } else if (input.startsWith("KP=")) {
      kp = input.substring(3).toFloat();
      saveToEEPROM();
      Serial.print("Updated Kp via TWAI and saved to EEPROM: ");
      Serial.println(kp);
    } else if (input.startsWith("KI=")) {
      ki = input.substring(3).toFloat();
      saveToEEPROM();
      Serial.print("Updated Ki via TWAI and saved to EEPROM: ");
      Serial.println(ki);
    } else if (input.startsWith("KD=")) {
      kd = input.substring(3).toFloat();
      saveToEEPROM();
      Serial.print("Updated Kd via TWAI and saved to EEPROM: ");
      Serial.println(kd);
    } else {
      Serial.println("Invalid TWAI input. Use 'ZERO', a number (0-360) for target angle, or 'KP=value', 'KI=value', 'KD=value' to set PID parameters.");
    }
  }
}









