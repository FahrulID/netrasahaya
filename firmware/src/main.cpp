#include <Arduino.h>
#include <AFMotor.h>

// commandBuffer example, 66 00 00 FF FF FF FF 00 99
// commandBuffer consist of 9 bytes
// 1st byte: 0x66 (start byte)
// 2nd byte: 0x00 (motor state of motor 1 and 2)
// 3rd byte: 0x00 (motor state of motor 3 and 4)
// 4th byte: 0xFF (motor speed of motor 1)
// 5th byte: 0xFF (motor speed of motor 2)
// 6th byte: 0xFF (motor speed of motor 3)
// 7th byte: 0xFF (motor speed of motor 4)
// 8th byte: 0x00 (hash byte, 0x00 = 0x00 ^ 0x00 ^ 0xFF ^ 0xFF ^ 0xFF ^ 0xFF)
// 9th byte: 0x99 (end byte)
// Motor state 0 = stop, 1 = manual forward, 2 = slow forward, 3 = fast forward, 4 = manual backward, 5 = slow backward, 6 = fast backward
const int COMMAND_LENGTH = 9;
byte commandBuffer[COMMAND_LENGTH];
const int STOP_SPEED = 0;
const int SLOW_SPEED = 100;
const int FAST_SPEED = 255;
const bool DEBUG = true;
unsigned long currentMillis;

// Example => Motor 1 speed 255 state 0
char debugBuffer[64];
const unsigned long debugPeriod = 1000;  // Period in milliseconds
unsigned long lastDebugMillis = 0; // Last time debug printed 

// Motors
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
AF_DCMotor motors[] = {motor1, motor2, motor3, motor4};
int motorSpeed[] = {0, 0, 0, 0};
int motorState[] = {RELEASE, RELEASE, RELEASE, RELEASE};

void pushBuffer(byte data);
bool isCommandValid();
void applyCommand();

void disableAllMotor();
void enableAllMotor();
void runMotor();
void debugMotor();

void setup() {
  Serial.begin(115200);

  disableAllMotor();

  commandBuffer[0] = 0x66;
  commandBuffer[COMMAND_LENGTH - 1] = 0x99;
}

void loop() {
  currentMillis = millis();
  if (currentMillis - lastDebugMillis >= debugPeriod)
  {
    lastDebugMillis = currentMillis;

    if(DEBUG)
      debugMotor();

    sprintf(debugBuffer, "Command: %02X %02X %02X %02X %02X %02X %02X %02X %02X Valid: %d\n", commandBuffer[0], commandBuffer[1], commandBuffer[2], commandBuffer[3], commandBuffer[4], commandBuffer[5], commandBuffer[6], commandBuffer[7], commandBuffer[8], isCommandValid());
    Serial.print(debugBuffer);
  }

  if(!Serial.available())
    return;

  char c = Serial.read();
  pushBuffer(c);

  bool isValid = isCommandValid();
  if (!isValid) {
    return;
  }

  applyCommand();

  runMotor();
}

void pushBuffer(byte data) {
  for (int i = 0; i < COMMAND_LENGTH - 1; i++) {
    commandBuffer[i] = commandBuffer[i + 1];
  }

  commandBuffer[COMMAND_LENGTH - 1] = data;
}

bool isCommandValid() {
  // Command is valid if the first byte is 0x66 and the last byte is 0x99 and the hash byte is correct
  if (commandBuffer[0] == 0x66 && commandBuffer[COMMAND_LENGTH - 1] == 0x99) {
    byte hash = 0x00;
    for (int i = 1; i < COMMAND_LENGTH - 2; i++) {
      hash ^= commandBuffer[i];
    }

    if (hash == commandBuffer[COMMAND_LENGTH - 2]) {
      return true;
    }
  }

  return false;
}

void applyCommand() {
  int speedByteStartIndex = 0;

  for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    // byteIndex based on motor number, index 1 = motor 1 and 2, index 2 = motor 3 and 4
    int byteIndex = ((int) (i / 2)) + 1;
    int state = 0;

    if (i % 2 == 0)
      state = (int) (commandBuffer[byteIndex] & 0xF0) / 16; // get high nibble
    else
      state = (int) (commandBuffer[byteIndex] & 0x0F); // get low nibble

    switch(state) {
      case 0:
        motorState[i] = RELEASE;
        motorSpeed[i] = STOP_SPEED;
        break;
      case 1:
        motorState[i] = FORWARD;
        motorSpeed[i] = STOP_SPEED;
        break;
      case 2:
        motorState[i] = FORWARD;
        motorSpeed[i] = SLOW_SPEED;
        break;
      case 3:
        motorState[i] = FORWARD;
        motorSpeed[i] = FAST_SPEED;
        break;
      case 4:
        motorState[i] = BACKWARD;
        motorSpeed[i] = STOP_SPEED;
        break;
      case 5:
        motorState[i] = BACKWARD;
        motorSpeed[i] = SLOW_SPEED;
        break;
      case 6:
        motorState[i] = BACKWARD;
        motorSpeed[i] = FAST_SPEED;
        break;
    }

    // check if last index
    if (i == sizeof(motors) / sizeof(motors[0]) - 1) {
      speedByteStartIndex = byteIndex + 1;
    }
  }

  for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    int byteIndex = speedByteStartIndex + i;
    int speed = (int) commandBuffer[byteIndex];

    if(DEBUG)
    {
      sprintf(debugBuffer, "Motor: %d Byte Index: %d Speed Byte: %02X Speed: %d\n", i, byteIndex, commandBuffer[byteIndex], speed);
      Serial.print(debugBuffer);
    }

    // Only apply speed when the speed is not 0 (stop) and override the value from states
    if (speed != 0 && motorState[i] != RELEASE) {
      motorSpeed[i] = speed;
    }
  }
}

void disableAllMotor() {
  for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    motors[i].run(RELEASE);
  }
}

void enableAllMotor() {
  for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    motors[i].run(FORWARD);
  }
}

void runMotor() {
  for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    motors[i].run(motorState[i]);
    motors[i].setSpeed(motorSpeed[i]);
  }
}

void debugMotor() {
  for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    sprintf(debugBuffer, "Motor %d Motor Speed %d Motor State %d \n", i, motorSpeed[i], motorState[i]);
    Serial.print(debugBuffer);
  }
}