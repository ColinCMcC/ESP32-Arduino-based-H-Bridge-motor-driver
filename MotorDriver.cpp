#include "MotorDriver.h"

// This library is designed to control a motor either with or without encoders
// It is highly inspired by the DcMotor controls used in FIRST Tech Challenge
//================================================================================================================================//

// Initial variable asignments when an instance is created
MotorDriver::MotorDriver(int pin1, int pin2, int pwmCh1, int pwmCh2) {
  _pin1 = pin1;
  _pin2 = pin2;

  _pwmChA = pwmCh1;
  _pwmChB = pwmCh2;
}

// Define the encoder pins on the motor
void MotorDriver::setEncoderPins(int pinA, int pinB) {
  _encoderPinA = pinA;
  _encoderPinB = pinB;
}

// Set the PID constants for the motor
void MotorDriver::setPIDConstants(float kp, float ki, float kd, int deadzone) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _deadzone = deadzone;
}

// Set the RunMode of the motor
void MotorDriver::setMode(MotorMode mode) {
  _currentMode = mode;
}

// Set the direction of the motor
void MotorDriver::setDirection(MotorDirection direction) {
  _direction = direction;
}

// Initialize motor
void MotorDriver::initializeBuild() {
  // Creates the PWM channels
  ledcSetup(_pwmChA, 1000, 8);
  ledcSetup(_pwmChB, 1000, 8);

  // Attaches the pwm channels to the pins
  ledcAttachPin(_pin1, _pwmChA);
  ledcAttachPin(_pin2, _pwmChB);
}

// Update motor power
void MotorDriver::updateMotor() {
  setPower(pidCalculator(_targetPos));
}

// Calculates PID for the motor
float MotorDriver::pidCalculator(int target) {
  long currentTime = micros();
  float deltaT = ((float)(currentTime - prevTime)) / 1.0e6;

  int error = target - _encoderCount;                    // Proportional
  errorSum += error;                                     // Integral
  float errorDerivative = (error - errorPrev) / deltaT;  // Derivative

  float u = (_kp * error) + (_ki * errorSum) + (_kd * errorDerivative);

  // Sets the deadzone of the motor
  if (abs(_encoderCount - target) <= _deadzone) {
    u = 0;
    errorSum = 0;
  }

  prevTime = currentTime;
  errorPrev = error;

  // Makes sure u is between -255 and 255
  u = constrain(u, -255, 255);

  return u;
}

// Handles the encoders
void IRAM_ATTR MotorDriver::handleMotorEncoder() {
  if (_direction == FORWARD) {
    if (digitalRead(_encoderPinA) > digitalRead(_encoderPinB)) {
      _encoderCount++;
    } else {
      _encoderCount--;
    }
  } else {
    if (digitalRead(_encoderPinA) > digitalRead(_encoderPinB)) {
      _encoderCount--;
    } else {
      _encoderCount++;
    }
  }
}

// Set the target position for the motor
void MotorDriver::setTargetPosition(int targetPos) {
  _targetPos = targetPos;
}

// Override the position of the encoder
void MotorDriver::setEncoderPosition(int encoderPosition) {
  _encoderCount = encoderPosition;
}

// Resets the encoder count to 0
void MotorDriver::resetEncoder() {
  _encoderCount = 0;
}

// Set the power of the motor directly
void MotorDriver::setPower(int power) {

  // If the motor is in reverse mode, then switch the direction it is running
  if (_direction == REVERSE) {
    power = -power;
  }

  float speed = fabs(power);
  if (speed > 255) {
    speed = 255;
  }

  if (power > 0) {
    ledcWrite(_pwmChA, speed);
    ledcWrite(_pwmChB, 0);
  } else {
    ledcWrite(_pwmChA, 0);
    ledcWrite(_pwmChB, speed);
  }
}

// Return the current target of the motor
int MotorDriver::getTargetPosition() {
  return _targetPos;
}

// Retuns the current position of the motor
int MotorDriver::getCurrentPosition() {
  return _encoderCount;
}

// Returns the direction of the motor
MotorDirection MotorDriver::getDirection() {
  if (_direction == FORWARD) {
    return FORWARD;
  } else {
    return REVERSE;
  }
}

// Return true if the motor is not within 10 counts of the target position
bool MotorDriver::isBusy() {
  bool busy = false;
  if (abs(_targetPos - _encoderCount) >= _deadzone) {

    busy = true;
  } else {
    busy = false;
  }

  return busy;
}