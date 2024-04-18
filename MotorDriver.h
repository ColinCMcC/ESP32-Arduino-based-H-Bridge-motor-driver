#ifndef MotorDriver_h
#define MotorDriver_h

#include "Arduino.h"

// Possible directions of the motor
enum MotorDirection {
  FORWARD,
  REVERSE
};

// Possible modes of the motor
enum MotorMode {
  RUN_WITHOUT_ENCODER,
  RUN_TO_POSITION
};

class MotorDriver {
private:
  int _pin1 = 0, _pin2 = 0;                // Motor pins
  int _encoderPinA = 0, _encoderPinB = 0;  // Encoder pins
  int _pwmChA = 0, _pwmChB = 0;            // PWM channels for the motors

  MotorMode _currentMode = RUN_WITHOUT_ENCODER;  // Holds the current mode of the motor. Default is running without encoder

  bool runMotor = true;

  int sampleTime = 10;

  int _deadzone = 10;
  int _targetPos = 0;
  long prevTime;
  float errorSum = 0;
  float errorPrev;
  float u = 0;

  float _kp = 1.0, _ki = 0.0, _kd = 0.0;  // Motor PID constants. Defaults are set here

  MotorDirection _direction = FORWARD;  // Motor direction(Default is forward)

public:
  MotorDriver(int pin1, int pin2, int pwmCh1, int pwmCh2);

  int _encoderCount = 0;  // Current position of the motor

  void setEncoderPins(int pinA, int pinB);                           // Define the encoder pins on the motor
  void setPIDConstants(float kp, float ki, float kd, int deadzone);  // Set the PID constants for the motor
  void setMode(MotorMode mode);                                      // Set the RunMode of the motor
  void setDirection(MotorDirection direction);                       // Set the direction of the motor
  void initializeBuild();                                            // Initialize motor

  void updateMotor();                   // Update motor power
  float pidCalculator(int target);      // Calculate power based of PID for the motor
  void IRAM_ATTR handleMotorEncoder();  // Handles the encoders (IRAM_ATTR stores it in the high speed flash)

  void setTargetPosition(int target);            // Set the target position of the motor
  void setEncoderPosition(int encoderPosition);  // Set the encoder position of the motor
  void resetEncoder();                           // Resets the encoder count to 0
  void setPower(int power);                      // Set the power of the motor directly

  int getTargetPosition();        // Return the current target of the motor
  int getCurrentPosition();       // Retuns the current position of the motor
  MotorDirection getDirection();  // Returns the direction of the motor
  bool isBusy();                  // Return true if the motor is within 10 counts of the target position
};

#endif