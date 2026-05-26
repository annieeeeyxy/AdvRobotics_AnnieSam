#include "motor_control.h"

#include <Arduino.h>
#include <Servo.h>

#include "config.h"
#include "state.h"

static Servo steerServo;
static Servo escMotor;

void setupMotorControl() {
  steerServo.attach(SERVO_PIN);
  escMotor.attach(ESC_PIN);

  setSteeringServo(servoCenter);
  stopMotor();
}

void setSteeringServo(int angle) {
  currentServoPosition = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  steerServo.write(currentServoPosition);
}

void setEscSpeed(int speed) {
  motorSpeed = constrain(speed, 0, 180);
  currentEscOutput = motorSpeed;
  escMotor.write(currentEscOutput);
}

void stopMotor() {
  currentEscOutput = ESC_NEUTRAL;
  escMotor.write(currentEscOutput);
}
