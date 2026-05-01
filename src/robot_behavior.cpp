#include "robot_behavior.h"

#include <Arduino.h>

#include "config.h"
#include "huskylens_module.h"
#include "imu_module.h"
#include "motor_control.h"
#include "state.h"

static float smoothedLineOutput = 0.0;
static int lockedAvoidSteer = AVOID_LEFT_STEER;
static bool avoidSteerLocked = false;
static unsigned long lastImuFallbackPrintMs = 0;

static int chooseAvoidSteer() {
  if (frontAngle >= 360.0 - FRONT_HALF_ANGLE) return AVOID_RIGHT_STEER;
  if (frontAngle <= FRONT_HALF_ANGLE) return AVOID_LEFT_STEER;
  return AVOID_LEFT_STEER;
}

static void driveStraightWithImu() {
  if (!isImuReady()) {
    setSteeringServo(servoCenter);
    setEscSpeed(motorSpeed);
    return;
  }

  float currentYaw = getYaw();
  float error = angleError(targetYaw, currentYaw);

  int correction = headingKp * error;
  correction = constrain(correction, -MAX_IMU_CORRECTION, MAX_IMU_CORRECTION);

  // If the robot corrects in the wrong direction, change this to:
  // int steer = servoCenter - correction;
  int steer = servoCenter + correction;

  setSteeringServo(steer);
  setEscSpeed(motorSpeed);

  unsigned long now = millis();
  if (now - lastHeadingDebugAt >= 500) {
    lastHeadingDebugAt = now;
    Serial.print("[IMU heading] targetYaw=");
    Serial.print(targetYaw, 2);
    Serial.print(" currentYaw=");
    Serial.print(currentYaw, 2);
    Serial.print(" error=");
    Serial.print(error, 2);
    Serial.print(" correction=");
    Serial.print(correction);
    Serial.print(" steer=");
    Serial.println(steer);
  }
}

static void driveStraightWithoutImu() {
  setSteeringServo(servoCenter);
  setEscSpeed(motorSpeed);

  unsigned long now = millis();
  if (now - lastImuFallbackPrintMs >= 1000) {
    lastImuFallbackPrintMs = now;
    Serial.println("[IMU] unavailable, using center steering fallback");
  }
}

static void followLine() {
  float error = 0.0;
  if (!readLineError(error)) return;

  if (abs(error) < LINE_ERROR_DEADBAND_PIXELS) {
    error = 0.0;
  }

  unsigned long now = millis();
  float dt = (now - lastPidTime) / 1000.0;
  if (dt <= 0) return;

  errorSum += error * dt;

  float dError = (error - lastError) / dt;
  float output = kp * error + ki * errorSum + kd * dError;
  smoothedLineOutput = (LINE_OUTPUT_SMOOTHING * smoothedLineOutput) + ((1.0 - LINE_OUTPUT_SMOOTHING) * output);

  int targetSteer = servoCenter - smoothedLineOutput;
  int steer = constrain(targetSteer, currentServoPosition - MAX_LINE_SERVO_STEP, currentServoPosition + MAX_LINE_SERVO_STEP);
  setSteeringServo(steer);
  setEscSpeed(motorSpeed);

  lastError = error;
  lastPidTime = now;
}

static void avoidObject() {
  if (!avoidSteerLocked) {
    lockedAvoidSteer = chooseAvoidSteer();
    avoidSteerLocked = true;
  }

  setSteeringServo(lockedAvoidSteer);
  setEscSpeed(motorSpeed);

  if (!frontBlocked) {
    avoidSteerLocked = false;

    if (isImuReady()) {
      targetYaw = getYaw();
    }

    mode = CENTER_AFTER_AVOID;
    centerAfterAvoidStart = millis();
  }
}

static void centerAfterAvoid() {
  if (isImuReady()) {
    driveStraightWithImu();
  } else {
    driveStraightWithoutImu();
  }

  if (frontBlocked) {
    mode = AVOID_OBJECT;
    return;
  }

  if (millis() - centerAfterAvoidStart >= CENTER_AFTER_AVOID_TIME) {
    avoidSteerLocked = false;
    mode = FIND_LINE;
  }
}

static void findLine() {
  setSteeringServo(servoCenter);
  setEscSpeed(motorSpeed);

  if (frontBlocked) {
    mode = AVOID_OBJECT;
    return;
  }

  float error = 0.0;
  if (readLineError(error)) {
    resetPidState();
    mode = FOLLOW_LINE;
  }
}

void setupRobotBehavior() {
  resetPidState();
  smoothedLineOutput = 0.0;
  lockedAvoidSteer = AVOID_LEFT_STEER;
  avoidSteerLocked = false;
  lastImuFallbackPrintMs = 0;
}

void updateRobotBehavior() {
#if ENABLE_ROBOT_BEHAVIOR
  if (emergencyStop) {
    setSteeringServo(servoCenter);
    stopMotor();
    return;
  }

  if (mode == FOLLOW_LINE && frontBlocked) {
    avoidSteerLocked = false;
    mode = AVOID_OBJECT;
  }

  if (mode == FOLLOW_LINE) {
    followLine();
  } else if (mode == AVOID_OBJECT) {
    avoidObject();
  } else if (mode == CENTER_AFTER_AVOID) {
    centerAfterAvoid();
  } else if (mode == FIND_LINE) {
    findLine();
  }
#endif
}
