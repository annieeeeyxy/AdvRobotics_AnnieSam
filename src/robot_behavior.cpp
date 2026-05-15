#include "robot_behavior.h"

#include <Arduino.h>

#include "config.h"
#include "huskylens_module.h"
#include "imu_module.h"
#include "motor_control.h"
#include "state.h"

static int lockedAvoidSteer = AVOID_LEFT_STEER;
static bool avoidSteerLocked = false;
static unsigned long lastImuFallbackPrintMs = 0;
static unsigned long yellowLineLostStartMs = 0;
static int lastYellowLineSteer = DEFAULT_SERVO_CENTER;

static int chooseAvoidSteer() {
  if (frontAngle >= 360.0 - FRONT_HALF_ANGLE) return AVOID_RIGHT_STEER;
  if (frontAngle <= FRONT_HALF_ANGLE) return AVOID_LEFT_STEER;
  return AVOID_LEFT_STEER;
}

static void driveStraightWithImu() {
  if (imuReadOk != true) {
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

static void followYellowLine() {
  YellowLineBlock block;
  int error = 0;
  if (!readYellowLineBlock(block, error)) {
    unsigned long now = millis();
    if (yellowLineLostStartMs == 0) {
      yellowLineLostStartMs = now;
    }

    if (now - yellowLineLostStartMs < YELLOW_LINE_LOST_GRACE_MS) {
      // Hold the last good heading through brief HUSKYLENS dropouts.
      if (imuReadOk == true) {
        driveStraightWithImu();
      } else {
        setSteeringServo(lastYellowLineSteer);
        setEscSpeed(motorSpeed);
      }
      return;
    }

    setSteeringServo(servoCenter);
    stopMotor();
    mode = FIND_COLOR;
    return;
  }

  yellowLineLostStartMs = 0;

  int steer = servoCenter;
  if (abs(error) > yellowLineDeadbandPixels) {
    unsigned long now = millis();
    float dt = (now - lastPidTime) / 1000.0;
    if (dt <= 0.0) {
      dt = 0.001;
    }

    errorSum += error * dt;
    errorSum = constrain(errorSum, -300.0, 300.0);

    float derivative = (error - lastError) / dt;
    int correction = kp * error + ki * errorSum + kd * derivative;
    correction = constrain(correction, -yellowLineMaxTurn, yellowLineMaxTurn);
    steer = servoCenter - correction;
    lastError = error;
    lastPidTime = now;

    Serial.print("[YELLOW LINE] errorX=");
    Serial.print(error);
    Serial.print(" correction=");
    Serial.print(correction);
    Serial.print(" steer=");
    Serial.println(steer);

    if (imuReadOk == true) {
      targetYaw = getYaw();
    }
  } else {
    Serial.println("[YELLOW LINE] Line centered. Going straight.");
    if (imuReadOk == true) {
      driveStraightWithImu();
      lastYellowLineSteer = currentServoPosition;
      return;
    }
  }

  setSteeringServo(steer);
  lastYellowLineSteer = steer;
  setEscSpeed(motorSpeed);
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

    if (imuReadOk == true) {
      targetYaw = getYaw();
    }

    mode = CENTER_AFTER_AVOID;
    centerAfterAvoidStart = millis();
  }
}

static void centerAfterAvoid() {
  if (imuReadOk == true) {
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
    mode = FIND_COLOR;
  }
}

static void findYellowLine() {
  setSteeringServo(servoCenter);
  stopMotor();

  if (frontBlocked) {
    mode = AVOID_OBJECT;
    return;
  }

  YellowLineBlock block;
  int error = 0;
  if (readYellowLineBlock(block, error)) {
    resetPidState();
    mode = FOLLOW_COLOR;
  }
}

void setupRobotBehavior() {
  resetPidState();
  mode = FOLLOW_COLOR;
  emergencyStop = !START_LINE_TRACKING_ON_BOOT;
  setSteeringServo(servoCenter);
  if (emergencyStop) {
    stopMotor();
  }
  lockedAvoidSteer = AVOID_LEFT_STEER;
  avoidSteerLocked = false;
  lastImuFallbackPrintMs = 0;
  yellowLineLostStartMs = 0;
  lastYellowLineSteer = servoCenter;
}

void updateRobotBehavior() {
#if ENABLE_ROBOT_BEHAVIOR
  if (emergencyStop) {
    setSteeringServo(servoCenter);
    stopMotor();
    return;
  }

  if (ENABLE_LIDAR && mode == FOLLOW_COLOR && frontBlocked) {
    avoidSteerLocked = false;
    mode = AVOID_OBJECT;
  }

  if (mode == FOLLOW_COLOR) {
    followYellowLine();
  } else if (ENABLE_LIDAR && mode == AVOID_OBJECT) {
    avoidObject();
  } else if (mode == CENTER_AFTER_AVOID) {
    centerAfterAvoid();
  } else if (mode == FIND_COLOR) {
    findYellowLine();
  }
#endif
}
