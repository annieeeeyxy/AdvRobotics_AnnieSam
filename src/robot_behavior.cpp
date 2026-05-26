#include "robot_behavior.h"

#include <Arduino.h>
#include <TinyGPSPlus.h>

#include "config.h"
#include "gps_module.h"
#include "huskylens_module.h"
#include "imu_module.h"
#include "motor_control.h"
#include "state.h"

static int lockedAvoidSteer = AVOID_LEFT_STEER;
static bool avoidSteerLocked = false;
static unsigned long lastImuFallbackPrintMs = 0;
static unsigned long yellowLineLostStartMs = 0;
static int lastYellowLineSteer = DEFAULT_SERVO_CENTER;

struct GpsWaypoint {
  double lat;
  double lon;
};

static const GpsWaypoint gpsOvalWaypoints[] = {
  {40.342222, -74.696861},
  {40.342306, -74.696639},
  {40.342444, -74.696556},
  {40.342583, -74.696528},
  {40.342694, -74.696583},
  {40.342694, -74.696722},
  {40.342611, -74.696833},
  {40.342500, -74.696889},
  {40.342389, -74.696944},
  {40.342250, -74.696917}
};

static const int GPS_WAYPOINT_COUNT = sizeof(gpsOvalWaypoints) / sizeof(gpsOvalWaypoints[0]);

static int chooseAvoidSteer() {
  if (frontAngle >= 360.0 - FRONT_HALF_ANGLE) return AVOID_RIGHT_STEER;
  if (frontAngle <= FRONT_HALF_ANGLE) return AVOID_LEFT_STEER;
  return AVOID_LEFT_STEER;
}

static int gpsSteerTowardBearing(float targetBearingDeg, float gpsCourseDeg, float gainScale, int maxTurn) {
  float currentHeadingDeg = imuReadOk ? getYaw() : gpsCourseDeg;
  gpsHeadingError = angleError(targetBearingDeg, currentHeadingDeg);

  int correction = gpsHeadingError * GPS_NAV_STEER_GAIN * gainScale;
  correction = constrain(correction, -maxTurn, maxTurn);
  return servoCenter - correction;
}

static int limitSteerStep(int targetSteer, int maxStep) {
  int delta = targetSteer - currentServoPosition;
  delta = constrain(delta, -maxStep, maxStep);
  return currentServoPosition + delta;
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
    Serial.print(steer);
    Serial.print(" direction=");
    if (steer < servoCenter) Serial.println("LEFT/LOW");
    else if (steer > servoCenter) Serial.println("RIGHT/HIGH");
    else Serial.println("CENTER");
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
    yellowLineLostStartMs = millis();
    lastYellowLineSteer = servoCenter;
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
    if (correction == 0) {
      correction = error > 0 ? YELLOW_LINE_MIN_CORRECTION : -YELLOW_LINE_MIN_CORRECTION;
    } else if (abs(correction) < YELLOW_LINE_MIN_CORRECTION) {
      correction = correction > 0 ? YELLOW_LINE_MIN_CORRECTION : -YELLOW_LINE_MIN_CORRECTION;
    }
    steer = YELLOW_LINE_REVERSE_STEERING ? servoCenter + correction : servoCenter - correction;
    steer = limitSteerStep(steer, YELLOW_LINE_MAX_SERVO_STEP);
    lastError = error;
    lastPidTime = now;

    Serial.print("[YELLOW LINE] errorX=");
    Serial.print(error);
    Serial.print(" correction=");
    Serial.print(correction);
    Serial.print(" steer=");
    Serial.print(steer);
    Serial.print(" direction=");
    if (steer < servoCenter) Serial.println("LEFT/LOW");
    else if (steer > servoCenter) Serial.println("RIGHT/HIGH");
    else Serial.println("CENTER");

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
  if (frontBlocked) {
    mode = AVOID_OBJECT;
    return;
  }

  YellowLineBlock block;
  int error = 0;
  if (readYellowLineBlock(block, error)) {
    resetPidState();
    mode = FOLLOW_COLOR;
    return;
  }

  setSteeringServo(servoCenter);
  stopMotor();
}

static void followGpsOval() {
  GpsLocation location = gps();

  if (!location.hasFix || location.lastFixMs == 0 || location.ageMs > 3000) {
    setSteeringServo(servoCenter);
    setEscSpeed(motorSpeed);
    Serial.println("[GPS NAV] Waiting for a fresh GPS fix, holding selected speed");
    return;
  }

  const GpsWaypoint& waypoint = gpsOvalWaypoints[gpsWaypointIndex];
  targetLat = waypoint.lat;
  targetLon = waypoint.lon;

  gpsDistanceToWaypoint = TinyGPSPlus::distanceBetween(
    location.latitude,
    location.longitude,
    waypoint.lat,
    waypoint.lon
  );
  gpsBearingToWaypoint = TinyGPSPlus::courseTo(
    location.latitude,
    location.longitude,
    waypoint.lat,
    waypoint.lon
  );

  if (gpsDistanceToWaypoint <= GPS_WAYPOINT_REACHED_M) {
    gpsWaypointIndex++;
    if (gpsWaypointIndex >= GPS_WAYPOINT_COUNT) {
      gpsWaypointIndex = 0;
      gpsCompletedLoops++;
    }
    Serial.print("[GPS NAV] Reached waypoint, next=");
    Serial.println(gpsWaypointIndex + 1);
    setEscSpeed(motorSpeed);
    return;
  }

  int steer = gpsSteerTowardBearing(
    gpsBearingToWaypoint,
    location.courseDeg,
    1.0f,
    GPS_NAV_MAX_TURN
  );

  setSteeringServo(steer);
  setEscSpeed(motorSpeed);

  unsigned long now = millis();
  if (now - lastPrintAt >= 500) {
    lastPrintAt = now;
    Serial.print("[GPS NAV] wp=");
    Serial.print(gpsWaypointIndex + 1);
    Serial.print("/");
    Serial.print(GPS_WAYPOINT_COUNT);
    Serial.print(" dist=");
    Serial.print(gpsDistanceToWaypoint, 1);
    Serial.print(" bearing=");
    Serial.print(gpsBearingToWaypoint, 1);
    Serial.print(" course=");
    Serial.print(location.courseDeg, 1);
    Serial.print(" headingSource=");
    Serial.print(imuReadOk ? "IMU" : "GPS");
    Serial.print(" error=");
    Serial.print(gpsHeadingError, 1);
    Serial.print(" steer=");
    Serial.println(currentServoPosition);
  }
}

void startGpsOvalNavigation() {
  GpsLocation location = gps();
  if (location.hasFix) {
    float bestDistance = 1000000.0;
    int bestIndex = 0;
    for (int i = 0; i < GPS_WAYPOINT_COUNT; i++) {
      float distance = TinyGPSPlus::distanceBetween(
        location.latitude,
        location.longitude,
        gpsOvalWaypoints[i].lat,
        gpsOvalWaypoints[i].lon
      );
      if (distance < bestDistance) {
        bestDistance = distance;
        bestIndex = i;
      }
    }
    gpsWaypointIndex = bestIndex;
  } else {
    gpsWaypointIndex = 0;
  }

  gpsDistanceToWaypoint = 0.0;
  gpsBearingToWaypoint = 0.0;
  gpsHeadingError = 0.0;
  emergencyStop = false;
  mode = GPS_NAV;
  resetPidState();
}

void setupRobotBehavior() {
  resetPidState();
  mode = GPS_NAV;
  emergencyStop = true;
  setSteeringServo(servoCenter);
  stopMotor();
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
  } else if (mode == GPS_NAV) {
    followGpsOval();
  } else if (ENABLE_LIDAR && mode == AVOID_OBJECT) {
    avoidObject();
  } else if (mode == CENTER_AFTER_AVOID) {
    centerAfterAvoid();
  } else if (mode == FIND_COLOR) {
    findYellowLine();
  }
#endif
}
