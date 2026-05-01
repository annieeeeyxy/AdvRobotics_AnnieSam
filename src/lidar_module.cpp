#include "lidar_module.h"

#include <Arduino.h>
#include <RPLidar.h>

#include "config.h"
#include "state.h"

static RPLidar lidar;
static int blockedScanCount = 0;
static int clearScanCount = 0;

static void restartLidar() {
#if ENABLE_LIDAR
  analogWrite(LIDAR_MOTOR_PIN, 0);
  delay(50);

  rplidar_response_device_info_t info;
  if (IS_OK(lidar.getDeviceInfo(info, 100))) {
    analogWrite(LIDAR_MOTOR_PIN, 255);
    delay(1000);
    lidar.startScan();
    Serial.println("LiDAR restarted");
  } else {
    Serial.println("LiDAR not found");
  }
#endif
}

void setupLidar() {
#if ENABLE_LIDAR
  blockedScanCount = 0;
  clearScanCount = 0;
  Serial2.begin(115200);
  lidar.begin(Serial2);
  pinMode(LIDAR_MOTOR_PIN, OUTPUT);
  analogWrite(LIDAR_MOTOR_PIN, 255);
  delay(1000);
  lidar.startScan();
#else
  frontDistance = 10000.0;
  frontAngle = -1.0;
  frontBlocked = false;
#endif
}

void updateFrontLidar() {
#if ENABLE_LIDAR
  float scanFrontDistance = 10000.0;
  float scanFrontAngle = -1.0;

  unsigned long start = millis();

  while (millis() - start < 100) {
    if (!IS_OK(lidar.waitPoint())) {
      Serial.println("LiDAR disconnected");
      restartLidar();
      return;
    }

    float d = lidar.getCurrentPoint().distance;
    float a = lidar.getCurrentPoint().angle;
    float q = lidar.getCurrentPoint().quality;

    if (q <= 10 || d <= 0) continue;

    bool inFront = (a >= 360.0 - FRONT_HALF_ANGLE || a <= FRONT_HALF_ANGLE);

    if (inFront && d < scanFrontDistance) {
      scanFrontDistance = d;
      scanFrontAngle = a;
    }
  }

  frontDistance = scanFrontDistance;
  frontAngle = scanFrontAngle;

  if (scanFrontDistance < FRONT_STOP_DIST) {
    blockedScanCount++;
    clearScanCount = 0;
  } else if (scanFrontDistance > FRONT_CLEAR_DIST) {
    clearScanCount++;
    blockedScanCount = 0;
  } else {
    blockedScanCount = 0;
    clearScanCount = 0;
  }

  if (!frontBlocked && blockedScanCount >= LIDAR_BLOCK_CONFIRM_SCANS) {
    frontBlocked = true;
  } else if (frontBlocked && clearScanCount >= LIDAR_CLEAR_CONFIRM_SCANS) {
    frontBlocked = false;
  }

  unsigned long now = millis();
  if (now - lastSerialReport < SERIAL_REPORT_INTERVAL) return;
  lastSerialReport = now;

  Serial.print("Front distance: ");
  if (frontDistance < 9999) Serial.print(frontDistance);
  else Serial.print("none");
  Serial.print(" mm | angle: ");
  if (frontAngle >= 0) Serial.print(frontAngle);
  else Serial.print("none");
  Serial.print(" | blocked: ");
  Serial.print(frontBlocked ? "YES" : "NO");
  Serial.print(" | servo: ");
  Serial.print(currentServoPosition);
  Serial.print(" | mode: ");
  Serial.println(modeName());
#else
  frontDistance = 10000.0;
  frontAngle = -1.0;
  frontBlocked = false;
#endif
}
