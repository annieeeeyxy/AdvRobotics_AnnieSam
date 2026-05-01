#include <Arduino.h>

#include "config.h"
#include "huskylens_module.h"
#include "imu_module.h"
#include "lidar_module.h"
#include "motor_control.h"
#include "robot_behavior.h"
#include "web_dashboard.h"

#if !ENABLE_IMU_STANDALONE_TEST

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("Booting Nimbus firmware");

  setupImu();

  if (ENABLE_WEB) {
    setupWebDashboard();
  }

  setupMotorControl();

  if (ENABLE_HUSKYLENS) {
    setupHuskylens();
  }

  if (ENABLE_LIDAR) {
    setupLidar();
  }

  setupRobotBehavior();

  Serial.println("Ready");
  Serial.println("WiFi: " WIFI_SSID);
  Serial.println("Open: http://192.168.3.1");
}

void loop() {
  updateImuCached();
  serviceWebDashboard();
  updateFrontLidar();
  updateRobotBehavior();
}

#endif
