#include <Arduino.h>

#include "config.h"
#include "gps_module.h"
#include "huskylens_module.h"
#include "imu_module.h"
#include "lidar_module.h"
#include "motor_control.h"
#include "robot_behavior.h"
#include "web_dashboard.h"

#if !ENABLE_IMU_STANDALONE_TEST

void setup() {
  Serial.begin(115200);
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 5000) {
    delay(10);
  }

  Serial.println();
  Serial.println("Booting GPS web monitor firmware");
  setupGps();
  setupHuskylens();
  setupLidar();
  setupImu();
  setupMotorControl();
  setupRobotBehavior();
  setupWebDashboard();
  Serial.println("Ready. Open PlatformIO monitor at 115200 baud.");
  Serial.println("WiFi: " WIFI_SSID);
  Serial.println("Open: http://192.168.3.1");
}

void loop() {
  serviceGps();
  serviceHuskylensColorTest();
  updateFrontLidar();
  updateImuCached();
  serviceWebDashboard();
  updateRobotBehavior();
}

#endif
