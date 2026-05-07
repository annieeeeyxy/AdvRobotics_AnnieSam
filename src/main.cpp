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


  if(ENABLE_IMU){
    setupImu();
  }
  if (ENABLE_WEB) {
    setupWebDashboard();
  }

  if (ENABLE_WEB || ENABLE_ROBOT_BEHAVIOR) {
    setupMotorControl();
  }

  if (ENABLE_HUSKYLENS) {
    setupHuskylens();
  }

  if (ENABLE_LIDAR) {
    setupLidar();
  }

  setupRobotBehavior();

  Serial.println("Ready");
  Serial.println("Robot starts stopped. Open the web dashboard and press Enable Line Following to drive.");
  Serial.println("WiFi: " WIFI_SSID);
  Serial.println("Open: http://192.168.3.1");
}

void loop() {
  updateImuCached();
  serviceWebDashboard();
  updateFrontLidar();
  serviceHuskylensColorTest();
  updateRobotBehavior();
}

#endif
