#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "imu_i2c_driver.hpp"

#if ENABLE_IMU_STANDALONE_TEST

static const unsigned long PRINT_INTERVAL_MS = 250;

static imu_measurement_t imu_data;
static unsigned long lastPrintMs = 0;

static bool scanI2cBus() {
  bool foundAnyDevice = false;

  Serial.println("I2C scan start");

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      foundAnyDevice = true;
      Serial.print("Found device at 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (!foundAnyDevice) {
    Serial.println("NO I2C DEVICE FOUND ON D20/D21");
  }

  return foundAnyDevice;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("Testing Arduino Giga default I2C bus: D20/SDA, D21/SCL");

  Wire.begin();
  scanI2cBus();

  IIC_Init();

  int versionResult = IMU_I2C_ReadVersion();
  Serial.print("IMU_I2C_ReadVersion result: ");
  Serial.println(versionResult);

  int readAllResult = IMU_I2C_ReadAll(&imu_data);
  Serial.print("IMU_I2C_ReadAll result: ");
  Serial.println(readAllResult);

  if (readAllResult == 0) {
    Serial.print("yaw=");
    Serial.print(imu_data.euler[2], 3);
    Serial.print(" pitch=");
    Serial.print(imu_data.euler[1], 3);
    Serial.print(" roll=");
    Serial.println(imu_data.euler[0], 3);
  }
}

void loop() {
  int readAllResult = IMU_I2C_ReadAll(&imu_data);

  unsigned long now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintMs = now;

  Serial.print("IMU_I2C_ReadAll result: ");
  Serial.println(readAllResult);

  if (readAllResult == 0) {
    Serial.print("yaw=");
    Serial.print(imu_data.euler[2], 3);
    Serial.print(" pitch=");
    Serial.print(imu_data.euler[1], 3);
    Serial.print(" roll=");
    Serial.println(imu_data.euler[0], 3);
  }
}

#endif
