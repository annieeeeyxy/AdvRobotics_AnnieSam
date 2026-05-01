#include "imu_module.h"

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "imu_i2c_driver.hpp"
#include "state.h"

static imu_measurement_t imuData;
static float cachedYaw = 0.0;
static float cachedPitch = 0.0;
static float cachedRoll = 0.0;
static unsigned long lastImuFailurePrintMs = 0;

static void cacheImuData() {
  cachedRoll = imuData.euler[0];
  cachedPitch = imuData.euler[1];
  cachedYaw = imuData.euler[2];
}

static void clearCachedImuData() {
  cachedRoll = 0.0;
  cachedPitch = 0.0;
  cachedYaw = 0.0;
}

static void printImuFailureThrottled(int result) {
  unsigned long now = millis();
  if (now - lastImuFailurePrintMs < 500) {
    return;
  }

  lastImuFailurePrintMs = now;
  Serial.print("[IMU] read failed, code=");
  Serial.println(result);
}

static void scanI2cBus() {
  Serial.println("Scanning I2C bus...");
  int found = 0;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("No I2C devices found");
  }
}

void setupImu() {
#if ENABLE_IMU
  Serial.println("IMU init start");
  IIC_Init();
  scanI2cBus();

  int versionResult = IMU_I2C_ReadVersion();
  Serial.print("IMU version result: ");
  Serial.println(versionResult);

  int firstReadResult = IMU_I2C_ReadAll(&imuData);
  Serial.print("IMU first read result: ");
  Serial.println(firstReadResult);

  if (firstReadResult == 0) {
    imuReadOk = true;
    cacheImuData();
    targetYaw = getYaw();
    Serial.println("IMU ready");
  } else {
    imuReadOk = false;
    clearCachedImuData();
    Serial.println("IMU not ready");
  }
#else
  imuReadOk = false;
  clearCachedImuData();
#endif
}

bool readImu() {
#if ENABLE_IMU
  int result = IMU_I2C_ReadAll(&imuData);

  if (result == 0) {
    imuReadOk = true;
    cacheImuData();
    return true;
  }

  imuReadOk = false;
  clearCachedImuData();
  printImuFailureThrottled(result);
  return false;
#else
  imuReadOk = false;
  clearCachedImuData();
  return false;
#endif
}

void updateImuCached() {
#if ENABLE_IMU
  unsigned long now = millis();

  if (now - lastImuUpdateMs >= IMU_UPDATE_INTERVAL_MS) {
    lastImuUpdateMs = now;
    int result = IMU_I2C_ReadAll(&imuData);

    if (result == 0) {
      imuReadOk = true;
      cacheImuData();
    } else {
      imuReadOk = false;
      clearCachedImuData();
      printImuFailureThrottled(result);
    }
  }

  if (!ENABLE_ROBOT_BEHAVIOR && now - lastPrintAt >= IMU_PRINT_INTERVAL_MS) {
    lastPrintAt = now;
    if (imuReadOk) {
      print_sensor_data(imuData);
    }
  }
#endif
}

bool isImuReady() {
  return imuReadOk;
}

float getYaw() {
  return cachedYaw;
}

float getPitch() {
  return cachedPitch;
}

float getRoll() {
  return cachedRoll;
}

float angleError(float target, float current) {
  float error = target - current;

  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;

  return error;
}
