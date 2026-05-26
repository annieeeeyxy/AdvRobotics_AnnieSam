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
static bool imuFullReadOk = false;

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
  Serial.print("Scanning IMU bus: Wire1 SDA=D");
  Serial.print(IMU_I2C_SDA_PIN);
  Serial.print(" SCL=D");
  Serial.println(IMU_I2C_SCL_PIN);
  int found = 0;
  bool imuFound = false;

  for (uint8_t address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    uint8_t status = Wire1.endTransmission();
    if (status == 0) {
      Serial.print("Found device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      if (address == IMU_I2C_ADDRESS) {
        imuFound = true;
      }
      found++;
    } else if (address == IMU_I2C_ADDRESS) {
      Serial.print("IMU address 0x");
      if (IMU_I2C_ADDRESS < 16) Serial.print("0");
      Serial.print(IMU_I2C_ADDRESS, HEX);
      Serial.print(" did not ACK, status=");
      Serial.println(status);
    }
  }

  if (imuFound) {
    Serial.print("IMU 0x");
    if (IMU_I2C_ADDRESS < 16) Serial.print("0");
    Serial.print(IMU_I2C_ADDRESS, HEX);
    Serial.println(" found");
  }

  if (found == 0) {
    Serial.println("No I2C devices found");
  }
}

void setupImu() {
#if ENABLE_IMU
  Serial.println("IMU init start");
  Serial.println("Arduino GIGA IMU bus: using Wire1 for D20(SDA)/D21(SCL). Use Wire only if the IMU is wired to the board's default SDA/SCL pins.");
  Serial.print("Expected IMU I2C address: 0x");
  if (IMU_I2C_ADDRESS < 16) Serial.print("0");
  Serial.println(IMU_I2C_ADDRESS, HEX);
  Wire1.begin();
  IIC_Init();
  scanI2cBus();

  int versionResult = IMU_I2C_ReadVersion();
  Serial.print("IMU version result: ");
  Serial.println(versionResult);

  int coreReadResult = IMU_I2C_ReadEulerAndAccelerometer(&imuData);
  Serial.print("IMU accel+Euler read result: ");
  Serial.println(coreReadResult);

  int firstReadResult = IMU_I2C_ReadAll(&imuData);
  Serial.print("IMU first read result: ");
  Serial.println(firstReadResult);
  imuFullReadOk = (firstReadResult == 0);

  if (firstReadResult == 0 || coreReadResult == 0) {
    imuReadOk = true;
    cacheImuData();
    targetYaw = getYaw();
    Serial.println(firstReadResult == 0 ? "IMU ready, full read works" : "IMU ready, accel+Euler works; full read failed in optional data");
    print_sensor_data(imuData);
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
  imuFullReadOk = (result == 0);

  if (result != 0) {
    result = IMU_I2C_ReadEulerAndAccelerometer(&imuData);
  }

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
    imuFullReadOk = (result == 0);

    if (result != 0) {
      result = IMU_I2C_ReadEulerAndAccelerometer(&imuData);
    }

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
      if (imuFullReadOk) {
        print_sensor_data(imuData);
      } else {
        Serial.print("[IMU] accel[g] x=");
        Serial.print(imuData.accel[0], 3);
        Serial.print(" y=");
        Serial.print(imuData.accel[1], 3);
        Serial.print(" z=");
        Serial.print(imuData.accel[2], 3);
        Serial.print(" | Euler[deg] roll=");
        Serial.print(imuData.euler[0], 3);
        Serial.print(" pitch=");
        Serial.print(imuData.euler[1], 3);
        Serial.print(" yaw=");
        Serial.println(imuData.euler[2], 3);
      }
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
