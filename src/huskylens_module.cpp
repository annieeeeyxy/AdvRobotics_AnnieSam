#include "huskylens_module.h"

#include <Arduino.h>
#include <Wire.h>
#include "HUSKYLENS.h"

#include "config.h"
#include "state.h"

static HUSKYLENS husky;
static bool huskyReady = false;
static unsigned long lastHuskyRetryMs = 0;

static bool ensureHuskylensReady() {
#if ENABLE_HUSKYLENS
  if (huskyReady) {
    return true;
  }

  unsigned long now = millis();
  if (lastHuskyRetryMs != 0 && now - lastHuskyRetryMs < 1000) {
    return false;
  }
  lastHuskyRetryMs = now;

  // HUSKYLENS is wired to pins 20/21, which are Wire1 on the Arduino GIGA.
  // The IMU also uses Wire1, so both devices share the same I2C bus.
  Wire1.begin();
  if (!husky.begin(Wire1)) {
    Serial.println("[HUSKYLENS] Not found on Wire1 pins 20/21. Check I2C wiring and HUSKYLENS protocol setting.");
    return false;
  }

  huskyReady = true;
  if (husky.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION)) {
    Serial.println("[HUSKYLENS] Algorithm set to Color Recognition for yellow line tracking");
  } else {
    Serial.println("[HUSKYLENS] Could not set Color Recognition algorithm");
  }
  return true;
#else
  return false;
#endif
}

void setupHuskylens() {
#if ENABLE_HUSKYLENS
  Serial.println("[HUSKYLENS] Starting yellow line tracking setup");
  Serial.print("[HUSKYLENS] Using Wire1 SDA=D");
  Serial.print(HUSKYLENS_I2C_SDA_PIN);
  Serial.print(" SCL=D");
  Serial.println(HUSKYLENS_I2C_SCL_PIN);
  Serial.println("[HUSKYLENS] On the screen: set RGB Brightness=-1, RGB Gain Lock=ON, Color Recognition mode, then Save and Exit.");
  Serial.println("[HUSKYLENS] Clear old learned colors, aim at the yellow road line, then learn yellow with the HUSKYLENS button.");

  ensureHuskylensReady();

  // RGB Brightness, RGB Gain Lock, clearing old learned colors, and learning the
  // yellow road line must be done manually on the HUSKYLENS screen with this library.
#endif
}

bool readYellowLineBlock(YellowLineBlock& block, int& error) {
#if ENABLE_HUSKYLENS
  int targetX = YELLOW_LINE_TARGET_X;

  if (!ensureHuskylensReady()) {
    yellowLineVisible = false;
    return false;
  }

  // Request one fresh frame from HUSKYLENS. We filter the returned results below
  // so ARROW results are ignored and only BLOCK results can be selected.
  if (!husky.request()) {
    Serial.println("[HUSKYLENS] Request failed");
    yellowLineVisible = false;
    return false;
  }

  if (!husky.available()) {
    Serial.println("Yellow line lost");
    yellowLineVisible = false;
    return false;
  }

  bool foundBlock = false;
  YellowLineBlock bestBlock = {0, 0, 0, 0, 0, 0};

  while (husky.available()) {
    HUSKYLENSResult result = husky.read();
    if (result.command != COMMAND_RETURN_BLOCK) {
      continue;
    }
    if (result.ID != YELLOW_LINE_ID) {
      continue;
    }

    int area = result.width * result.height;
    if (!foundBlock || area > bestBlock.area) {
      bestBlock.xCenter = result.xCenter;
      bestBlock.yCenter = result.yCenter;
      bestBlock.width = result.width;
      bestBlock.height = result.height;
      bestBlock.id = result.ID;
      bestBlock.area = area;
      foundBlock = true;
    }
  }

  if (!foundBlock) {
    Serial.println("Yellow line lost");
    yellowLineVisible = false;
    return false;
  }

  block = bestBlock;

  error = block.xCenter - targetX;
  yellowLineVisible = true;
  yellowLineId = block.id;
  yellowLineXCenter = block.xCenter;
  yellowLineYCenter = block.yCenter;
  yellowLineWidth = block.width;
  yellowLineHeight = block.height;
  yellowLineArea = block.area;
  yellowLineErrorX = error;
  yellowLineLastSeenMs = millis();

  return true;
#else
  error = 0;
  yellowLineVisible = false;
  block.xCenter = 0;
  block.yCenter = 0;
  block.width = 0;
  block.height = 0;
  block.id = 0;
  block.area = 0;
  return false;
#endif
}

void serviceHuskylensColorTest() {
#if ENABLE_HUSKYLENS
  if (ENABLE_ROBOT_BEHAVIOR && !emergencyStop) {
    return;
  }

  YellowLineBlock block;
  int errorX = 0;

  // Test only: report the largest learned yellow color block. No driving logic
  // belongs here, so this can be used safely while tuning HUSKYLENS color mode.
  if (!readYellowLineBlock(block, errorX)) {
    return;
  }

  Serial.print("Yellow line visible: yes");
  Serial.print(" ID=");
  Serial.print(block.id);
  Serial.print(" xCenter=");
  Serial.print(block.xCenter);
  Serial.print(" yCenter=");
  Serial.print(block.yCenter);
  Serial.print(" width=");
  Serial.print(block.width);
  Serial.print(" height=");
  Serial.print(block.height);
  Serial.print(" area=");
  Serial.print(block.area);
  Serial.print(" errorX=");
  Serial.println(errorX);
#endif
}
