#include "huskylens_module.h"

#include <Arduino.h>
#include <Wire.h>
#include "HUSKYLENS.h"

#include "config.h"
#include "state.h"

static HUSKYLENS husky;

void setupHuskylens() {
#if ENABLE_HUSKYLENS
  while (!husky.begin(Wire)) {
    Serial.println("HUSKYLENS not found");
    delay(100);
  }
  husky.writeAlgorithm(ALGORITHM_LINE_TRACKING);
#endif
}

bool readLineError(float& error) {
#if ENABLE_HUSKYLENS
  if (!husky.request()) return false;
  if (!husky.available()) return false;

  HUSKYLENSResult result = husky.read();
  if (result.command != COMMAND_RETURN_ARROW) return false;

  error = result.xTarget - imageCenter;
  return true;
#else
  error = 0.0;
  return false;
#endif
}
