#include "state.h"

#include "config.h"
#include "KVStore.h"
#include "kvstore_global_api.h"

struct SavedSettings {
  uint32_t magic;
  float savedKp;
  float savedKi;
  float savedKd;
  int savedMotorSpeed;
  int savedDeadbandPixels;
  int savedMaxTurn;
};

static const char* SETTINGS_KEY = "line_settings";
static const uint32_t SETTINGS_MAGIC = 0x4C494E45; // "LINE"

Mode mode = FOLLOW_COLOR;

float kp = 0.5f;
float ki = 0.0f;
float kd = 0.1f;
float errorSum = 0.0;
float lastError = 0.0;
unsigned long lastPidTime = 0;

int servoCenter = DEFAULT_SERVO_CENTER;
int currentServoPosition = DEFAULT_SERVO_CENTER;
int imageCenter = DEFAULT_IMAGE_CENTER;
int motorSpeed = 100;
bool emergencyStop = !START_LINE_TRACKING_ON_BOOT;

float frontDistance = 10000.0;
float frontAngle = -1.0;
bool frontBlocked = false;

bool yellowLineVisible = false;
int yellowLineId = 0;
int yellowLineXCenter = 0;
int yellowLineYCenter = 0;
int yellowLineWidth = 0;
int yellowLineHeight = 0;
int yellowLineArea = 0;
int yellowLineErrorX = 0;
unsigned long yellowLineLastSeenMs = 0;
int yellowLineDeadbandPixels = 10;
float yellowLineSteerGain = DEFAULT_YELLOW_LINE_STEER_GAIN;
int yellowLineMaxTurn = 30;

bool imuReadOk = false;
float targetYaw = 0.0;
float headingKp = DEFAULT_HEADING_KP;
unsigned long lastImuUpdateMs = 0;

unsigned long lastSerialReport = 0;
unsigned long lastPrintAt = 0;
unsigned long lastHeadingDebugAt = 0;
unsigned long centerAfterAvoidStart = 0;

const char* modeName() {
  if (mode == FOLLOW_COLOR) return "FOLLOW_COLOR";
  if (mode == AVOID_OBJECT) return "AVOID_OBJECT";
  if (mode == CENTER_AFTER_AVOID) return "CENTER_AFTER_AVOID";
  if (mode == FIND_COLOR) return "FIND_COLOR";
  return "UNKNOWN";
}

void resetPidState() {
  errorSum = 0.0;
  lastError = 0.0;
  lastPidTime = millis();
}

void loadSavedSettings() {
  SavedSettings saved;
  size_t actualSize = 0;

  int result = kv_get(SETTINGS_KEY, &saved, sizeof(saved), &actualSize);
  if (result != 0 || actualSize != sizeof(saved) || saved.magic != SETTINGS_MAGIC) {
    Serial.println("[SETTINGS] No saved line settings found, using defaults");
    return;
  }

  kp = saved.savedKp;
  ki = saved.savedKi;
  kd = saved.savedKd;
  motorSpeed = constrain(saved.savedMotorSpeed, 0, 180);
  yellowLineDeadbandPixels = constrain(saved.savedDeadbandPixels, 0, 60);
  yellowLineMaxTurn = constrain(saved.savedMaxTurn, 0, 35);

  Serial.println("[SETTINGS] Loaded saved line settings");
  Serial.print("[SETTINGS] speed=");
  Serial.println(motorSpeed);
  Serial.print("[SETTINGS] kp=");
  Serial.println(kp, 3);
}

void saveCurrentSettings() {
  SavedSettings saved = {
    SETTINGS_MAGIC,
    kp,
    ki,
    kd,
    motorSpeed,
    yellowLineDeadbandPixels,
    yellowLineMaxTurn
  };

  int result = kv_set(SETTINGS_KEY, &saved, sizeof(saved), 0);
  if (result == 0) {
    Serial.println("[SETTINGS] Saved line settings");
  } else {
    Serial.print("[SETTINGS] Save failed, code=");
    Serial.println(result);
  }
}
