#include "state.h"

#include "config.h"

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
bool emergencyStop = true;

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
int yellowLineMaxTurn = 25;

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
