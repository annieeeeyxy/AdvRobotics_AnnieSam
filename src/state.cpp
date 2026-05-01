#include "state.h"

#include "config.h"

Mode mode = FOLLOW_LINE;

float kp = DEFAULT_KP;
float ki = DEFAULT_KI;
float kd = DEFAULT_KD;
float errorSum = 0.0;
float lastError = 0.0;
unsigned long lastPidTime = 0;

int servoCenter = DEFAULT_SERVO_CENTER;
int currentServoPosition = DEFAULT_SERVO_CENTER;
int imageCenter = DEFAULT_IMAGE_CENTER;
int motorSpeed = DEFAULT_MOTOR_SPEED;
bool emergencyStop = false;

float frontDistance = 10000.0;
float frontAngle = -1.0;
bool frontBlocked = false;

bool imuReadOk = false;
float targetYaw = 0.0;
float headingKp = DEFAULT_HEADING_KP;
unsigned long lastImuUpdateMs = 0;

unsigned long lastSerialReport = 0;
unsigned long lastPrintAt = 0;
unsigned long lastHeadingDebugAt = 0;
unsigned long centerAfterAvoidStart = 0;

const char* modeName() {
  if (mode == FOLLOW_LINE) return "FOLLOW_LINE";
  if (mode == AVOID_OBJECT) return "AVOID_OBJECT";
  if (mode == CENTER_AFTER_AVOID) return "CENTER_AFTER_AVOID";
  if (mode == FIND_LINE) return "FIND_LINE";
  return "UNKNOWN";
}

void resetPidState() {
  errorSum = 0.0;
  lastError = 0.0;
  lastPidTime = millis();
}
