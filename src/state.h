#ifndef STATE_H
#define STATE_H

#include <Arduino.h>

enum Mode {
  FOLLOW_LINE,
  AVOID_OBJECT,
  CENTER_AFTER_AVOID,
  FIND_LINE
};

extern Mode mode;

extern float kp;
extern float ki;
extern float kd;
extern float errorSum;
extern float lastError;
extern unsigned long lastPidTime;

extern int servoCenter;
extern int currentServoPosition;
extern int imageCenter;
extern int motorSpeed;
extern bool emergencyStop;

extern float frontDistance;
extern float frontAngle;
extern bool frontBlocked;

extern bool imuReadOk;
extern float targetYaw;
extern float headingKp;
extern unsigned long lastImuUpdateMs;

extern unsigned long lastSerialReport;
extern unsigned long lastPrintAt;
extern unsigned long lastHeadingDebugAt;
extern unsigned long centerAfterAvoidStart;

const char* modeName();
void resetPidState();

#endif
