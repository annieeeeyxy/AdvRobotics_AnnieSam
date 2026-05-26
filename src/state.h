#ifndef STATE_H
#define STATE_H

#include <Arduino.h>

enum Mode {
  FOLLOW_COLOR,
  AVOID_OBJECT,
  CENTER_AFTER_AVOID,
  FIND_COLOR,
  GPS_NAV
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
extern int currentEscOutput;
extern bool emergencyStop;

extern float frontDistance;
extern float frontAngle;
extern bool frontBlocked;

extern bool yellowLineVisible;
extern int yellowLineId;
extern int yellowLineXCenter;
extern int yellowLineYCenter;
extern int yellowLineWidth;
extern int yellowLineHeight;
extern int yellowLineArea;
extern int yellowLineErrorX;
extern unsigned long yellowLineLastSeenMs;
extern int yellowLineDeadbandPixels;
extern float yellowLineSteerGain;
extern int yellowLineMaxTurn;

extern bool imuReadOk;
extern float targetYaw;
extern float headingKp;
extern unsigned long lastImuUpdateMs;

extern unsigned long lastSerialReport;
extern unsigned long lastPrintAt;
extern unsigned long lastHeadingDebugAt;
extern unsigned long centerAfterAvoidStart;

extern int gpsWaypointIndex;
extern float gpsDistanceToWaypoint;
extern float gpsBearingToWaypoint;
extern float gpsHeadingError;
extern unsigned long gpsCompletedLoops;

const char* modeName();
void resetPidState();
void loadSavedSettings();
void saveCurrentSettings();

#endif
