#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>

struct GpsLocation {
  bool hasFix;
  bool hasReceivedData;
  double latitude;
  double longitude;
  uint32_t satellites;
  double hdop;
  bool hdopValid;
  double speedKmph;
  double courseDeg;
  double distanceToTargetMeters;
  double bearingToTargetDeg;
  unsigned long ageMs;
  unsigned long lastFixMs;
  unsigned long charsProcessed;
  unsigned long sentencesWithFix;
  unsigned long checksumFailures;
  unsigned long baud;
};

extern double targetLat;
extern double targetLon;

void setupGps();
void serviceGps();
GpsLocation gps();

#endif
