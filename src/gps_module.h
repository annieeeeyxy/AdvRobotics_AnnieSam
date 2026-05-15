#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>

struct GpsLocation {
  bool hasFix;
  double latitude;
  double longitude;
  uint32_t satellites;
  double hdop;
  unsigned long ageMs;
  unsigned long lastFixMs;
  unsigned long charsProcessed;
  unsigned long sentencesWithFix;
  unsigned long checksumFailures;
  unsigned long baud;
};

void setupGps();
void serviceGps();
GpsLocation gps();

#endif
