#include "gps_module.h"

#include <TinyGPSPlus.h>

#include "config.h"

/*
  GPS wiring for Arduino GIGA R1:
  GPS VCC -> 3.3V or 5V depending on module
  GPS GND -> GIGA GND
  GPS TX  -> GIGA pin 17 / Serial3 RX
  GPS RX  -> optional / not connected

  Navigation notes for later:
  GPS gives the robot's current position.
  The target bearing gives the direction from the robot to the goal.
  A compass/IMU should be used for the current robot heading.
  heading error = targetBearing - currentHeading
  Later, PID can use this error to control the steering servo.
*/

double targetLat = 0.0;
double targetLon = 0.0;

static TinyGPSPlus gpsParser;
static GpsLocation latestGps = {
  false,
  false,
  0.0,
  0.0,
  0,
  0.0,
  false,
  0.0,
  0.0,
  0.0,
  0.0,
  0,
  0,
  0,
  0,
  0,
  GPS_BAUD
};

static unsigned long lastGpsPrintMs = 0;

static void updateStats() {
  latestGps.charsProcessed = gpsParser.charsProcessed();
  latestGps.hasReceivedData = latestGps.charsProcessed > 0;
  latestGps.sentencesWithFix = gpsParser.sentencesWithFix();
  latestGps.checksumFailures = gpsParser.failedChecksum();

  if (gpsParser.satellites.isValid()) {
    latestGps.satellites = gpsParser.satellites.value();
  }

  if (gpsParser.hdop.isValid()) {
    latestGps.hdop = gpsParser.hdop.hdop();
    latestGps.hdopValid = true;
  }

  if (gpsParser.speed.isValid()) {
    latestGps.speedKmph = gpsParser.speed.kmph();
  }

  if (gpsParser.course.isValid()) {
    latestGps.courseDeg = gpsParser.course.deg();
  }
}

static void updateTargetCalculation() {
  if (!latestGps.hasFix) return;

  latestGps.distanceToTargetMeters = TinyGPSPlus::distanceBetween(
    latestGps.latitude,
    latestGps.longitude,
    targetLat,
    targetLon
  );
  latestGps.bearingToTargetDeg = TinyGPSPlus::courseTo(
    latestGps.latitude,
    latestGps.longitude,
    targetLat,
    targetLon
  );
}

static void printGpsReport() {
  Serial.println();
  Serial.println("[GPS] ----- GPS report -----");

  Serial.print("[GPS] Data received: ");
  Serial.println(latestGps.hasReceivedData ? "YES" : "NO");
  Serial.print("[GPS] Characters processed: ");
  Serial.println(latestGps.charsProcessed);

  if (!latestGps.hasReceivedData) {
    Serial.println("[GPS] No GPS data received. Check wiring or baud rate.");
  }

  Serial.print("[GPS] Satellites: ");
  Serial.println(latestGps.satellites);

  if (latestGps.hasFix) {
    Serial.print("[GPS] Latitude: ");
    Serial.println(latestGps.latitude, 6);
    Serial.print("[GPS] Longitude: ");
    Serial.println(latestGps.longitude, 6);
    Serial.print("[GPS] Speed km/h: ");
    Serial.println(latestGps.speedKmph, 2);
    Serial.print("[GPS] Course deg: ");
    Serial.println(latestGps.courseDeg, 2);
    Serial.print("[GPS] HDOP: ");
    if (latestGps.hdopValid) Serial.println(latestGps.hdop, 2);
    else Serial.println("not available");
  } else {
    Serial.println("[GPS] Location not valid yet. Move GPS near a window or outside.");
    Serial.print("[GPS] HDOP: ");
    if (latestGps.hdopValid) Serial.println(latestGps.hdop, 2);
    else Serial.println("not available");
  }
}

void setupGps() {
#if ENABLE_GPS
  latestGps.baud = GPS_BAUD;
  GPS_SERIAL_PORT.begin(GPS_BAUD);
  Serial.print("[GPS] Reading NMEA from Serial3 at ");
  Serial.print(GPS_BAUD);
  Serial.println(" baud");
  Serial.println("[GPS] GPS TX should be connected to GIGA pin 17 / Serial3 RX. GPS RX can be left unconnected.");
#endif
}

void serviceGps() {
#if ENABLE_GPS
  while (GPS_SERIAL_PORT.available()) {
    char c = GPS_SERIAL_PORT.read();
    gpsParser.encode(c);
  }

  updateStats();

  if (gpsParser.location.isValid()) {
    latestGps.latitude = gpsParser.location.lat();
    latestGps.longitude = gpsParser.location.lng();
    latestGps.hasFix = true;
    latestGps.ageMs = gpsParser.location.age();
    if (gpsParser.location.isUpdated()) {
      latestGps.lastFixMs = millis();
    }
  } else {
    latestGps.hasFix = false;
  }

  updateTargetCalculation();

  if (millis() - lastGpsPrintMs >= 1000) {
    lastGpsPrintMs = millis();
    printGpsReport();
  }
#endif
}

GpsLocation gps() {
  GpsLocation snapshot = latestGps;

#if ENABLE_GPS
  if (snapshot.lastFixMs == 0) {
    snapshot.ageMs = 0;
  } else {
    snapshot.ageMs = millis() - snapshot.lastFixMs;
  }
#endif

  return snapshot;
}
