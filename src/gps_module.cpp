#include "gps_module.h"

#include <TinyGPSPlus.h>

#include "config.h"

static TinyGPSPlus gpsParser;
static GpsLocation latestGps = {
  false,
  0.0,
  0.0,
  0,
  0.0,
  0,
  0,
  0,
  0,
  0,
  GPS_PRIMARY_BAUD
};

static unsigned long currentBaud = GPS_PRIMARY_BAUD;
static unsigned long baudStartedAtMs = 0;

static void beginGpsSerial(unsigned long baud) {
  currentBaud = baud;
  latestGps.baud = baud;
  baudStartedAtMs = millis();
  GPS_SERIAL_PORT.begin(baud);

  Serial.print("[GPS] Serial2 started at ");
  Serial.print(baud);
  Serial.println(" baud");
}

static void updateStats() {
  latestGps.charsProcessed = gpsParser.charsProcessed();
  latestGps.sentencesWithFix = gpsParser.sentencesWithFix();
  latestGps.checksumFailures = gpsParser.failedChecksum();

  if (gpsParser.satellites.isValid()) {
    latestGps.satellites = gpsParser.satellites.value();
  }

  if (gpsParser.hdop.isValid()) {
    latestGps.hdop = gpsParser.hdop.hdop();
  }
}

void setupGps() {
#if ENABLE_GPS
  beginGpsSerial(GPS_PRIMARY_BAUD);
  Serial.println("[GPS] Reading NMEA on Serial2 RX/TX");
#endif
}

void serviceGps() {
#if ENABLE_GPS
  while (GPS_SERIAL_PORT.available()) {
    char c = GPS_SERIAL_PORT.read();
    gpsParser.encode(c);
  }

  updateStats();

  if (gpsParser.location.isUpdated()) {
    latestGps.latitude = gpsParser.location.lat();
    latestGps.longitude = gpsParser.location.lng();
    latestGps.lastFixMs = millis();
    latestGps.hasFix = gpsParser.location.isValid();
    latestGps.ageMs = gpsParser.location.age();

    Serial.print("[GPS] LAT=");
    Serial.print(latestGps.latitude, 6);
    Serial.print(" LON=");
    Serial.print(latestGps.longitude, 6);
    Serial.print(" SAT=");
    Serial.print(latestGps.satellites);
    Serial.print(" HDOP=");
    Serial.println(latestGps.hdop, 2);
  }

  if (latestGps.charsProcessed == 0 && millis() - baudStartedAtMs > GPS_BAUD_SWITCH_MS) {
    unsigned long nextBaud = (currentBaud == GPS_PRIMARY_BAUD) ? GPS_FALLBACK_BAUD : GPS_PRIMARY_BAUD;
    beginGpsSerial(nextBaud);
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
