#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/************ Compile-time module switches ************/
#define ENABLE_WEB 1
#define ENABLE_IMU 0
#define ENABLE_LIDAR 0
#define ENABLE_HUSKYLENS 1
#define ENABLE_ROBOT_BEHAVIOR 1

/************ WiFi ************/
#define WIFI_SSID "GIGA_PID"
#define WIFI_PASS "12345678"
#define WEB_PORT 80

/************ Pins ************/
const int SERVO_PIN = 7;
const int ESC_PIN = 6;
const int LIDAR_MOTOR_PIN = 5;

/************ I2C / IMU ************/
#define IMU_I2C_ADDRESS 0x23           // address shown by I2C scanner
const int IMU_I2C_SDA_PIN = 20;        // Arduino GIGA Wire1 SDA
const int IMU_I2C_SCL_PIN = 21;        // Arduino GIGA Wire1 SCL

/************ I2C / HUSKYLENS ************/
// HUSKYLENS uses the default Wire bus, matching the working DFRobot example.

/************ Robot settings ************/
const int DEFAULT_SERVO_CENTER = 90;
const int DEFAULT_IMAGE_CENTER = 160;
const int ESC_NEUTRAL = 90;
const int SERVO_MIN_ANGLE = 60;
const int SERVO_MAX_ANGLE = 120;
const bool START_LINE_TRACKING_ON_BOOT = true;

/************ Yellow line tracking ************/
const int YELLOW_LINE_TARGET_X = 160;       // HUSKYLENS image width is about 320 pixels
const int YELLOW_LINE_ID = 1;               // learned yellow line color ID on HUSKYLENS
const float DEFAULT_YELLOW_LINE_STEER_GAIN = 0.08;  // higher = stronger steering response
const unsigned long YELLOW_LINE_LOST_GRACE_MS = 700; // keep running through brief camera misses

/************ LiDAR ************/
const float FRONT_STOP_DIST = 1000.0;  // mm
const float FRONT_CLEAR_DIST = 1200.0; // mm; must clear this distance before obstacle is released
const float FRONT_HALF_ANGLE = 35.0;   // only react to +/-35 degrees in front
const int LIDAR_BLOCK_CONFIRM_SCANS = 2;
const int LIDAR_CLEAR_CONFIRM_SCANS = 1;
const unsigned long SERIAL_REPORT_INTERVAL = 500;

/************ IMU heading control ************/
const float DEFAULT_HEADING_KP = 1.2;
const int MAX_IMU_CORRECTION = 20;
const unsigned long IMU_UPDATE_INTERVAL_MS = 100;
const unsigned long IMU_PRINT_INTERVAL_MS = 250;

/************ Avoidance ************/
const int AVOID_LEFT_STEER = 120;   // tune this if left/right are reversed
const int AVOID_RIGHT_STEER = 60;
const unsigned long CENTER_AFTER_AVOID_TIME = 800;

#endif
