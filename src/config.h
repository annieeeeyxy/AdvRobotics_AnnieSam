#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/************ Compile-time module switches ************/
#define ENABLE_IMU_STANDALONE_TEST 0
#define ENABLE_WEB 1
#define ENABLE_IMU 1
#define ENABLE_LIDAR 1
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

/************ PID defaults ************/
const float DEFAULT_KP = 0.22;
const float DEFAULT_KI = 0.00;
const float DEFAULT_KD = 0.06;

/************ Line tracking sensitivity ************/
const float LINE_ERROR_DEADBAND_PIXELS = 8.0;  // ignore small HUSKYLENS jitter near center
const float LINE_OUTPUT_SMOOTHING = 0.65;      // larger = smoother/slower steering response
const int MAX_LINE_SERVO_STEP = 4;             // max degrees servo can move per line update

/************ Robot settings ************/
const int DEFAULT_SERVO_CENTER = 90;
const int DEFAULT_IMAGE_CENTER = 160;
const int DEFAULT_MOTOR_SPEED = 95;
const int ESC_NEUTRAL = 90;
const int SERVO_MIN_ANGLE = 60;
const int SERVO_MAX_ANGLE = 120;

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
const unsigned long IMU_UPDATE_INTERVAL_MS = 500;
const unsigned long IMU_PRINT_INTERVAL_MS = 1000;
const unsigned long IMU_REGISTER_READ_DELAY_MS = 8;

/************ Avoidance ************/
const int AVOID_LEFT_STEER = 120;   // tune this if left/right are reversed
const int AVOID_RIGHT_STEER = 60;
const unsigned long CENTER_AFTER_AVOID_TIME = 800;

#endif
