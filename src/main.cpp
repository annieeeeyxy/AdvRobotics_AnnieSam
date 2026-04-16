#include <Wire.h>
#include "HUSKYLENS.h"
#include <Servo.h>
#include <WiFi.h>
#include <RPLidar.h>

/************ WiFi ************/
#define WIFI_SSID "GIGA_PID"
#define WIFI_PASS "12345678"
#define PORT 80

WiFiServer server(PORT);

/************ Devices ************/
HUSKYLENS husky;
Servo steerServo;
Servo escMotor;
RPLidar lidar;

/************ Pins ************/
const int SERVO_PIN = 7;
const int ESC_PIN = 6;
const int LIDAR_MOTOR_PIN = 5;

/************ PID ************/
float kp = 0.35;
float ki = 0.00;
float kd = 0.15;

float errorSum = 0;
float lastError = 0;
unsigned long lastPidTime = 0;

/************ Settings ************/
int servoCenter = 90;
int currentServoPosition = 90;
int imageCenter = 160;
int motorSpeed = 90;
bool emergencyStop = false;

/************ LiDAR ************/
float frontDistance = 10000.0;
float frontAngle = -1.0;
bool frontBlocked = false;
unsigned long lastSerialReport = 0;

const float FRONT_STOP_DIST = 900.0;   // mm
const float FRONT_HALF_ANGLE = 35.0;   // only react to +/-35 degrees in front
const unsigned long SERIAL_REPORT_INTERVAL = 500;   // print LiDAR status every 0.5 seconds

/************ Avoidance ************/
enum Mode {
  FOLLOW_LINE,
  AVOID_OBJECT,
  CENTER_AFTER_AVOID,
  FIND_LINE
};

Mode mode = FOLLOW_LINE;
const int AVOID_LEFT_STEER = 120;   // tune this if left/right are reversed
const int AVOID_RIGHT_STEER = 60;
const unsigned long CENTER_AFTER_AVOID_TIME = 800;  // ms
unsigned long centerAfterAvoidStart = 0;

/************************************************
   Small helpers
************************************************/
String getValue(String req, String key) {
  int start = req.indexOf(key + "=");
  if (start == -1) return "";

  start += key.length() + 1;

  int end = req.indexOf("&", start);
  if (end == -1) end = req.indexOf(" ", start);
  if (end == -1) return "";

  return req.substring(start, end);
}

const char* modeName() {
  if (mode == FOLLOW_LINE) return "FOLLOW_LINE";
  if (mode == AVOID_OBJECT) return "AVOID_OBJECT";
  if (mode == CENTER_AFTER_AVOID) return "CENTER_AFTER_AVOID";
  if (mode == FIND_LINE) return "FIND_LINE";
  return "UNKNOWN";
}

void setSteeringServo(int angle) {
  currentServoPosition = constrain(angle, 60, 120);
  steerServo.write(currentServoPosition);
}

int chooseAvoidSteer() {
  if (frontAngle >= 360.0 - FRONT_HALF_ANGLE) return AVOID_RIGHT_STEER;
  if (frontAngle <= FRONT_HALF_ANGLE) return AVOID_LEFT_STEER;
  return AVOID_LEFT_STEER;
}

/************************************************
   Web page
************************************************/
void sendPage(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();

  client.print("<!DOCTYPE html><html><head>");
  client.print("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  client.print("<title>Nimbus Dashboard</title>");

  client.print("<style>");
  client.print("body{font-family:Arial,sans-serif;margin:0;background:#f4f6f8;color:#1f2933;}");
  client.print(".wrap{max-width:960px;margin:auto;padding:20px;}");
  client.print(".box{background:white;border:1px solid #d9e0e6;border-radius:16px;padding:20px;margin-bottom:18px;}");
  client.print(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:12px;}");
  client.print(".card{background:white;border:1px solid #d9e0e6;border-radius:14px;padding:16px;}");
  client.print(".label{font-size:12px;color:#5b6875;text-transform:uppercase;}");
  client.print(".value{font-size:24px;font-weight:bold;margin-top:8px;}");
  client.print(".btn{width:100%;padding:14px;border:none;border-radius:14px;font-weight:bold;cursor:pointer;}");
  client.print(".blue{background:#2f6ea3;color:white;}");
  client.print(".red{background:#c73b3b;color:white;}");
  client.print(".green{background:#2f7d4f;color:white;}");
  client.print(".row{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;}");
  client.print("input[type=text],input[type=range]{width:100%;padding:12px;border:1px solid #d9e0e6;border-radius:10px;box-sizing:border-box;}");
  client.print("</style>");

  client.print("<script>");
  client.print("function showSpeed(v){document.getElementById('speedText').innerHTML=v;}");
  client.print("document.addEventListener('keydown', function(e){if(e.code==='Space'){e.preventDefault();window.location.href='/stop';}});");
  client.print("</script>");

  client.print("</head><body><div class='wrap'>");

  client.print("<div class='box'>");
  client.print("<h2>Nimbus Line + LiDAR Dashboard</h2>");
  client.print("<div class='grid'>");

  client.print("<div class='card'><div class='label'>Mode</div><div class='value'>");
  client.print(modeName());
  client.print("</div></div>");

  client.print("<div class='card'><div class='label'>Kp</div><div class='value'>");
  client.print(kp, 2);
  client.print("</div></div>");

  client.print("<div class='card'><div class='label'>Ki</div><div class='value'>");
  client.print(ki, 2);
  client.print("</div></div>");

  client.print("<div class='card'><div class='label'>Kd</div><div class='value'>");
  client.print(kd, 2);
  client.print("</div></div>");

  client.print("<div class='card'><div class='label'>Motor Speed</div><div class='value'><span id='speedText'>");
  client.print(motorSpeed);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Servo Position</div><div class='value'>");
  client.print(currentServoPosition);
  client.print("</div></div>");

  client.print("<div class='card'><div class='label'>Front Distance</div><div class='value'>");
  if (frontDistance < 9999) client.print(frontDistance, 0);
  else client.print("none");
  client.print("</div></div>");

  client.print("<div class='card'><div class='label'>Front Angle</div><div class='value'>");
  if (frontAngle >= 0) client.print(frontAngle, 0);
  else client.print("none");
  client.print("</div></div>");

  client.print("<div class='card'><div class='label'>Front Blocked</div><div class='value'>");
  client.print(frontBlocked ? "YES" : "NO");
  client.print("</div></div>");

  client.print("<div class='card'><div class='label'>Emergency Stop</div><div class='value'>");
  client.print(emergencyStop ? "ACTIVE" : "READY");
  client.print("</div></div>");

  client.print("</div></div>");

  client.print("<div class='box'>");
  client.print("<h3>Control</h3>");
  client.print("<form action='/set'>");
  client.print("<div class='row'>");

  client.print("<div><div class='label'>Kp</div><input type='text' name='kp' value='");
  client.print(kp, 2);
  client.print("'></div>");

  client.print("<div><div class='label'>Ki</div><input type='text' name='ki' value='");
  client.print(ki, 2);
  client.print("'></div>");

  client.print("<div><div class='label'>Kd</div><input type='text' name='kd' value='");
  client.print(kd, 2);
  client.print("'></div>");

  client.print("</div><br>");

  client.print("<div class='label'>Motor Speed</div>");
  client.print("<input type='range' name='speed' min='0' max='180' value='");
  client.print(motorSpeed);
  client.print("' oninput='showSpeed(this.value)'>");

  client.print("<br><br><input class='btn blue' type='submit' value='Apply Settings'>");
  client.print("</form><br>");

  client.print("<div class='row'>");
  client.print("<form action='/stop'><input class='btn red' type='submit' value='Emergency Stop'></form>");
  client.print("<form action='/resume'><input class='btn green' type='submit' value='Resume'></form>");
  client.print("</div>");

  client.print("</div></div></body></html>");
}

/************************************************
   Web request
************************************************/
void handleClient(WiFiClient& client) {
  unsigned long start = millis();
  while (client.connected() && !client.available()) {
    if (millis() - start > 1000) {
      client.stop();
      return;
    }
  }

  String req = client.readStringUntil('\r');

  while (client.available()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }

  if (req.indexOf("/stop") != -1) {
    emergencyStop = true;
    setSteeringServo(servoCenter);
    escMotor.write(90);
  }
  else if (req.indexOf("/resume") != -1) {
    emergencyStop = false;
    mode = FOLLOW_LINE;
    errorSum = 0;
    lastError = 0;
    lastPidTime = millis();
    escMotor.write(motorSpeed);
  }
  else if (req.indexOf("/set") != -1) {
    String sKp = getValue(req, "kp");
    String sKi = getValue(req, "ki");
    String sKd = getValue(req, "kd");
    String sSpeed = getValue(req, "speed");

    if (sKp != "") kp = sKp.toFloat();
    if (sKi != "") ki = sKi.toFloat();
    if (sKd != "") kd = sKd.toFloat();
    if (sSpeed != "") motorSpeed = constrain(sSpeed.toInt(), 0, 180);

    if (!emergencyStop) {
      escMotor.write(motorSpeed);
    }
  }

  sendPage(client);
  client.stop();
}

/************************************************
   LiDAR
************************************************/
void restartLidar() {
  analogWrite(LIDAR_MOTOR_PIN, 0);
  delay(50);

  rplidar_response_device_info_t info;
  if (IS_OK(lidar.getDeviceInfo(info, 100))) {
    analogWrite(LIDAR_MOTOR_PIN, 255);
    delay(1000);
    lidar.startScan();
    Serial.println("LiDAR restarted");
  } else {
    Serial.println("LiDAR not found");
  }
}

void readFrontLidar() {
  frontDistance = 10000.0;
  frontAngle = -1.0;
  frontBlocked = false;

  unsigned long start = millis();

  while (millis() - start < 100) {
    if (!IS_OK(lidar.waitPoint())) {
      Serial.println("LiDAR disconnected");
      restartLidar();
      return;
    }

    float d = lidar.getCurrentPoint().distance;
    float a = lidar.getCurrentPoint().angle;
    float q = lidar.getCurrentPoint().quality;

    if (q <= 10 || d <= 0) continue;

    bool inFront = (a >= 360.0 - FRONT_HALF_ANGLE || a <= FRONT_HALF_ANGLE);

    if (inFront && d < frontDistance) {
      frontDistance = d;
      frontAngle = a;
    }
  }

  if (frontDistance < FRONT_STOP_DIST) {
    frontBlocked = true;
  }

  unsigned long now = millis();
  if (now - lastSerialReport < SERIAL_REPORT_INTERVAL) return;
  lastSerialReport = now;

  Serial.print("Front distance: ");
  if (frontDistance < 9999) Serial.print(frontDistance);
  else Serial.print("none");
  Serial.print(" mm | angle: ");
  if (frontAngle >= 0) Serial.print(frontAngle);
  else Serial.print("none");
  Serial.print(" | blocked: ");
  Serial.print(frontBlocked ? "YES" : "NO");
  Serial.print(" | servo: ");
  Serial.print(currentServoPosition);
  Serial.print(" | mode: ");
  Serial.println(modeName());
}

/************************************************
   Robot behavior
************************************************/
void followLine() {
  if (!husky.request()) return;
  if (!husky.available()) return;

  HUSKYLENSResult result = husky.read();
  if (result.command != COMMAND_RETURN_ARROW) return;

  unsigned long now = millis();
  float dt = (now - lastPidTime) / 1000.0;
  if (dt <= 0) return;

  float error = result.xTarget - imageCenter;
  errorSum += error * dt;

  float dError = (error - lastError) / dt;
  float output = kp * error + ki * errorSum + kd * dError;

  int steer = servoCenter - output;
  setSteeringServo(steer);
  escMotor.write(motorSpeed);

  lastError = error;
  lastPidTime = now;
}

void avoidObject() {
  setSteeringServo(chooseAvoidSteer());
  escMotor.write(motorSpeed);

  if (!frontBlocked) {
    mode = CENTER_AFTER_AVOID;
    centerAfterAvoidStart = millis();
  }
}

void centerAfterAvoid() {
  setSteeringServo(servoCenter);
  escMotor.write(motorSpeed);

  if (frontBlocked) {
    mode = AVOID_OBJECT;
    return;
  }

  if (millis() - centerAfterAvoidStart >= CENTER_AFTER_AVOID_TIME) {
    mode = FIND_LINE;
  }
}

void findLine() {
  setSteeringServo(servoCenter);
  escMotor.write(motorSpeed);

  if (frontBlocked) {
    mode = AVOID_OBJECT;
    return;
  }

  if (!husky.request()) return;
  if (!husky.available()) return;

  HUSKYLENSResult result = husky.read();
  if (result.command == COMMAND_RETURN_ARROW) {
    errorSum = 0;
    lastError = 0;
    lastPidTime = millis();
    mode = FOLLOW_LINE;
  }
}

/************************************************
   Setup
************************************************/
void setup() {
  Serial.begin(115200);
  delay(1000);

  // WiFi AP
  WiFi.beginAP(WIFI_SSID, WIFI_PASS);
  server.begin();

  // I2C
  Wire.begin();

  // servo + ESC
  steerServo.attach(SERVO_PIN);
  escMotor.attach(ESC_PIN);

  setSteeringServo(servoCenter);
  escMotor.write(90);

  // HUSKYLENS
  while (!husky.begin(Wire)) {
    Serial.println("HUSKYLENS not found");
    delay(100);
  }
  husky.writeAlgorithm(ALGORITHM_LINE_TRACKING);

  // LiDAR on Serial2
  Serial2.begin(115200);
  lidar.begin(Serial2);
  pinMode(LIDAR_MOTOR_PIN, OUTPUT);
  analogWrite(LIDAR_MOTOR_PIN, 255);
  delay(1000);
  lidar.startScan();

  lastPidTime = millis();

  Serial.println("Ready");
  Serial.println("WiFi: GIGA_PID");
  Serial.println("Open: http://192.168.3.1");
}

/************************************************
   Loop
************************************************/
void loop() {
  WiFiClient client = server.accept();
  if (client) {
    handleClient(client);
  }

  if (emergencyStop) {
    setSteeringServo(servoCenter);
    escMotor.write(90);
    return;
  }

  readFrontLidar();

  if (mode == FOLLOW_LINE && frontBlocked) {
    mode = AVOID_OBJECT;
  }

  if (mode == FOLLOW_LINE) {
    followLine();
  } else if (mode == AVOID_OBJECT) {
    avoidObject();
  } else if (mode == CENTER_AFTER_AVOID) {
    centerAfterAvoid();
  } else if (mode == FIND_LINE) {
    findLine();
  }
}
