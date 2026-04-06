#include <Wire.h>
#include "HUSKYLENS.h"
#include <Servo.h>
#include <WiFi.h>

/**************** WIFI ****************/
#define WIFI_SSID "GIGA_PID"
#define WIFI_PASS "12345678"
#define PORT 80

WiFiServer server(PORT);

/**************** DEVICES ****************/
HUSKYLENS huskylens;
Servo steerServo;   // steering servo
Servo escMotor;     // ESC motor

/**************** PINS ****************/
const int SERVO_PIN = 7;
const int ESC_PIN   = 6;

/**************** PID ****************/
float Kp = 0.35;
float Ki = 0.0;
float Kd = 0.15;

float errorSum = 0;
float lastError = 0;
unsigned long lastTime = 0;

/**************** SETTINGS ****************/
int servoCenter = 90;
int imageCenter = 160;
int motorSpeed = 90;      // initial motor speed
bool emergencyStop = false;

/**********************************************************
   Make webpage
**********************************************************/
String makePage() {
  String page = "";

  page += "<!DOCTYPE html><html><head>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  page += "<title>PID Dashboard</title>";

  page += "<style>";
  page += "body{font-family:Arial;text-align:center;margin-top:30px;}";
  page += "input[type=range]{width:300px;}";
  page += "input,button{font-size:16px;padding:8px;margin:6px;}";
  page += "</style>";

  page += "<script>";
  page += "function showSpeed(val){document.getElementById('speedText').innerHTML=val;}";
  page += "</script>";

  page += "</head><body>";

  page += "<h2>PID Dashboard</h2>";

  page += "<p>Kp = " + String(Kp, 2) + "</p>";
  page += "<p>Ki = " + String(Ki, 2) + "</p>";
  page += "<p>Kd = " + String(Kd, 2) + "</p>";
  page += "<p>Motor Speed = <span id='speedText'>" + String(motorSpeed) + "</span></p>";
  page += "<p>Emergency Stop = ";
  page += (emergencyStop ? "ON" : "OFF");
  page += "</p>";

  page += "<form action='/set'>";
  page += "Kp: <input name='kp' value='" + String(Kp, 2) + "'><br>";
  page += "Ki: <input name='ki' value='" + String(Ki, 2) + "'><br>";
  page += "Kd: <input name='kd' value='" + String(Kd, 2) + "'><br><br>";

  page += "Motor Speed:<br>";
  page += "<input type='range' name='speed' min='0' max='180' value='" + String(motorSpeed) + "' oninput='showSpeed(this.value)'><br><br>";

  page += "<input type='submit' value='Update'>";
  page += "</form><br>";

  page += "<form action='/stop'>";
  page += "<input type='submit' value='EMERGENCY STOP' style='background:red;color:white;'>";
  page += "</form>";

  page += "<form action='/resume'>";
  page += "<input type='submit' value='RESUME' style='background:green;color:white;'>";
  page += "</form>";

  page += "</body></html>";

  return page;
}

/**********************************************************
   Get value from URL
   Example: if URL has kp=0.5, getValue(req, "kp") returns "0.5"
**********************************************************/
String getValue(String req, String name) {
  int start = req.indexOf(name + "=");
  if (start == -1) return "";

  start = start + name.length() + 1;

  int end = req.indexOf("&", start);
  if (end == -1) end = req.indexOf(" ", start);

  if (end == -1) return "";

  return req.substring(start, end);
}

/**********************************************************
   Handle webpage request
**********************************************************/
void handleClient(WiFiClient& client) {
  String req = client.readStringUntil('\r');
  client.flush();

  // STOP
  if (req.indexOf("/stop") != -1) {
    emergencyStop = true;
    steerServo.write(servoCenter);
    escMotor.write(90);   // neutral / stop
  }

  // RESUME
  else if (req.indexOf("/resume") != -1) {
    emergencyStop = false;
    errorSum = 0;
    lastError = 0;
    lastTime = millis();
    escMotor.write(motorSpeed);
  }

  // UPDATE PID + SPEED
  else if (req.indexOf("/set") != -1) {
    String kpText = getValue(req, "kp");
    String kiText = getValue(req, "ki");
    String kdText = getValue(req, "kd");
    String speedText = getValue(req, "speed");

    if (kpText != "") Kp = kpText.toFloat();
    if (kiText != "") Ki = kiText.toFloat();
    if (kdText != "") Kd = kdText.toFloat();
    if (speedText != "") motorSpeed = speedText.toInt();

    motorSpeed = constrain(motorSpeed, 0, 180);

    if (!emergencyStop) {
      escMotor.write(motorSpeed);
    }
  }

  // Send webpage
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println();
  client.println(makePage());
  client.stop();
}

/**********************************************************
   Setup
**********************************************************/
void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("Starting AP...");

  int status = WiFi.beginAP(WIFI_SSID, WIFI_PASS);
  if (status != WL_AP_LISTENING) {
    Serial.println("AP FAIL");
    while (1);
  }

  server.begin();

  Serial.println("Connect WiFi: GIGA_PID");
  Serial.println("Open browser: http://192.168.3.1");

  // I2C
  Wire.begin();

  // Servo
  steerServo.attach(SERVO_PIN);
  steerServo.write(servoCenter);

  // ESC
  escMotor.attach(ESC_PIN);
  escMotor.write(motorSpeed);   // initial = 90

  // HUSKYLENS
  while (!huskylens.begin(Wire)) {
    Serial.println("HUSKY FAIL");
    delay(100);
  }

  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);

  lastTime = millis();
}

/**********************************************************
   Main loop
**********************************************************/
void loop() {
  // Check webpage client
  WiFiClient client = server.accept();
  if (client) {
    handleClient(client);
  }

  // If stopped, keep servo centered and motor neutral
  if (emergencyStop) {
    steerServo.write(servoCenter);
    escMotor.write(90);
    return;
  }

  // Ask HUSKYLENS for data
  if (!huskylens.request()) return;
  if (!huskylens.available()) return;

  HUSKYLENSResult result = huskylens.read();

  // Only use arrow result for line tracking
  if (result.command == COMMAND_RETURN_ARROW) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;

    if (dt <= 0) return;

    // Error = line target position - image center
    float error = result.xTarget - imageCenter;

    // PID
    errorSum = errorSum + error * dt;
    float dError = (error - lastError) / dt;

    float output = Kp * error + Ki * errorSum + Kd * dError;

    // Servo steering
    int servoValue = servoCenter - output;
    servoValue = constrain(servoValue, 60, 120);

    steerServo.write(servoValue);

    lastError = error;
    lastTime = now;
  }
}