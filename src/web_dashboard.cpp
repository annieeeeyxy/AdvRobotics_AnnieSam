#include "web_dashboard.h"

#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "imu_module.h"
#include "motor_control.h"
#include "state.h"

static WiFiServer server(WEB_PORT);

static String getValue(String req, String key) {
  int start = req.indexOf(key + "=");
  if (start == -1) return "";

  start += key.length() + 1;

  int end = req.indexOf("&", start);
  if (end == -1) end = req.indexOf(" ", start);
  if (end == -1) return "";

  return req.substring(start, end);
}

static void sendPage(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Cache-Control: no-store");
  client.println("Pragma: no-cache");
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
  client.print("document.addEventListener('keydown',function(e){if(e.code==='Space'){e.preventDefault();window.location.href='/stop';}});");
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

  client.print("<div class='card'><div class='label'>IMU Status</div><div class='value'>");
  client.print(isImuReady() ? "READY" : "FAILED");
  client.print("</div></div>");

  client.print("<div class='card'><div class='label'>Yaw</div><div class='value'>");
  if (isImuReady()) client.print(getYaw(), 1);
  else client.print("N/A");
  client.print("</div></div>");

  client.print("<div class='card'><div class='label'>Pitch</div><div class='value'>");
  if (isImuReady()) client.print(getPitch(), 1);
  else client.print("N/A");
  client.print("</div></div>");

  client.print("<div class='card'><div class='label'>Roll</div><div class='value'>");
  if (isImuReady()) client.print(getRoll(), 1);
  else client.print("N/A");
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

static void handleClient(WiFiClient& client) {
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
    stopMotor();
  } else if (req.indexOf("/resume") != -1) {
    emergencyStop = false;
    mode = FOLLOW_LINE;
    resetPidState();
    setEscSpeed(motorSpeed);
  } else if (req.indexOf("/set") != -1) {
    String sKp = getValue(req, "kp");
    String sKi = getValue(req, "ki");
    String sKd = getValue(req, "kd");
    String sSpeed = getValue(req, "speed");

    if (sKp != "") kp = sKp.toFloat();
    if (sKi != "") ki = sKi.toFloat();
    if (sKd != "") kd = sKd.toFloat();
    if (sSpeed != "") motorSpeed = constrain(sSpeed.toInt(), 0, 180);

    if (!emergencyStop) {
      setEscSpeed(motorSpeed);
    }
  }

  sendPage(client);
  client.stop();
}

void setupWebDashboard() {
#if ENABLE_WEB
  WiFi.beginAP(WIFI_SSID, WIFI_PASS);
  server.begin();
#endif
}

void serviceWebDashboard() {
#if ENABLE_WEB
  WiFiClient client = server.accept();
  if (client) {
    handleClient(client);
  }
#endif
}
