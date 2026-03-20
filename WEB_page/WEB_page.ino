#include <WiFi.h>
#include <WiFiWebServer.h>
#include "webpage.h"

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

WiFiWebServer server(80);

float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
float setpoint = 512.0;

void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("Open this IP in your browser: http://");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/data", handleSensorData);
  server.on("/update", handlePidUpdate);

  server.begin();
}

void loop() {
  server.handleClient();
}

void handlePidUpdate() {
  Kp = server.arg("kp").toFloat();
  Ki = server.arg("ki").toFloat();
  Kd = server.arg("kd").toFloat();
  setpoint = server.arg("sp").toFloat();

  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Ki: ");
  Serial.println(Ki);
  Serial.print("Kd: ");
  Serial.println(Kd);
  Serial.print("Setpoint: ");
  Serial.println(setpoint);

  server.send(200, "text/plain", "PID values updated");
}

void handleSensorData() {
  int sensorValue = analogRead(A0);
  server.send(200, "text/plain", String(sensorValue));
}

void handleRoot() {
  server.send_P(200, "text/html", htmlPage);
}
