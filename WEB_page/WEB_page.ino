#include <WiFi.h>
#include <WiFiWebServer.h>
#include "webpage.h"

// Replace these with your WiFi name and password
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Create the web server on port 80
WiFiWebServer server(80);

// PID values
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
float setpoint = 512.0;

// Sensor value from A0
int sensorValue = 0;

// Used to update the sensor without blocking
unsigned long lastSensorTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
  }

  pinMode(A0, INPUT);

  Serial.println("Starting Arduino GIGA web server...");

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

  // Webpage routes
  server.on("/", handleRoot);
  server.on("/data", handleSensorData);
  server.on("/update", handlePidUpdate);

  server.begin();
  Serial.println("Server started");
}

void loop() {
  // Keep the server responsive
  server.handleClient();

  // Read the sensor every 200 ms
  if (millis() - lastSensorTime >= 200) {
    lastSensorTime = millis();
    sensorValue = analogRead(A0);
  }
}

// PID update handler
void handlePidUpdate() {
  if (server.hasArg("kp")) {
    Kp = server.arg("kp").toFloat();
  }

  if (server.hasArg("ki")) {
    Ki = server.arg("ki").toFloat();
  }

  if (server.hasArg("kd")) {
    Kd = server.arg("kd").toFloat();
  }

  if (server.hasArg("sp")) {
    setpoint = server.arg("sp").toFloat();
  }

  Serial.println("PID updated:");
  Serial.print("Kp = ");
  Serial.println(Kp);
  Serial.print("Ki = ");
  Serial.println(Ki);
  Serial.print("Kd = ");
  Serial.println(Kd);
  Serial.print("Setpoint = ");
  Serial.println(setpoint);
  Serial.println();

  server.send(200, "text/plain", "PID values updated");
}

// Sensor data handler
void handleSensorData() {
  server.send(200, "text/plain", String(sensorValue));
}

// Root webpage request
void handleRoot() {
  server.send_P(200, "text/html", htmlPage);
}
