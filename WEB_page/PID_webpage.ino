#include <WiFi.h>
#include "webpage.h"

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

WiFiServer server(80);

float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
float setpoint = 512.0;

String getValue(String request, String name);
void handlePidUpdate(WiFiClient client, String request);
void handleSensorData(WiFiClient client);
void handleRoot(WiFiClient client);

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

  server.begin();
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    String request = client.readStringUntil('\r');
    client.flush();

    if (request.indexOf("GET /update?") >= 0) {
      handlePidUpdate(client, request);
    } else if (request.indexOf("GET /data") >= 0) {
      handleSensorData(client);
    } else {
      handleRoot(client);
    }

    delay(1);
    client.stop();
  }
}

String getValue(String request, String name) {
  int start = request.indexOf(name + "=");
  if (start == -1) {
    return "";
  }

  start = start + name.length() + 1;
  int end = request.indexOf("&", start);

  if (end == -1) {
    end = request.indexOf(" ", start);
  }

  return request.substring(start, end);
}

void handlePidUpdate(WiFiClient client, String request) {
  String kpValue = getValue(request, "kp");
  String kiValue = getValue(request, "ki");
  String kdValue = getValue(request, "kd");
  String spValue = getValue(request, "sp");

  Kp = kpValue.toFloat();
  Ki = kiValue.toFloat();
  Kd = kdValue.toFloat();
  setpoint = spValue.toFloat();

  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Ki: ");
  Serial.println(Ki);
  Serial.print("Kd: ");
  Serial.println(Kd);
  Serial.print("Setpoint: ");
  Serial.println(setpoint);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println("PID values updated");
}

void handleSensorData(WiFiClient client) {
  int sensorValue = analogRead(A0);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println(sensorValue);
}

void handleRoot(WiFiClient client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.print(htmlPage);
}
