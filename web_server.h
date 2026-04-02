#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include "DHTesp.h"

// ----------- Wi-Fi Credentials -----------
extern const char* WIFI_SSID;
extern const char* WIFI_PASS;

// ----------- Backend Configuration -----------
extern String backendHost;
extern int backendPort;
extern const char* DEVICE_ID;
extern const char* LAB_NAME;

// ----------- Sensor Objects (from main.ino) -----------
extern DHTesp dht;
extern int MQ2_A0;

// Flame detection (array of 5 sensors — see fire_sensor.h)
extern int flameDO[5];
extern bool checkFire();  // Returns true if ANY flame sensor triggers

// Flame detection variables
extern bool fireDetected;
extern bool sirenActive;

// ----------- Web Server Object (declared in main .ino) -----------
extern WebServer server;

// ----------- Helper Functions -----------
String getFireStatus(bool fireDetected) {
  return fireDetected ? "FIRE" : "NORMAL";
}

String getSmokeStatus(int gasValue, int threshold) {
  return (gasValue > threshold) ? "SMOKE" : "CLEAR";
}

// ----------- Determine Event Type for Backend -----------
String getEventType(float temp, int gasValue, bool fireDetected) {
  // Critical conditions (matching GUIDE.md thresholds)
  if (fireDetected || temp >= 42.0 || gasValue >= 600) {
    return "critical";
  }
  // Warning conditions
  if (temp >= 35.0 || gasValue >= 400) {
    return "warning";
  }
  return "normal";
}

// ----------- Get Siren Status -----------
String getSirenStatus(bool sirenActive) {
  return sirenActive ? "active" : "inactive";
}

// ----------- Setup Wi-Fi and Server -----------
void webServerSetup() {
  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to Wi-Fi");
  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retryCount++;
    if (retryCount > 30) {
      Serial.println("\nFailed to connect Wi-Fi!");
      return;
    }
  }
  Serial.println("\nWi-Fi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Define /data endpoint for JSON (compatible with backend expectations)
  server.on("/data", []() {
    TempAndHumidity th = dht.getTempAndHumidity();
    int gasValue = analogRead(MQ2_A0);
    bool fireDetectedLocal = checkFire();  // Uses all 5 flame sensors

    String fireStatus = getFireStatus(fireDetectedLocal);
    String smokeStatus = getSmokeStatus(gasValue, 300);
    String eventType = getEventType(th.temperature, gasValue, fireDetectedLocal);
    String sirenStatus = getSirenStatus(sirenActive);

    // Build JSON response with device info for backend compatibility
    DynamicJsonDocument doc(512);
    doc["deviceId"] = DEVICE_ID;
    doc["labName"] = LAB_NAME;
    doc["temperature"] = th.temperature;
    doc["humidity"] = th.humidity;
    doc["gas"] = gasValue;
    doc["fire"] = fireStatus;
    doc["smoke"] = smokeStatus;
    doc["eventType"] = eventType;
    doc["siren"] = sirenStatus;
    doc["online"] = true;

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  // Health check endpoint (matching backend /api/health)
  server.on("/health", []() {
    DynamicJsonDocument doc(128);
    doc["success"] = true;
    doc["message"] = "Device is running";
    doc["deviceId"] = DEVICE_ID;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  server.begin();
  Serial.println("Web server started on port 80");
}

// ----------- Loop Handler -----------
void webServerLoop() {
  server.handleClient();
}

#endif
