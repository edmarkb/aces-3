#include <HardwareSerial.h>
#include <DHTesp.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include "siren_control.h"
#include "siren_automatic.h"
#include "mq2_sensor.h"
#include "oled_display.h"
#include "fire_sensor.h"

// ============================================
// ⚡ DEVICE CONFIGURATION — CHANGE FOR EACH BOARD!
// ============================================
// Each ESP32 board MUST have a unique DEVICE_ID.
// Valid IDs: "ACES-1", "ACES-2", "ACES-3"
// The backend rejects any other format.
// See ESP32_DEVICE_ID_GUIDE.md for full details.
//
//   Board #1 → "ACES-1"  (Computer Laboratory 1)
//   Board #2 → "ACES-2"  (Computer Laboratory 2)
//   Board #3 → "ACES-3"  (Computer Laboratory 3)
//
// >>> ONLY CHANGE THESE TWO LINES: <<<
const char* DEVICE_ID = "ACES-3";
const char* LAB_NAME  = "Food Laboratory";
// ============================================

// ============================================
// DEVICE VALIDATION
// ============================================
// Validate DEVICE_ID at startup — halts with error if invalid
void validateDeviceId() {
  const char* validIds[] = {"ACES-1", "ACES-2", "ACES-3"};
  for (int i = 0; i < 3; i++) {
    if (strcmp(DEVICE_ID, validIds[i]) == 0) return; // Valid!
  }
  // Invalid DEVICE_ID — halt with clear error
  Serial.println("\n❌❌❌ INVALID DEVICE_ID: " + String(DEVICE_ID));
  Serial.println("   Must be exactly: ACES-1, ACES-2, or ACES-3");
  Serial.println("   Fix DEVICE_ID at top of ACES3.ino and re-flash.");
  Serial.println("   See ESP32_DEVICE_ID_GUIDE.md for help.");
  Serial.println("❌❌❌ HALTING — fix DEVICE_ID and re-upload!\n");
  while (true) { delay(10000); } // Halt forever
}

// ============================================
// WIFI CONFIGURATION
// ============================================
const char* WIFI_SSID = "ACES-IOT WIFI";
const char* WIFI_PASS = "bestinthesispls";

// ============================================
// BACKEND CONFIGURATION
// ============================================
String backendHost = "192.168.8.10";
int backendPort = 3000;

// ============================================
// PIN DEFINITIONS
// ============================================
int DHT_PIN = 15;

// ----------- Detection Flags -----------
bool fireDetected = false;
bool gasDetected = false;
bool lastFlameState = false;       // Track flame transitions for immediate POST

// ----------- Backend Event Tracking -----------
bool lastWarningState = false;
bool lastCriticalState = false;
unsigned long lastBackendLog = 0;
const unsigned long BACKEND_LOG_COOLDOWN = 6000; // 6 seconds between logs (staggered)

// ----------- Objects -----------
HardwareSerial sim800(1);
DHTesp dht;
WebServer server(80);

// ----------- SMS Cooldown -----------
unsigned long lastSMS = 0;
const unsigned long SMS_COOLDOWN = 10000;

// ----------- DHT Cooldown -----------
TempAndHumidity dhtData;
unsigned long lastDHTread = 0;

// ----------- Sensor Data Pushing (Push Architecture) -----------
// ESP32 pushes data to backend via HTTP POST.
// Uses FAST interval (500ms) when siren is active for responsive control,
// normal 2s interval otherwise to reduce server load.
// Backend no longer polls ESP32 — see PUSH_ARCHITECTURE_GUIDE.md
unsigned long lastHttpPost = 0;
const unsigned long HTTP_POST_INTERVAL_NORMAL = 2000;  // 2s when idle
const unsigned long HTTP_POST_INTERVAL_FAST   = 500;   // 500ms when siren active

unsigned long getPostInterval() {
  return sirenActive ? HTTP_POST_INTERVAL_FAST : HTTP_POST_INTERVAL_NORMAL;
}

// ----------- Operation Stagger -----------
// Prevents multiple HTTP calls from firing at the same time
bool httpBusy = false;
unsigned long httpBusySince = 0;           // When httpBusy was last set to true
unsigned long lastSuccessfulPost = 0;      // Last time a POST got a 200 response
const unsigned long HTTP_BUSY_TIMEOUT = 10000;   // Force-clear httpBusy after 10s
const unsigned long HTTP_RECOVERY_TIMEOUT = 60000; // Restart WiFi after 60s of no POST

// ----------- SSE (Server-Sent Events) for Instant Siren Control -----------
// Persistent connection to backend — server pushes siren commands instantly.
// POST response sirenCommand is kept as fallback.
WiFiClient sseClient;
String sseBuffer = "";
bool sseConnected = false;
unsigned long lastSSEAttempt = 0;
const unsigned long SSE_RECONNECT_INTERVAL = 3000;  // Retry every 3s if disconnected

// ----------- Phone Numbers -----------
#define MAX_NUMBERS 10  // Support up to 10 alert contacts
String PHONE_NUMBERS[MAX_NUMBERS];
int PHONE_COUNT = 0;

// ==========================
// NON-BLOCKING SMS QUEUE + STATE MACHINE — SIM800L
// ==========================
// SMS sending is non-blocking. Numbers are added to a queue,
// and processSMSQueue() sends one SMS per loop() iteration.
// This ensures siren commands (SSE) are never blocked while SMS is sending.

#define SMS_QUEUE_SIZE 20  // Max queued SMS (10 contacts × 2 events = 20)

// ---- SMS Queue ----
struct SMSItem {
  String phoneNumber;
  String message;
};

SMSItem smsQueue[SMS_QUEUE_SIZE];
int smsQueueHead = 0;   // Next item to send
int smsQueueTail = 0;   // Next free slot
int smsQueueCount = 0;  // Items in queue

// SMS sending state machine
enum SMSState { SMS_IDLE, SMS_SET_MODE, SMS_WAIT_MODE, SMS_SET_NUMBER, SMS_WAIT_PROMPT, SMS_SEND_BODY, SMS_WAIT_OK, SMS_COOL_DOWN };
SMSState smsState = SMS_IDLE;
unsigned long smsStepStart = 0;
String smsResponse = "";

// Add SMS to the queue (non-blocking, returns immediately)
void queueSMS(String phoneNumber, String message) {
  if (smsQueueCount >= SMS_QUEUE_SIZE) {
    Serial.println("⚠️ SMS queue full — dropping: " + phoneNumber);
    return;
  }
  smsQueue[smsQueueTail].phoneNumber = phoneNumber;
  smsQueue[smsQueueTail].message = message;
  smsQueueTail = (smsQueueTail + 1) % SMS_QUEUE_SIZE;
  smsQueueCount++;
  Serial.println("📱 Queued SMS to: " + phoneNumber + " (" + String(smsQueueCount) + " in queue)");
}

// Call this in loop() — processes one SMS at a time without blocking
void processSMSQueue() {
  unsigned long now = millis();

  // Read any available SIM800L data into smsResponse
  while (sim800.available()) {
    char c = sim800.read();
    smsResponse += c;
  }

  switch (smsState) {

    case SMS_IDLE:
      // Nothing to send?
      if (smsQueueCount == 0) return;

      // Start sending next SMS
      Serial.println("📱 Sending SMS to: " + smsQueue[smsQueueHead].phoneNumber);
      smsResponse = "";
      sim800.println("AT+CMGF=1");
      smsStepStart = now;
      smsState = SMS_WAIT_MODE;
      break;

    case SMS_WAIT_MODE:
      if (smsResponse.indexOf("OK") >= 0 || now - smsStepStart > 2000) {
        smsResponse = "";
        sim800.println("AT+CMGS=\"" + smsQueue[smsQueueHead].phoneNumber + "\"");
        smsStepStart = now;
        smsState = SMS_WAIT_PROMPT;
      }
      break;

    case SMS_WAIT_PROMPT:
      if (smsResponse.indexOf(">") >= 0 || now - smsStepStart > 3000) {
        smsResponse = "";
        sim800.print(smsQueue[smsQueueHead].message);
        delay(100);  // Brief pause before Ctrl+Z (required by SIM800L)
        sim800.write(26);  // Ctrl+Z to send
        smsStepStart = now;
        smsState = SMS_WAIT_OK;
      }
      break;

    case SMS_WAIT_OK:
      if (smsResponse.indexOf("OK") >= 0) {
        Serial.println("✅ SMS sent to " + smsQueue[smsQueueHead].phoneNumber);
        smsStepStart = now;
        smsState = SMS_COOL_DOWN;
      } else if (smsResponse.indexOf("ERROR") >= 0 || now - smsStepStart > 10000) {
        Serial.println("❌ SMS failed to " + smsQueue[smsQueueHead].phoneNumber + " | " + smsResponse);
        smsStepStart = now;
        smsState = SMS_COOL_DOWN;
      }
      break;

    case SMS_COOL_DOWN:
      // 2s gap between SMS — SIM800L needs recovery time
      if (now - smsStepStart >= 2000) {
        // Dequeue the sent item
        smsQueueHead = (smsQueueHead + 1) % SMS_QUEUE_SIZE;
        smsQueueCount--;
        smsResponse = "";
        smsState = SMS_IDLE;  // Will pick up next item on next loop()

        if (smsQueueCount > 0) {
          Serial.println("📱 " + String(smsQueueCount) + " SMS remaining in queue");
        } else {
          Serial.println("📱 All SMS sent");
        }
      }
      break;

    default:
      smsState = SMS_IDLE;
      break;
  }
}

// ==========================
// BACKEND LOG FUNCTION
// ==========================
String getTimestamp() {
  // Returns timestamp in format: "2/5/2026, 10:30:00 AM"
  // For proper timestamps, use NTP sync (recommended for production)
  unsigned long ms = millis();
  unsigned long totalSeconds = ms / 1000;
  unsigned long seconds = totalSeconds % 60;
  unsigned long minutes = (totalSeconds / 60) % 60;
  unsigned long hours = (totalSeconds / 3600) % 24;
  
  // Format: M/D/YYYY, H:MM:SS AM/PM
  String ampm = (hours >= 12) ? "PM" : "AM";
  int hour12 = hours % 12;
  if (hour12 == 0) hour12 = 12;
  
  char buffer[30];
  sprintf(buffer, "2/5/2026, %d:%02lu:%02lu %s", hour12, minutes, seconds, ampm.c_str());
  return String(buffer);
}

// Get backend URL (with mDNS or fallback IP)
String getBackendUrl() {
  return "http://" + backendHost + ":" + String(backendPort);
}

void sendLogToBackend(const char* eventType, const char* alertMessage, float temp, float hum, int gas) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[BACKEND] WiFi not connected - cannot log event");
    return;
  }
  
  // Prevent simultaneous HTTP calls
  if (httpBusy) {
    Serial.println("[BACKEND] HTTP busy - skipping this request");
    return;
  }
  httpBusy = true;
  httpBusySince = millis();

  HTTPClient http;
  String url = getBackendUrl() + "/api/logs";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(1000); // 1 second timeout - fast to prevent blocking

  // Build ESP URL based on static IP
  String espUrl = "http://" + WiFi.localIP().toString() + "/data";
  
  StaticJsonDocument<384> doc;  // Use static to avoid heap fragmentation
  doc["deviceId"] = DEVICE_ID;
  doc["labName"] = LAB_NAME;
  doc["espUrl"] = espUrl;
  doc["eventType"] = eventType;
  doc["alertMessage"] = alertMessage;
  doc["temperature"] = temp;
  doc["humidity"] = hum;
  doc["gas"] = gas;
  doc["timestamp"] = getTimestamp();
  
  String jsonBody;
  serializeJson(doc, jsonBody);
  
  int httpCode = http.POST(jsonBody);
  
  if (httpCode > 0) {
    Serial.printf("[BACKEND] ✓ Log sent, code: %d\n", httpCode);
  } else {
    Serial.printf("[BACKEND] ✗ Failed: %s\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();
  httpBusy = false;
  yield();  // Allow other tasks to run
}

// Send event to /api/events endpoint (simplified logging)
void sendEventToBackend(const char* eventType, const char* alertMessage) {
  if (WiFi.status() != WL_CONNECTED) return;
  if (httpBusy) return;  // Prevent collision
  httpBusy = true;
  httpBusySince = millis();

  HTTPClient http;
  String url = getBackendUrl() + "/api/events";
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(1000); // 1 second timeout - fast
  
  StaticJsonDocument<192> doc;  // Static to avoid heap fragmentation
  doc["deviceId"] = DEVICE_ID;
  doc["labName"] = LAB_NAME;
  doc["eventType"] = eventType;
  doc["alertMessage"] = alertMessage;
  
  String jsonBody;
  serializeJson(doc, jsonBody);
  
  int httpCode = http.POST(jsonBody);
  
  if (httpCode > 0) {
    Serial.printf("[BACKEND] Event sent, code: %d\n", httpCode);
  } else {
    Serial.printf("[BACKEND] Event failed: %s\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();
  httpBusy = false;
  yield();
}

// Send BFP Alert event
void sendBfpAlert(const char* phoneNumber) {
  if (WiFi.status() != WL_CONNECTED) return;
  if (httpBusy) return;  // Prevent collision
  httpBusy = true;
  httpBusySince = millis();

  HTTPClient http;
  String url = getBackendUrl() + "/api/logs";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(1000); // 1 second timeout - fast

  int gasValue = analogRead(MQ2_A0);
  
  StaticJsonDocument<384> doc;  // Static to avoid heap fragmentation
  doc["deviceId"] = DEVICE_ID;
  doc["labName"] = LAB_NAME;
  doc["eventType"] = "bfp_alert";
  doc["alertMessage"] = String("BFP Emergency dispatch sent to ") + phoneNumber;
  doc["temperature"] = dhtData.temperature;
  doc["humidity"] = dhtData.humidity;
  doc["gas"] = gasValue;
  doc["timestamp"] = getTimestamp();
  
  String jsonBody;
  serializeJson(doc, jsonBody);
  
  int httpCode = http.POST(jsonBody);
  
  if (httpCode > 0) {
    Serial.printf("[BACKEND] BFP alert sent, code: %d\n", httpCode);
  }
  
  http.end();
  httpBusy = false;
  yield();
}
// ==========================
// SSE (Server-Sent Events) — INSTANT SIREN CONTROL
// ==========================
// Opens a persistent connection to GET /api/siren-stream/<deviceId>.
// Server pushes sirenCommand changes instantly — zero polling delay.
// POST response sirenCommand is kept as a fallback.
void connectSSE() {
  if (sseClient.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.println("[SSE] Connecting to siren stream...");

  if (sseClient.connect(backendHost.c_str(), backendPort)) {
    // Send HTTP GET request for SSE
    String path = "/api/siren-stream/" + String(DEVICE_ID);
    sseClient.println("GET " + path + " HTTP/1.1");
    sseClient.println("Host: " + backendHost);
    sseClient.println("Accept: text/event-stream");
    sseClient.println("Connection: keep-alive");
    sseClient.println();

    sseConnected = true;
    sseBuffer = "";
    Serial.println("[SSE] Connected — listening for siren commands");
  } else {
    sseConnected = false;
    Serial.println("[SSE] Connection failed");
  }
}

void processSSEEvent(String event) {
  // Skip heartbeat comments (lines starting with ':')
  if (event.startsWith(":")) return;

  // Skip HTTP headers on first response
  if (event.indexOf("HTTP/1.1") >= 0) return;

  // Find "data: " prefix
  int dataIndex = event.indexOf("data: ");
  if (dataIndex < 0) return;

  String jsonStr = event.substring(dataIndex + 6);
  jsonStr.trim();

  // Parse JSON
  StaticJsonDocument<2048> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);
  if (error) {
    Serial.println("[SSE] JSON parse error: " + String(error.c_str()));
    Serial.println("[SSE] Raw data: " + jsonStr.substring(0, 200));
    return;
  }

  // Log the SSE event type for debugging
  String eventType = doc.containsKey("type") ? doc["type"].as<String>() : "(no type)";
  Serial.println("[SSE] Event received — type: " + eventType);

  // Handle siren command
  if (doc.containsKey("sirenCommand")) {
    bool command = doc["sirenCommand"];
    // Use current sensor readings for critical state
    int gasValue = analogRead(MQ2_A0);
    bool isCritical = fireDetected || (dhtData.temperature >= 42.0) || (gasValue >= 600);
    handleSirenCommandSSE(command, isCritical);
    Serial.println("[SSE] Siren command: " + String(command ? "ON" : "OFF"));
  }

  // Handle SMS command (manual siren ON or auto-activation) — QUEUED, non-blocking
  if (doc.containsKey("type") && doc["type"] == "send-sms") {
    String msg = doc["smsMessage"].as<String>();
    Serial.println("[SSE-SMS] Message: " + msg);

    if (doc.containsKey("alertContacts")) {
      JsonArray contacts = doc["alertContacts"];
      if (contacts.size() > 0) {
        for (JsonVariant contact : contacts) {
          String num = contact.as<String>();
          Serial.println("[SSE-SMS] Queueing to: " + num);
          queueSMS(num, msg);
        }
        Serial.println("📱 SSE: Queued SMS for " + String(contacts.size()) + " contacts");
      } else {
        Serial.println("⚠️ [SSE-SMS] alertContacts array is empty!");
      }
    } else {
      Serial.println("⚠️ [SSE-SMS] No alertContacts field in event!");
    }
  }

  // Handle BFP dispatch (SMS to fire department) — QUEUED, non-blocking
  if (doc.containsKey("type") && doc["type"] == "bfp-dispatch") {
    String phone = doc["phoneNumber"].as<String>();
    String msg = doc["message"].as<String>();
    Serial.println("🚨 [BFP] Dispatching to: " + phone);
    Serial.println("🚨 [BFP] Message: " + msg);
    queueSMS(phone, msg);
    Serial.println("🚨 BFP dispatch SMS queued for " + phone);
    // Log BFP dispatch to backend
    sendBfpAlert(phone.c_str());
  }
}

void handleSSE() {
  // Reconnect if disconnected
  if (!sseClient.connected()) {
    if (sseConnected) {
      sseConnected = false;
      Serial.println("[SSE] Disconnected");
    }
    unsigned long now = millis();
    if (now - lastSSEAttempt >= SSE_RECONNECT_INTERVAL) {
      lastSSEAttempt = now;
      connectSSE();
    }
    return;
  }

  // Read incoming SSE data (non-blocking)
  while (sseClient.available()) {
    char c = sseClient.read();
    if (c == '\r') continue;  // Strip \r — normalize to \n only
    sseBuffer += c;

    // SSE events end with double newline
    if (sseBuffer.endsWith("\n\n")) {
      processSSEEvent(sseBuffer);
      sseBuffer = "";
    }

    // Safety: prevent buffer overflow on garbage data
    if (sseBuffer.length() > 4096) {
      Serial.println("[SSE] Buffer overflow — clearing");
      sseBuffer = "";
    }
  }
}

// ==========================
// POST SENSOR DATA TO BACKEND (Push Architecture — Primary)
// ==========================
// ============================================
// SIREN STATE SYNC — Independent safety net
// ============================================
// Lightweight GET every 3s that bypasses httpBusy and SSE lock.
// If SSE is dead and POST is blocked, this catches mismatches.
unsigned long lastSirenSyncTime = 0;
const unsigned long SIREN_SYNC_INTERVAL = 3000;

void checkSirenSync() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastSirenSyncTime < SIREN_SYNC_INTERVAL) return;
  lastSirenSyncTime = millis();

  HTTPClient http;
  String url = getBackendUrl() + "/api/siren-state/" + String(DEVICE_ID);
  http.begin(url);
  http.setTimeout(2000);

  int httpCode = http.GET();
  if (httpCode == 200) {
    String response = http.getString();
    StaticJsonDocument<64> doc;
    if (!deserializeJson(doc, response)) {
      bool serverState = doc["sirenCommand"] | false;
      if (serverState != sirenActive) {
        Serial.printf("[SIREN-SYNC] Mismatch! Server=%s, Local=%s — correcting\n",
          serverState ? "ON" : "OFF", sirenActive ? "ON" : "OFF");
        applySirenState(serverState, false);
      }
    }
  }
  http.end();
}

// ESP32 pushes sensor data every 2s via HTTP POST.
// Backend receives → saves to DB → emits WebSocket to frontend.
// Backend watchdog marks device offline after 30s of no POST.
void postSensorDataToBackend() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HTTP] WiFi not connected, skipping POST");
    return;
  }

  // Safety: force-clear httpBusy if it's been stuck for too long
  if (httpBusy && (millis() - httpBusySince > HTTP_BUSY_TIMEOUT)) {
    Serial.println("[HTTP] ⚠️ httpBusy stuck for 10s — force-clearing");
    httpBusy = false;
  }

  // Prevent simultaneous HTTP calls
  if (httpBusy) {
    Serial.println("[HTTP] HTTP busy - skipping this request");
    return;
  }
  httpBusy = true;
  httpBusySince = millis();
  
  HTTPClient http;
  String url = getBackendUrl() + "/api/sensor-data";
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(3000); // 3 second timeout
  
  int gasValue = analogRead(MQ2_A0);
  
  // Create JSON payload matching backend expectations (static to avoid heap fragmentation)
  StaticJsonDocument<192> doc;
  doc["deviceId"] = DEVICE_ID;
  doc["temperature"] = isnan(dhtData.temperature) ? 0 : dhtData.temperature;
  doc["humidity"] = isnan(dhtData.humidity) ? 0 : dhtData.humidity;
  doc["gas"] = gasValue;
  doc["flame"] = fireDetected;
  
  String payload;
  serializeJson(doc, payload);
  
  int httpCode = http.POST(payload);
  
  if (httpCode == 200 || httpCode == 201) {
    lastSuccessfulPost = millis();
    String response = http.getString();
    Serial.printf("✅ [HTTP] Sensor data POSTed\n");
    
    // Parse sirenCommand + SMS fields from backend response
    StaticJsonDocument<2048> responseDoc;
    DeserializationError error = deserializeJson(responseDoc, response);
    
    if (!error) {
      // --- Siren command (existing) ---
      bool serverSirenCommand = responseDoc["sirenCommand"] | false;
      bool isCritical = fireDetected || (dhtData.temperature >= 42.0) || (gasValue >= 600);
      Serial.printf("[POST] sirenCommand=%s, sirenActive=%s\n", serverSirenCommand ? "ON" : "OFF", sirenActive ? "ON" : "OFF");
      handleSirenCommand(serverSirenCommand, isCritical);

      // --- Update alert contacts from response ---
      if (responseDoc.containsKey("alertContacts")) {
        JsonArray contacts = responseDoc["alertContacts"];
        PHONE_COUNT = 0;
        for (JsonVariant contact : contacts) {
          if (PHONE_COUNT < MAX_NUMBERS) {
            PHONE_NUMBERS[PHONE_COUNT] = contact.as<String>();
            PHONE_COUNT++;
          }
        }
        Serial.printf("📱 [POST] Alert contacts updated: %d numbers\n", PHONE_COUNT);
      }

      // --- SMS alert (server-controlled with local 10s cooldown) ---
      bool shouldSendSMS = responseDoc["sendSMS"] | false;
      Serial.printf("[POST] sendSMS=%s\n", shouldSendSMS ? "true" : "false");

      if (shouldSendSMS) {
        unsigned long now = millis();
        if (now - lastSMS >= SMS_COOLDOWN) {
          lastSMS = now;

          String smsMsg = responseDoc["smsMessage"].as<String>();
          Serial.println("[SMS] Message: " + smsMsg);

          // Queue SMS to all alert contacts (non-blocking — processed in loop())
          if (responseDoc.containsKey("alertContacts")) {
            JsonArray smsContacts = responseDoc["alertContacts"];
            if (smsContacts.size() > 0) {
              for (JsonVariant contact : smsContacts) {
                String num = contact.as<String>();
                Serial.println("[SMS] Queueing to: " + num);
                queueSMS(num, smsMsg);
              }
              Serial.println("📱 POST: Queued SMS for " + String(smsContacts.size()) + " contacts");
            } else {
              Serial.println("⚠️ [SMS] sendSMS=true but alertContacts is empty!");
            }
          } else {
            Serial.println("⚠️ [SMS] sendSMS=true but no alertContacts field in response!");
          }
        } else {
          Serial.printf("⏳ [SMS] Cooldown active (%lus remaining)\n", (SMS_COOLDOWN - (now - lastSMS)) / 1000);
        }
      }
    } else {
      Serial.println("⚠️  [HTTP] Failed to parse response JSON");
    }
  } else if (httpCode > 0) {
    Serial.printf("⚠️  [HTTP] Response: %d\n", httpCode);
  } else {
    Serial.printf("❌ [HTTP] Failed: %s\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();
  httpBusy = false;
  yield();  // Allow other tasks to run
}
// ==========================
// EEPROM CLEAN FUNCTION
// ==========================
void clearAllNumbersFromEEPROM() {
  char empty[32] = {0};
  for (int i = 0; i < MAX_NUMBERS; i++) {
    EEPROM.put(i * 32, empty);
  }
  EEPROM.commit();
}

// ==========================
// UPDATE PHONE NUMBERS
// ==========================
void handleSetNumbers() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Body missing");
    return;
  }

  DynamicJsonDocument doc(1024);  // 1KB — supports up to 10 phone numbers
  if (deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  if (!doc.containsKey("numbers")) {
    server.send(400, "text/plain", "Missing numbers");
    return;
  }

  // 🔥 CLEAR OLD NUMBERS FIRST
  clearAllNumbersFromEEPROM();

  JsonArray arr = doc["numbers"].as<JsonArray>();
  PHONE_COUNT = min((int)arr.size(), MAX_NUMBERS);

  for (int i = 0; i < PHONE_COUNT; i++) {
    PHONE_NUMBERS[i] = arr[i].as<String>();
    char buffer[32] = {0};
    PHONE_NUMBERS[i].toCharArray(buffer, 32);
    EEPROM.put(i * 32, buffer);
  }

  EEPROM.commit();
  server.send(200, "text/plain", "Numbers updated cleanly");
}

// ==========================
// ALERT NUMBERS ENDPOINT (from Backend)
// ==========================
// Receives phone numbers in +63 format from backend
// Format: {"contacts": ["+639171234567", "+639181234567"]}
void handleAlertNumbers() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"Body missing\"}");
    return;
  }

  DynamicJsonDocument doc(1024);  // 1KB — supports up to 10 contacts in +63 format
  if (deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  if (!doc.containsKey("contacts")) {
    server.send(400, "application/json", "{\"error\":\"Missing contacts array\"}");
    return;
  }

  // Clear existing numbers from EEPROM
  clearAllNumbersFromEEPROM();

  JsonArray contacts = doc["contacts"].as<JsonArray>();
  PHONE_COUNT = 0;

  // Store new numbers (already in +63 format)
  for (JsonVariant num : contacts) {
    if (PHONE_COUNT < MAX_NUMBERS) {
      PHONE_NUMBERS[PHONE_COUNT] = num.as<String>();
      
      // Save to EEPROM for persistence
      char buffer[32] = {0};
      PHONE_NUMBERS[PHONE_COUNT].toCharArray(buffer, 32);
      EEPROM.put(PHONE_COUNT * 32, buffer);
      
      Serial.printf("📱 Alert number [%d]: %s\n", PHONE_COUNT, PHONE_NUMBERS[PHONE_COUNT].c_str());
      PHONE_COUNT++;
    }
  }

  EEPROM.commit();
  
  Serial.printf("✅ Alert contacts updated: %d numbers\n", PHONE_COUNT);
  
  // Send success response
  StaticJsonDocument<64> response;
  response["success"] = true;
  response["count"] = PHONE_COUNT;
  String responseStr;
  serializeJson(response, responseStr);
  server.send(200, "application/json", responseStr);
}

// NOTE: handleGetData() removed — backend no longer polls ESP32.
// Sensor data is now pushed via HTTP POST every 2s (push architecture).

// Handle CORS preflight requests
void handleCorsOptions() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}

// ==========================
// APP ALARM ON/OFF — REMOVED
// ==========================
// Siren is now controlled exclusively by the backend server
// via the "sirenCommand" field in POST /api/sensor-data response.
// The local /siren endpoint is no longer needed.
// See ESP32_SIREN_GUIDE.md for details.

// ==========================
// SETUP
// ==========================
void setup() {
  // CRITICAL: Force siren relay OFF immediately — before ANY other init.
  // GPIO4 floats during boot and SIM800L init (~10s), which can
  // activate the relay. This must be the very first hardware action.
  pinMode(SIREN_PIN, INPUT);  // High-Z = guaranteed OFF for active-LOW relay

  Serial.begin(115200);
  Serial.println("\n========================================");
  Serial.println("ACES-3 Firmware v2.0 — 2026-03-16");
  Serial.println("RELAY_ACTIVE_HIGH = false");
  Serial.println("========================================");

  EEPROM.begin(512);
  PHONE_COUNT = 0;

  for (int i = 0; i < MAX_NUMBERS; i++) {
    char buffer[32] = {0};
    EEPROM.get(i * 32, buffer);
    String number = String(buffer);
    number.trim();
    if (number.length() > 0) {
      PHONE_NUMBERS[PHONE_COUNT++] = number;
    }
  }

  fireSensorSetup();
  dht.setup(DHT_PIN, DHTesp::DHT22);
  mq2Setup();

  sim800.begin(9600, SERIAL_8N1, 16, 17);
  delay(3000);  // SIM800L needs 3s+ to boot after power-on

  // Verify SIM800L is responding
  Serial.println("📱 Checking SIM800L...");
  bool sim800OK = false;
  for (int attempt = 0; attempt < 5; attempt++) {
    // Flush any leftover data
    while (sim800.available()) sim800.read();

    sim800.println("AT");
    delay(500);

    String atResponse = "";
    unsigned long waitStart = millis();
    while (millis() - waitStart < 1000) {
      while (sim800.available()) {
        atResponse += (char)sim800.read();
      }
    }
    Serial.println("[SIM800L] AT response: " + atResponse);

    if (atResponse.indexOf("OK") >= 0) {
      sim800OK = true;
      break;
    }
    Serial.printf("[SIM800L] Attempt %d/5 — no OK response, retrying...\n", attempt + 1);
    delay(1000);
  }

  if (sim800OK) {
    Serial.println("✅ SIM800L responding");

    // Set SMS text mode
    sim800.println("AT+CMGF=1");
    delay(500);
    while (sim800.available()) { Serial.print((char)sim800.read()); }
    Serial.println();

    // Check SIM card status
    sim800.println("AT+CPIN?");
    delay(500);
    String cpinResp = "";
    while (sim800.available()) { char c = sim800.read(); cpinResp += c; }
    Serial.println("[SIM800L] SIM status: " + cpinResp);
    if (cpinResp.indexOf("READY") >= 0) {
      Serial.println("✅ SIM card is ready");
    } else {
      Serial.println("⚠️ SIM card NOT ready — SMS will fail!");
    }

    // Check signal strength
    sim800.println("AT+CSQ");
    delay(500);
    String csqResp = "";
    while (sim800.available()) { char c = sim800.read(); csqResp += c; }
    Serial.println("[SIM800L] Signal: " + csqResp);

    // Check network registration
    sim800.println("AT+CREG?");
    delay(500);
    String cregResp = "";
    while (sim800.available()) { char c = sim800.read(); cregResp += c; }
    Serial.println("[SIM800L] Network: " + cregResp);

    Serial.println("📱 GSM module initialized");

    // ========== TEST SMS (uncomment to test) ==========
    // Change the phone number to your number and uncomment to test
    // String testNumber = "+639XXXXXXXXX";  // <-- PUT YOUR NUMBER HERE
    // Serial.println("📱 [TEST] Sending test SMS to: " + testNumber);
    // queueSMS(testNumber, "ACES-3 Test: SIM800L is working!");
    // ==================================================

  } else {
    Serial.println("❌ SIM800L NOT responding! Check wiring (TX→16, RX→17) and power supply.");
    Serial.println("   SMS features will NOT work until SIM800L is connected.");
  }

  oledSetup();
  oledBootMessageACES();

  sirenSetup();

  // ============================================
  // WIFI CONNECTION (DHCP — Huawei Pocket WiFi)
  // ============================================
  Serial.println("\n\n🔥 ACES IoT ESP32 Starting...");
  
  // Validate DEVICE_ID before anything else
  validateDeviceId();
  
  Serial.print("📟 Device ID (acesID): ");
  Serial.println(DEVICE_ID);
  Serial.print("🏷️  Lab Name: ");
  Serial.println(LAB_NAME);
  
  WiFi.setAutoReconnect(true);    // Auto-reconnect when WiFi drops
  WiFi.persistent(true);           // Remember WiFi credentials across reboots
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("📶 Connecting to Wi-Fi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("✅ WiFi connected");
    Serial.print("   IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("   Gateway: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("   Server: ");
    Serial.println(backendHost);
  } else {
    Serial.println("\n❌ WiFi failed — restarting...");
    ESP.restart();
  }

  // Backend is at fixed IP: 192.168.8.10:3000

  // Test backend connectivity
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("🔍 Testing backend health check...");
    HTTPClient http;
    http.begin(getBackendUrl() + "/api/health");
    http.setTimeout(3000);
    int code = http.GET();
    if (code == 200) {
      Serial.println("✅ Backend is reachable!");
    } else {
      Serial.printf("⚠️  Backend health check returned: %d\n", code);
    }
    http.end();
  }

  // Send device online log to backend
  sendLogToBackend("device_online", "Device came back online", 0, 0, 0);

  // Connect SSE for instant siren commands
  connectSSE();

  server.on("/set_numbers", HTTP_POST, handleSetNumbers);
  server.on("/alert-numbers", HTTP_POST, handleAlertNumbers);  // Backend sends +63 format
  // /data GET removed — backend no longer polls ESP32 (push architecture)
  // /siren endpoints removed — siren now controlled via sirenCommand
  // in POST /api/sensor-data response. See ESP32_SIREN_GUIDE.md

  server.begin();
  Serial.println("🌐 HTTP Server started on port 80");
  
  // Startup summary
  Serial.println("\n============================================");
  Serial.println("🚀 ACES IoT ESP32 Ready!");
  Serial.printf("📟 acesID: %s\n", DEVICE_ID);
  Serial.printf("🏷️  Lab: %s\n", LAB_NAME);
  Serial.printf("📍 IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("🖥️  Backend: %s:%d\n", backendHost.c_str(), backendPort);
  Serial.println("============================================\n");
}

// ==========================
// LOOP
// ==========================
void loop() {
  // Check for instant siren commands via SSE (non-blocking)
  handleSSE();

  // Process SMS queue (non-blocking — one step per iteration)
  processSMSQueue();

  fireDetected = checkFire();
  yield();  // Prevent WiFi stack blocking between sensor reads
  int gasValue = analogRead(MQ2_A0);
  yield();
  gasDetected = (gasValue > MQ2_threshold);

  if (millis() - lastDHTread >= 2000) {
    TempAndHumidity newData = dht.getTempAndHumidity();
    yield();
    if (!isnan(newData.temperature)) dhtData.temperature = newData.temperature;
    if (!isnan(newData.humidity)) dhtData.humidity = newData.humidity;
    lastDHTread = millis();
  }

  // Immediate POST on fire detection — don't wait for interval
  if (fireDetected && !lastFlameState) {
    postSensorDataToBackend();
    Serial.println("🔥 FIRE DETECTED — immediate POST sent!");
  }
  // Immediate POST when fire clears — update UI instantly
  if (!fireDetected && lastFlameState) {
    postSensorDataToBackend();
    Serial.println("✅ FIRE CLEARED — immediate POST sent!");
  }
  lastFlameState = fireDetected;

  bool highTemp = (dhtData.temperature >= 42.0);

  // 1. POST FIRST — dynamic interval: 500ms when siren active, 2s when idle
  //    (non-blocking: 1s timeout, no delay)
  if (millis() - lastHttpPost > getPostInterval()) {
    postSensorDataToBackend();
    lastHttpPost = millis();
  }
  yield();

  // 1b. SIREN STATE SYNC — independent safety net (every 3s, bypasses httpBusy)
  if (!httpBusy) {
    checkSirenSync();
  }
  yield();

  // 2. THEN handle local actions (OLED, siren, SMS)

  oledShow(
    isnan(dhtData.temperature) ? 0 : dhtData.temperature,
    isnan(dhtData.humidity) ? 0 : dhtData.humidity,
    gasValue,
    fireDetected || gasDetected || highTemp || sirenActive
  );

  // Siren is controlled by the backend via sirenCommand in POST response.
  // No local siren activation logic — see ESP32_SIREN_GUIDE.md
  bool automaticAlert = fireDetected || gasDetected || highTemp;

  // ---------------------------
  // AUTOMATIC ALERT SMS — NOW SERVER-CONTROLLED
  // ---------------------------
  // SMS is now triggered by the backend via sendSMS field in POST response.
  // The server handles 10-second cooldown between repeated alerts.
  // See ESP32_SIREN_GUIDE_SMS.md for details.

  // ---------------------------
  // BACKEND EVENT LOGGING (skip during siren to prevent freeze)
  // ---------------------------
  bool isWarning = (dhtData.temperature >= 35.0 && dhtData.temperature < 42.0) || 
                   (gasValue >= 400 && gasValue < 600) || 
                   (dhtData.humidity >= 75);
  bool isCritical = (dhtData.temperature >= 42.0) || (gasValue >= 600) || fireDetected;

  if (!sirenActive && automaticAlert && millis() - lastBackendLog > BACKEND_LOG_COOLDOWN) {
    // Log CRITICAL events (fire, extreme temp, high gas)
    if (isCritical && !lastCriticalState) {
      String critMsg = "";
      if (fireDetected) {
        critMsg = "FIRE DETECTED - Emergency alert!";
      } else if (gasValue >= 600) {
        critMsg = "Critical gas level detected - " + String(gasValue) + " ppm";
      } else if (dhtData.temperature >= 42.0) {
        critMsg = "Critical temperature detected - " + String(dhtData.temperature, 1) + "°C";
      }
      sendLogToBackend("critical", critMsg.c_str(), dhtData.temperature, dhtData.humidity, gasValue);
      lastBackendLog = millis();
    }
    // Log WARNING events (elevated readings)
    else if (isWarning && !isCritical && !lastWarningState) {
      String warnMsg = "";
      if (dhtData.temperature >= 35.0) {
        warnMsg = "High Temperature Warning - " + String(dhtData.temperature, 1) + "°C detected";
      } else if (gasValue >= 400) {
        warnMsg = "Elevated gas level warning - " + String(gasValue) + " ppm";
      } else if (dhtData.humidity >= 75) {
        warnMsg = "High humidity warning - " + String(dhtData.humidity, 0) + "%";
      }
      sendLogToBackend("warning", warnMsg.c_str(), dhtData.temperature, dhtData.humidity, gasValue);
      lastBackendLog = millis();
    }
  }
  
  lastWarningState = isWarning;
  lastCriticalState = isCritical;

  yield();  // Prevent watchdog timeout
  
  server.handleClient();
  yield();  // Prevent watchdog timeout

  // Monitor WiFi connection and auto-reconnect if disconnected
  static bool wasConnected = true;
  static unsigned long lastReconnectAttempt = 0;
  if (WiFi.status() != WL_CONNECTED) {
    if (wasConnected) {
      wasConnected = false;
      Serial.println("[WIFI] Connection lost!");
    }
    // Try to reconnect every 5 seconds
    if (millis() - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = millis();
      Serial.println("[WIFI] Attempting reconnect...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  } else if (!wasConnected) {
    wasConnected = true;
    Serial.println("[WIFI] Reconnected!");
    Serial.print("📍 IP Address: ");
    Serial.println(WiFi.localIP());
    sendEventToBackend("device_online", "Device reconnected after connection loss");
  }

  // Self-recovery: if WiFi says connected but no POST has succeeded in 60s,
  // the TCP stack is likely hung. Force a full WiFi reset.
  if (WiFi.status() == WL_CONNECTED && lastSuccessfulPost > 0 &&
      (millis() - lastSuccessfulPost > HTTP_RECOVERY_TIMEOUT)) {
    Serial.println("[RECOVERY] ⚠️ No successful POST in 60s — resetting WiFi stack");
    lastSuccessfulPost = millis(); // Reset timer to avoid rapid retries
    httpBusy = false;
    sseClient.stop();
    sseConnected = false;
    WiFi.disconnect(true);
    delay(1000);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}
