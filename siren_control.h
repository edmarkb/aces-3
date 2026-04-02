#ifndef SIREN_CONTROL_H
#define SIREN_CONTROL_H

// ============================================
// SERVER-COMMANDED SIREN CONTROL
// ============================================
// The backend is the source of truth — siren state comes from
// the "sirenCommand" field in the POST /api/sensor-data response
// AND from SSE (Server-Sent Events) for instant commands.
//
// SSE commands have PRIORITY over POST responses because POST
// responses can be stale (computed before the user's action).
// A 3-second lock prevents POST from overriding a recent SSE command.
//
// See ESP32_SIREN_GUIDE.md for full details.
// ============================================

#define SIREN_PIN 4   // GPIO4 → Relay IN1 (changed from GPIO5 — GPIO5 is a strapping pin)

// ============================================
// RELAY POLARITY — Set this to match YOUR relay module!
// ============================================
// Some relay modules are active LOW  (LOW = relay ON)  → set false
// Some relay modules are active HIGH (HIGH = relay ON) → set true
//
// ACES-3: Active LOW relay.
#define RELAY_ACTIVE_HIGH false

// Helpers — abstract away polarity so the rest of the code is clean
#define RELAY_ON  (RELAY_ACTIVE_HIGH ? HIGH : LOW)
#define RELAY_OFF (RELAY_ACTIVE_HIGH ? LOW  : HIGH)

// Auto-siren cooldown settings
#define SIREN_COOLDOWN_TESTING    30000    // 30 seconds  (testing)
#define SIREN_COOLDOWN_PRODUCTION 180000   // 3 minutes   (production)
#define USE_PRODUCTION_COOLDOWN   false    // false = 30s, true = 3 min

// SSE priority lock duration (ms)
// After an SSE command, POST responses are IGNORED for this duration.
// This prevents stale POST responses (sent before user's action) from
// overriding the SSE command.
#define SSE_LOCK_DURATION_MS 3000

bool sirenActive = false;               // Current relay state
bool sirenManualOff = false;            // User manually overrode siren OFF
bool prevCriticalState = false;         // Track critical transitions (for manual-off clearing)
unsigned long lastCriticalTime = 0;     // Last time critical conditions seen
unsigned long lastSSECommandTime = 0;   // When last SSE command was received
bool lastSSECommandValue = false;       // What the last SSE command was (ON/OFF)

unsigned long getSirenCooldown() {
  return USE_PRODUCTION_COOLDOWN ? SIREN_COOLDOWN_PRODUCTION : SIREN_COOLDOWN_TESTING;
}

void sirenSetup() {
  // HARDWARE FIX for 3.3V ESP32 vs 5V Relay issue:
  // For Active LOW relays powered by 5V, the ESP32's 3.3V "HIGH" signal
  // is sometimes not high enough to shut off the relay (5V - 3.3V = 1.7V leakage).
  // The solution is to set the pin to INPUT (High-Impedance) which physically 
  // breaks the circuit and guarantees the relay de-energizes.
  
  if (RELAY_ACTIVE_HIGH == false) {
    pinMode(SIREN_PIN, INPUT); // High-Z state = completely OFF for Active LOW
  } else {
    digitalWrite(SIREN_PIN, RELAY_OFF);
    pinMode(SIREN_PIN, OUTPUT);
  }
}

// Apply the siren state change to hardware
void applySirenState(bool shouldBeActive, bool isCritical) {
  // Only clear manual-off on a NEW critical event (safe → critical transition).
  if (isCritical && !prevCriticalState) {
    sirenManualOff = false;
    Serial.println("[SIREN] New critical event — manual override cleared");
  }
  if (isCritical) {
    lastCriticalTime = millis();
  }
  prevCriticalState = isCritical;

  // --- TURN ON ---
  if (shouldBeActive && !sirenActive && !sirenManualOff) {
    if (RELAY_ACTIVE_HIGH == false) {
      pinMode(SIREN_PIN, OUTPUT); // Enable output
    }
    digitalWrite(SIREN_PIN, RELAY_ON);
    sirenActive = true;
    Serial.println("[SIREN] ON (server command)");
  }
  // --- BLOCKED BY MANUAL OFF ---
  else if (shouldBeActive && !sirenActive && sirenManualOff) {
    // Server says ON but user manually silenced — stay off.
  }
  // --- TURN OFF ---
  else if (!shouldBeActive && sirenActive) {
    if (RELAY_ACTIVE_HIGH == false) {
      pinMode(SIREN_PIN, INPUT); // High-Z mode guaranteed OFF for 5V active-low relay
    } else {
      digitalWrite(SIREN_PIN, RELAY_OFF);
    }
    sirenActive = false;
    sirenManualOff = true;
    Serial.println("[SIREN] OFF (server command / manual override) - hardware High-Z triggered");
  }
  // --- ALREADY OFF, server confirms OFF — clear manual flag ---
  else if (!shouldBeActive && !sirenActive && sirenManualOff) {
    sirenManualOff = false;
  }
}

// Called from SSE handler — INSTANT, takes priority over POST
void handleSirenCommandSSE(bool shouldBeActive, bool isCritical) {
  lastSSECommandTime = millis();
  lastSSECommandValue = shouldBeActive;
  Serial.printf("[SIREN-SSE] Command: %s (lock set for %dms)\n",
    shouldBeActive ? "ON" : "OFF", SSE_LOCK_DURATION_MS);
  applySirenState(shouldBeActive, isCritical);
}

// Called from POST response — DEFERRED, can be overridden by SSE
void handleSirenCommand(bool shouldBeActive, bool isCritical) {
  // If an SSE command was received recently, POST responses are IGNORED
  // unless they agree with the SSE command. This prevents stale POST
  // responses from overriding the user's manual action.
  if (lastSSECommandTime > 0 && (millis() - lastSSECommandTime < SSE_LOCK_DURATION_MS)) {
    if (shouldBeActive != lastSSECommandValue) {
      Serial.printf("[SIREN-POST] IGNORED stale POST (sirenCommand=%s) — SSE lock active (%lums ago)\n",
        shouldBeActive ? "ON" : "OFF", millis() - lastSSECommandTime);
      return;  // Ignore this stale POST response
    }
  }
  applySirenState(shouldBeActive, isCritical);
}

#endif
