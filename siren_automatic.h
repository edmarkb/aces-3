#ifndef SIREN_AUTOMATIC_H
#define SIREN_AUTOMATIC_H

// ============================================
// AUTOMATIC SIREN LOGIC — REMOVED
// ============================================
// Siren is now controlled exclusively by the backend server
// via the "sirenCommand" field in the POST /api/sensor-data response.
//
// The ESP32 no longer independently activates the siren based
// on local sensor readings. The backend detects critical conditions
// (fire, temp >= 42°C, gas >= 600 ppm) and sends sirenCommand: true.
//
// See ESP32_SIREN_GUIDE.md for full details.
// ============================================

#endif
