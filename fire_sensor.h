#ifndef FIRE_SENSOR_H
#define FIRE_SENSOR_H

// ---- 5 Flame Sensor Pins ----
int flameDO[5] = {32, 33, 25, 26, 27};

// ---- Setup All Sensors ----
void fireSensorSetup() {
  for (int i = 0; i < 5; i++) {
    pinMode(flameDO[i], INPUT);
  }
}

// ---- Check One Sensor ----
bool checkFireSingle(int id) {
  return (digitalRead(flameDO[id]) == LOW);
}

// ---- Check ANY Fire ----
bool checkFire() {
  for (int i = 0; i < 5; i++) {
    if (digitalRead(flameDO[i]) == LOW) return true;
  }
  return false;
}

#endif
