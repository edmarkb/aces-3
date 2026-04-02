// MQ-2 Gas Sensor
int MQ2_A0 = 34;           // Analog pin
int MQ2_threshold = 300;   // Gas threshold

void mq2Setup() {
  // No special setup required for analog input
  pinMode(MQ2_A0, INPUT);
}

void mq2Loop() {
  int gasValue = analogRead(MQ2_A0);
  Serial.print("MQ-2 Gas Value: ");
  Serial.println(gasValue);
}
