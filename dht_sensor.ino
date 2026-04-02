extern DHTesp dht;  // use the same object from main file

void readDHT() {
  TempAndHumidity data = dht.getTempAndHumidity();

  Serial.print("Temp: ");
  Serial.print(data.temperature);
  Serial.print(" °C   Humidity: ");
  Serial.print(data.humidity);
  Serial.println(" %");

  // You can add conditions here
  // Example: if temperature > 50 degrees, send alert
  /*
  if (data.temperature > 50) {
    // sendSMSHighTemp();
  }
  */
}
