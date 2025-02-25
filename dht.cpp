#include <Arduino.h>
#include "DHT.h"
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

#define DHTPIN 7     // DHT22 data pin
#define DHTTYPE DHT22   

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(19200);
  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements
  delay(2000);

  // Reading temperature and humidity
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Check if any reads failed
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Print readings in CSV format
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%,");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");
}