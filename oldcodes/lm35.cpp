#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

#define LM35_PIN A0    // LM35 analog input pin

void setup() {
  Serial.begin(19200);
  analogReference(DEFAULT);  // Use default (5V) as reference
}

void loop() {
  // Read analog value and convert to temperature
  int rawValue = analogRead(LM35_PIN);
  float voltage = (rawValue * 5.0) / 1024.0;  // Convert to voltage
  float tempC = voltage * 100.0;  // LM35 gives 10mV per degree Celsius

  // Print temperature
  Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.println("°C");

  delay(1000);  // Wait 1 second between readings
}