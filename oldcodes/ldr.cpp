#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

#define LDR_PIN A1    // LDR analog input pin
#define LED_PIN 13    // Optional LED indicator

void setup() {
  Serial.begin(19200);
  pinMode(LDR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Read LDR value (0-1023)
  int rawValue = analogRead(LDR_PIN);
  
  // Convert to percentage (0-100%)
  int lightPercent = map(rawValue, 0, 1023, 0, 100);
  
  // Print values
  Serial.print("Raw Value: ");
  Serial.print(rawValue);
  Serial.print(" | Light Level: ");
  Serial.print(lightPercent);
  Serial.println("%");
  
  // Optional: LED brightness control
  analogWrite(LED_PIN, map(rawValue, 0, 1023, 255, 0));
  
  delay(1000);  // Wait 1 second between readings
}