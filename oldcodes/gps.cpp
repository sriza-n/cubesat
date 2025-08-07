#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>

#define GPS_RX 4
#define GPS_TX 5

TinyGPS gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
// unsigned long lastDebugTime = 0;
// unsigned long chars = 0;
// unsigned short sentences = 0;
// unsigned short failed = 0;

float lat = 0.0, lon = 0.0;
unsigned long lastGPSCheck = 0;
bool gpsDataReceived = false;

void setup() {
  Serial.begin(115200);  // Changed to standard baud rate
  gpsSerial.begin(9600);  // Most GPS modules use 9600 baud
  Serial.println("GPS Test Start");
}

void loop() {
  
  
  // while (gpsSerial.available()) {
  //   char c = gpsSerial.read();
  //   if (gps.encode(c)) {
  //     // Get position data
  //     // float lat, lon;
  //     unsigned long age;
  //     gps.f_get_position(&lat, &lon, &age);

  
  //       // Print data
  //       // Serial.print("Location: ");
  //       // Serial.print(lat, 6);
  //       // Serial.print(",");
  //       // Serial.println(lon, 6);
  //       // delay(100);
  //   }

    
  // }
    // GPS Data
    bool newData = false;
    unsigned long start = millis();
    
    // Try to collect data for up to 1 second
    while (millis() - start < 1000) {
      while (gpsSerial.available()) {
        char c = gpsSerial.read();
        // Debug - print raw data
        Serial.write(c);
        if (gps.encode(c)) {
          newData = true;
          gpsDataReceived = true;
        }
      }
    }

  // Serial.print("Location: ");
  // Serial.print(lat, 6);
  // Serial.print(",");
  // Serial.println(lon, 6);
  if (newData) {
    gps.f_get_position(&lat, &lon);
    
    // Print satellite data - FIXED: satellites() returns a value, doesn't take parameters
    unsigned short satellites = gps.satellites();
    
    // Display position and satellite info
    Serial.print("Location: ");
    Serial.print(lat, 6);
    Serial.print(",");
    Serial.println(lon, 6);
    Serial.print("Satellites: ");
    Serial.println(satellites);
  }
  delay(100);
  
}