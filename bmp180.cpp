#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
 #include <SPI.h>

#define BMP180_I2C_ADDRESS 0x77

Adafruit_BMP085 bmp;

void setup() {
  Wire.begin();  // Initialize I2C
  Serial.begin(19200);
  
  // Check if BMP180 is connected
  Wire.beginTransmission(BMP180_I2C_ADDRESS);
  byte error = Wire.endTransmission();
  
  if (error != 0 || !bmp.begin()) {
    Serial.print("BMP180 not found at 0x");
    Serial.println(BMP180_I2C_ADDRESS, HEX);
    while(1) {}
  }
}

void loop() {
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F;
  float altitude = bmp.readAltitude();

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Pressure: ");
  Serial.print(pressure);
  Serial.print(" hPa, Altitude: ");
  Serial.print(altitude);
  Serial.println(" meters");

  delay(2000);
}


// ---------------------------------------------------------------

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

// Create sensor instance
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

void setup() {
  Serial.begin(9600);
  
  // Initialize sensor
  if (!bmp.begin()) {
    Serial.println("Could not find BMP085 sensor");
    // while (1);
  }
}

void loop() {
  float pressure, temperature, altitude;
  sensors_event_t event;
  
  // Get pressure reading
  bmp.getEvent(&event);
  pressure = event.pressure;
  
  // Get temperature
  bmp.getTemperature(&temperature);
  
  // Calculate altitude
  altitude = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure);
  
  // Print readings
  Serial.print("Pressure: "); 
  Serial.print(pressure); 
  Serial.println(" hPa");
  
  Serial.print("Temperature: "); 
  Serial.print(temperature); 
  Serial.println(" C");
  
  Serial.print("Altitude: "); 
  Serial.print(altitude); 
  Serial.println(" m");
  
  Serial.println();
  delay(1000);
}