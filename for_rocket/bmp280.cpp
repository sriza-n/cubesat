#include <Wire.h>
#include <Adafruit_BMP280.h>

// Create sensor object
Adafruit_BMP280 bmp;

// Sea level pressure in hPa (customize for your location for better accuracy)
float seaLevelPressure = 1011.3;

void setup() {
  Serial.begin(115200);
  
  // Wait until serial port opens
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("BMP280 Altitude Sensor Test");
  
  // Initialize sensor with specific I2C address
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring or try a different address!");
    while (1);
  }
  
  // Default settings from datasheet
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode
                  Adafruit_BMP280::SAMPLING_X2,      // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,     // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,       // Filtering
                  Adafruit_BMP280::STANDBY_MS_1);  // Standby time
}

void loop() {
  // Read sensor data
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude(seaLevelPressure);
  
  // Print readings
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  
  Serial.print("Pressure: ");
  Serial.print(pressure / 100.0);
  Serial.println(" hPa");
  
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");
  
  Serial.println();
  delay(100);
}