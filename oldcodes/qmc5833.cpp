#include <Arduino.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
 #include <SPI.h>

#define QMC5883L_ADDR 0x0D

QMC5883LCompass compass;

void setup() {
  Wire.begin();
  Serial.begin(115200); 
  compass.init();
  compass.setMode(0x01, 0x0C, 0x10, 0x00);  // Continuous mode, 200Hz, 8G, 512 oversampling
}

void loop() {
  int x, y, z;
  
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);
  
  delay(100);  // Read every 100ms
}
// -----------------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_QMC5883.h>
#include <SPI.h>

#define QMC5883L_ADDR 0x1E

DFRobot_QMC5883 compass(&Wire, QMC5883L_ADDR);

void setup() {
  Serial.begin(115200);
  while (!compass.begin()) {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
  }
  delay(1000);
}

void loop() {
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  Serial.print("X:");
  Serial.print(mag.XAxis);
  Serial.print(" Y:");
  Serial.print(mag.YAxis);
  Serial.print(" Z:");
  Serial.println(mag.ZAxis);
  Serial.print("Degrees = ");
  Serial.println(mag.HeadingDegress);
  delay(1000);
}