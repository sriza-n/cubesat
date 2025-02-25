#include <Arduino.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
 #include <SPI.h>

#define QMC5883L_ADDR 0x1E

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