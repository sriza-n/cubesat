// address 0x68

#include <Arduino.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu(0x68);  // AD0 low = 0x68, AD0 high = 0x69

void setup() {
  Wire.begin();
  Serial.begin(19200);

  // Initialize MPU6050
  mpu.initialize();
  
  // Verify connection
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while(1);
  }
  
  // Set full scale ranges
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  // Read raw accel/gyro measurements
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Display values
  Serial.print("Accel X/Y/Z: ");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.println(az);
  Serial.print("Gyro X/Y/Z: ");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
  
  delay(100);
}