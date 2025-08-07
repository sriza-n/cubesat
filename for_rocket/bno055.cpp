#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections for ESP32
   ===========
   Connect SCL to GPIO 22
   Connect SDA to GPIO 21
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   GPS NEO-8M connections to ESP32:
   GPS TX -> ESP32 RX2 (GPIO 16)
   GPS RX -> ESP32 TX2 (GPIO 17)
   
   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;// 20ms for 50Hz
uint16_t GPS_SAMPLERATE_DELAY_MS = 1000; // GPS readings at 1Hz (1000ms)

// ESP32 I2C pins
#define I2C_SDA 21
#define I2C_SCL 22

// Timing variables for non-blocking sensor reading
unsigned long lastIMUReadTime = 0;
unsigned long lastGPSReadTime = 0;
unsigned long previousIMUReadTime = 0; // To calculate dt between IMU readings

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
void printEvent(sensors_event_t* event);
void printQuaternion(imu::Quaternion quat);

// GPS setup
TinyGPSPlus gps;
// ESP32 has hardware serial ports - using Serial2 for GPS
#define GPS_BAUDRATE 9600
void displayGPSInfo();

void setup(void)
{
  Serial.begin(115200);  // Serial for debug output
  
  // Initialize I2C bus for ESP32
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Initialize GPS on hardware Serial2 (default pins 16,17)
  Serial2.begin(GPS_BAUDRATE);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("ESP32 Orientation Sensor and GPS Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  // bno.setMode(); // Fusion mode with maximum filtering
  bno.setMode((adafruit_bno055_opmode_t)12); // Fusion mode with maximum filtering

  delay(1000);
}

void loop(void)
{
  unsigned long currentMillis = millis();
  
  // Process GPS data at GPS sample rate
  if (currentMillis - lastGPSReadTime >= GPS_SAMPLERATE_DELAY_MS) {
    lastGPSReadTime = currentMillis;
    
    // Process any available GPS data
    bool newGpsDataAvailable = false;
    while (Serial2.available() > 0) {
      if (gps.encode(Serial2.read())) {
        newGpsDataAvailable = true;
      }
    }
    
    // Only display GPS info once per sampling period
    if (newGpsDataAvailable) {
      displayGPSInfo();
    }
    
    // Check if GPS data isn't coming in after 5 seconds
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println("No GPS detected");
    }
  }

  // Read IMU data at IMU sample rate
  if (currentMillis - lastIMUReadTime >= BNO055_SAMPLERATE_DELAY_MS) {
    // Calculate dt (milliseconds since last IMU reading)
    unsigned long dt = lastIMUReadTime > 0 ? currentMillis - previousIMUReadTime : 0;
    previousIMUReadTime = currentMillis;
    lastIMUReadTime = currentMillis;
    
    // Print dt value
    Serial.print("IMU dt: ");
    Serial.print(dt);
    Serial.println(" ms");
    
    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    // Get quaternion data
    imu::Quaternion quat = bno.getQuat();
    printQuaternion(quat);

    printEvent(&orientationData);
    printEvent(&angVelocityData);
    printEvent(&linearAccelData);
    printEvent(&magnetometerData);
    printEvent(&accelerometerData);
    // printEvent(&gravityData);

    int8_t boardTemp = bno.getTemp();
    Serial.println();
    Serial.print(F("temperature: "));
    Serial.println(boardTemp);

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.println();
    Serial.print("Calibration: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);

    Serial.println("--");
  }
  
  // No delay here - loop runs continuously to check for timing conditions
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  // else if (event->type == SENSOR_TYPE_GRAVITY) {
  //   Serial.print("Gravity:");
  //   x = event->acceleration.x;
  //   y = event->acceleration.y;
  //   z = event->acceleration.z;
  // }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx=");
  Serial.print(x);
  
  // Add units to the output based on sensor type
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print(" m/s²");
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print(" deg");
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print(" µT");
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print(" rad/s");
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print(" rad/s");
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print(" m/s²");
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print(" m/s²");
  }
  
  Serial.print("\ty=");
  Serial.print(y);
  
  // Repeat units for y-axis
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print(" m/s²");
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print(" deg");
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print(" µT");
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print(" rad/s");
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print(" rad/s");
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print(" m/s²");
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print(" m/s²");
  }
  
  Serial.print("\tz=");
  Serial.print(z);
  
  // Repeat units for z-axis
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.println(" m/s²");
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.println(" deg");
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.println(" µT");
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.println(" rad/s");
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.println(" rad/s");
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.println(" m/s²");
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.println(" m/s²");
  }
  else {
    Serial.println();
  }
}

void printQuaternion(imu::Quaternion quat) {
  Serial.print("Quat:");
  Serial.print("\tw=");
  Serial.print(quat.w(), 4);
  Serial.print("\tx=");
  Serial.print(quat.x(), 4);
  Serial.print("\ty=");
  Serial.print(quat.y(), 4);
  Serial.print("\tz=");
  Serial.println(quat.z(), 4);
}

void displayGPSInfo() {
  Serial.println("GPS Data:");
  if (gps.location.isValid()) {
    Serial.print("Lat: "); 
    Serial.print(gps.location.lat(), 6);
    Serial.print("\tLng: "); 
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Location: Invalid");
  }
  
  if (gps.altitude.isValid()) {
    Serial.print("Alt: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" m");
  }
  
  if (gps.speed.isValid()) {
    Serial.print("Speed: ");
    Serial.print(gps.speed.kmph());
    Serial.println(" km/h");
  }
  
  if (gps.time.isValid()) {
    Serial.print("Time: ");
    if (gps.time.hour() < 10) Serial.print("0");
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print("0");
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print("0");
    Serial.println(gps.time.second());
  }
  
  if (gps.satellites.isValid()) {
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
  }
  
  Serial.println();
}


