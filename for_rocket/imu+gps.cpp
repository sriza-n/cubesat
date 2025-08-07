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
uint16_t GPS_SAMPLERATE_DELAY_MS = 250; // GPS readings at 1Hz (1000ms)

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

// Add global variables to store latest GPS data
bool gpsDataValid = false;
unsigned long lastGpsTimestamp = 0;

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
void printCombinedJSON(sensors_event_t* orientationData, sensors_event_t* angVelocityData, 
                      sensors_event_t* linearAccelData, sensors_event_t* magnetometerData, 
                      sensors_event_t* accelerometerData, sensors_event_t* gravityData,
                      imu::Quaternion quat, int8_t temp, uint8_t sys, uint8_t gyro, 
                      uint8_t accel, uint8_t mag, unsigned long dt);

void loop(void)
{
  unsigned long currentMillis = millis();
  
  // Process GPS data at GPS sample rate
  if (currentMillis - lastGPSReadTime >= GPS_SAMPLERATE_DELAY_MS) {
    lastGPSReadTime = currentMillis;
    
    // Process any available GPS data
    while (Serial2.available() > 0) {
      if (gps.encode(Serial2.read())) {
        gpsDataValid = true;
        lastGpsTimestamp = currentMillis;
      }
    }
    
    // Check if GPS data isn't coming in after 5 seconds
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println("{\"error\":\"No GPS detected\"}");
    }
  }

  // Read IMU data at IMU sample rate
  if (currentMillis - lastIMUReadTime >= BNO055_SAMPLERATE_DELAY_MS) {
    // Calculate dt (milliseconds since last IMU reading)
    unsigned long dt = lastIMUReadTime > 0 ? currentMillis - previousIMUReadTime : 0;
    previousIMUReadTime = currentMillis;
    lastIMUReadTime = currentMillis;
    
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    // Get quaternion data
    imu::Quaternion quat = bno.getQuat();
    
    int8_t boardTemp = bno.getTemp();

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    
    // Print combined IMU and GPS data in a single JSON
    printCombinedJSON(&orientationData, &angVelocityData, &linearAccelData, 
                    &magnetometerData, &accelerometerData, &gravityData,
                    quat, boardTemp, system, gyro, accel, mag, dt);
  }
  
  // No delay here - loop runs continuously to check for timing conditions
}

// New function to print combined IMU and GPS data in a single line JSON format
void printCombinedJSON(sensors_event_t* orientationData, sensors_event_t* angVelocityData, 
                      sensors_event_t* linearAccelData, sensors_event_t* magnetometerData, 
                      sensors_event_t* accelerometerData, sensors_event_t* gravityData,
                      imu::Quaternion quat, int8_t temp, uint8_t sys, uint8_t gyro, 
                      uint8_t accel, uint8_t mag, unsigned long dt) {
  
  // unsigned long timestamp = millis();
  
  // Print the entire JSON on a single line
  // Serial.print("{\"timestamp\":");
  // Serial.print(timestamp);
  // Serial.print(",\"dt_ms\":");
  // Serial.print(dt);
  
  // IMU Section
  Serial.print("\"imu\":{");
  
  // Quaternion data
  Serial.print("\"quaternion\":");
  Serial.print(quat.w());
  Serial.print(",");
  Serial.print(quat.x());
  Serial.print(",");
  Serial.print(quat.y());
  Serial.print(",");
  Serial.print(quat.z());
  
  // Gyroscope data
  Serial.print(",\"gyroscope\":");
  Serial.print(angVelocityData->gyro.x,3);
  Serial.print(",");
  Serial.print(angVelocityData->gyro.y,3);
  Serial.print(",");
  Serial.print(angVelocityData->gyro.z,3);
  Serial.print(",\"units\":\"rad/s\"");
  
  // Accelerometer data
  // Serial.print(",\"accelerometer\":");
  // Serial.print(accelerometerData->acceleration.x, 6);
  // Serial.print(",");
  // Serial.print(accelerometerData->acceleration.y, 6);
  // Serial.print(",");
  // Serial.print(accelerometerData->acceleration.z, 6);
  // Serial.print(",\"units\":\"m/s²\"}");
  
  // Linear acceleration data
  Serial.print(",\"linear_acceleration\":");
  Serial.print(linearAccelData->acceleration.x, 3);
  Serial.print(",");
  Serial.print(linearAccelData->acceleration.y, 3);
  Serial.print(",");
  Serial.print(linearAccelData->acceleration.z, 3);
  Serial.print(",\"units\":\"m/s²\"");
  
  // Magnetometer data
  Serial.print(",\"magnetometer\":");
  Serial.print(magnetometerData->magnetic.x,3);
  Serial.print(",");
  Serial.print(magnetometerData->magnetic.y,3);
  Serial.print(",");
  Serial.print(magnetometerData->magnetic.z,3);
  Serial.print(",\"units\":\"µT\"");
  
  // Orientation data
  Serial.print(",\"orientation\":");
  Serial.print(orientationData->orientation.x,3);
  Serial.print(",");
  Serial.print(orientationData->orientation.y,3);
  Serial.print(",");
  Serial.print(orientationData->orientation.z,3);
  Serial.print(",\"units\":\"degrees\"");
  
  // Calibration status /euler orientation
  Serial.print(",\"calibration\":");
  Serial.print(sys);
  Serial.print(",\"gyro\":");
  Serial.print(gyro);
  Serial.print(",\"accel\":");
  Serial.print(accel);
  Serial.print(",\"mag\":");
  Serial.print(mag);
  
  // Temperature
  Serial.print(",\"temperature\":");
  Serial.print(temp);
  Serial.print(",\"units\":\"celsius\"");
  
  Serial.print("}");
  
  // GPS Section - Fix the JSON structure
  Serial.print(",\"gps\":{");
  
  // Position data - Fix JSON format
  Serial.print("\"lat\":");
  Serial.print(gps.location.lat(), 7);
  Serial.print(",\"lng\":");
  Serial.print(gps.location.lng(), 7);
  Serial.print(",\"alt\":");
  Serial.print(gps.altitude.isValid() ? gps.altitude.meters() : 0);
 
  
  // Velocity data - Fix JSON format
  Serial.print(",\"speed_kmh\":");
  Serial.print(gps.speed.isValid() ? gps.speed.kmph() : 0);
  
  // Time data
  Serial.print(",\"time\":");
  
  if (gps.time.isValid()) {
    char timeStr[9];
    sprintf(timeStr, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.print("\"value\":\"");
    Serial.print(timeStr);
    Serial.print("\"");
  } else {
    Serial.print("\"value\":\"00:00:00\"");
  }
  Serial.print("}");
  
  // Close the entire JSON
  Serial.println("");

}


