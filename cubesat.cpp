#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <VirtualWire.h>

// Pin Definitions
#define GPS_RX 4
#define GPS_TX 5
#define LM35_PIN A0
#define LDR_PIN A1
#define BMP180_I2C_ADDRESS 0x77

TinyGPS gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
MPU6050 mpu;

// Add MPU6050 DMP variables
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

// Orientation/motion variables
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorInt16 gg;         // [x, y, z]            Gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/pitch/roll container

// Conversion constants
#define EARTH_GRAVITY_MS2 9.80665
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

float lat = 0.0, lon = 0.0; // Initialize GPS coordinates
float voltage = 0.0;
float temp_lm35 = 0.0;
int light = 0;

// Add GPS variables for better debugging
unsigned long chars = 0;
unsigned short sentences = 0, failed = 0;
unsigned long lastGpsReadTime = 0;
bool gpsFixed = false;

float pressure = 0.0;
float altitude = 0.0;

const int transmit_pin = 10;

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// Update sensorData size to accommodate quaternion (w,x,y,z) instead of raw acc/gyro
float sensorData[11]; // Array to hold all sensor readings

bool sendMessage(const float dataArray[], int arraySize) {
    uint8_t buf[arraySize * sizeof(float)];
    memcpy(buf, dataArray, arraySize * sizeof(float));
    digitalWrite(LED_BUILTIN, HIGH); // Flash LED to show transmitting
    vw_send(buf, arraySize * sizeof(float));
    vw_wait_tx(); // Wait until the whole message is gone
    digitalWrite(LED_BUILTIN, LOW);
    return true;
}

void setup() {
  Serial.begin(9600);
  
  // For NEO-6M, try these connections (RX/TX swapped from original)
  // NEO-6M TX connects to Arduino RX (GPS_RX pin)
  // NEO-6M RX connects to Arduino TX (GPS_TX pin)
  gpsSerial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock
  
  if (!bmp.begin()) {
    Serial.println("BMP180 init failed");
  }
  
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  if(!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  } else {
    Serial.println("MPU6050 connection successful");
    
    // Initialize DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // Auto-calibration
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println("Calibration complete, active offsets:");
    mpu.PrintActiveOffsets();
    
    if (devStatus == 0) {
      // Turn on the DMP
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
      Serial.println(F("DMP ready!"));
    } else {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
  }
  
  pinMode(LDR_PIN, INPUT);
  pinMode(LM35_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  analogReference(DEFAULT);

  vw_set_tx_pin(transmit_pin);
  vw_setup(2000); // Bits per sec

  Serial.println("All sensors initialized");
  Serial.println("Waiting for GPS signal...");
  delay(1000);
}

// New function to read GPS data with better error handling
bool readGPS() {
  bool newData = false;
  unsigned long start = millis();
  
  // Try to get new data for up to 1 second
  while (millis() - start < 1000) {
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      // Serial.write(c); // Uncomment for raw GPS data debugging
      if (gps.encode(c)) {
        newData = true;
        chars++;
      }
    }
    if (newData) break;
  }
  
  if (newData) {
    float temp_lat, temp_lon;
    unsigned long age;
    gps.f_get_position(&temp_lat, &temp_lon, &age);
    
    if (age != TinyGPS::GPS_INVALID_AGE) {
      lat = temp_lat;
      lon = temp_lon;
      
      // Check if we have a valid position
      if (lat != 0.0 || lon != 0.0) {
        gpsFixed = true;
      }
      
      // Get additional GPS data if needed
      // float altitude = gps.f_altitude();
      // unsigned long date, time;
      // gps.get_datetime(&date, &time);
      
      return true;
    }
  }
  
  // Set default coordinates when GPS is not available
  if (!gpsFixed) {
    lat = 27.656908;
    lon = 85.327476;
  }
  
  return false;
}

void loop() {
  // Replace the old GPS reading code with our new function
  if (millis() - lastGpsReadTime > 2000) { // Check GPS every 2 seconds
    if (readGPS()) {
      sentences++;
    } else {
      failed++;
    }
    lastGpsReadTime = millis();
  }
  
  temp_lm35 = map(analogRead(LM35_PIN), 0, 1023, 0, 100);

  light = map(analogRead(LDR_PIN), 0, 1023, 0, 100);

  sensors_event_t event;
  bmp.getEvent(&event);

  pressure = event.pressure; // pressure in hPa
  float temperature;
  bmp.getTemperature(&temperature);

  float seaLevelPressure = event.pressure / pow(1.0 - (1400.0/44330.0), 5.255);
  altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure, temperature);
    
  if (dmpReady) {
    // Read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // Get quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      
      // Get gravity
      mpu.dmpGetGravity(&gravity, &q);
      
      // Get Euler angles
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      // Get acceleration
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
      
      // Get gyro measurements
      mpu.dmpGetGyro(&gg, fifoBuffer);
    }
  }

  sensorData[0] = temp_lm35;
  sensorData[1] = light;
  sensorData[2] = pressure* 0.0145038;
  sensorData[3] = altitude;
  // Store quaternion values (w,x,y,z)
  sensorData[4] = q.w;
  sensorData[5] = q.x;
  sensorData[6] = q.y;
  sensorData[7] = q.z;
  // Store GPS and markers (index adjusted)
  sensorData[8] = lat;
  sensorData[9] = lon;
  sensorData[10] = 0xAAAA; // Marker
   
  Serial.println("---- SENSOR DATA ----");
  Serial.print("Temperature: "); Serial.print(sensorData[0]); Serial.println(" °C");
  Serial.print("Light: "); Serial.print(sensorData[1]); Serial.println(" %");
  Serial.print("Pressure: "); Serial.print(sensorData[2]); Serial.println(" PSI"); 
  Serial.print("Altitude: "); Serial.print(sensorData[3]); Serial.println(" m");
  Serial.print("Quaternion (w,x,y,z): "); 
  Serial.print(sensorData[4]); Serial.print(", ");
  Serial.print(sensorData[5]); Serial.print(", ");
  Serial.print(sensorData[6]); Serial.print(", ");
  Serial.println(sensorData[7]);
  Serial.print("GPS: "); 
  if (gpsFixed) {
    Serial.print(sensorData[8], 6); 
    Serial.print(", "); Serial.println(sensorData[9], 6);
  } else {
    Serial.println("No Fix");
  }
  
  // Serial.print("GPS Stats - Chars: "); Serial.print(chars);
  // Serial.print(" Sentences: "); Serial.print(sentences);
  // Serial.print(" Failed: "); Serial.println(failed);
  Serial.print("Marker: 0x"); Serial.println(sensorData[10], HEX);
  Serial.println("-------------------");

  // delay(600);
  sendMessage(sensorData, 11); // Update size from 12 to 11
  // delay(100);
}