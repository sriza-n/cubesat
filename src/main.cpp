#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <DHT.h>
#include <MPU6050.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <VirtualWire.h>
#include <DFRobot_QMC5883.h>

// Pin Definitions
#define GPS_RX 4
#define GPS_TX 5
#define DHTPIN 7
#define LM35_PIN A0
#define LDR_PIN A1
#define DHTTYPE DHT22
#define TRANSMIT_PIN 10
#define QMC5883L_ADDR 0x1E

// Timing constants
#define TRANSMIT_INTERVAL 1500    // Time between transmissions (ms)
#define SENSOR_READ_INTERVAL 1000 // Time between sensor readings (ms)
#define GPS_READ_TIMEOUT 100      // Maximum time to read GPS data (ms)

// Object Initialization
TinyGPS gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
DHT dht(DHTPIN, DHTTYPE);
MPU6050 mpu(0x68);
DFRobot_QMC5883 compass(&Wire, QMC5883L_ADDR);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// Global variables for sensor readings
float lat = 27.656908, lon = 85.327476;
float humidity = 0.0, temp_lm35 = 0.0, pressure = 0.0, altitude = 0.0, heading = 0.0;
int light = 0;
int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

// Timer variables
unsigned long lastSensorReadTime = 0;
unsigned long lastTransmitTime = 0;
bool sendFirstPacket = true;

// Preallocated buffers
char dataBuffer1[64];
char dataBuffer2[64];

bool sendMessage(const char* message) {
    uint8_t buf[strlen(message) + 1];
    memcpy(buf, message, strlen(message) + 1);
    
    digitalWrite(LED_BUILTIN, HIGH);
    vw_send(buf, strlen(message));
    vw_wait_tx();
    digitalWrite(LED_BUILTIN, LOW);
    
    return true;
}

void readSensors() {
    // DHT22 Data
    humidity = dht.readHumidity();
    
    // LM35 Data
    float voltage = (analogRead(LM35_PIN) * 5.0) / 1024.0;
    temp_lm35 = voltage * 100.0;
    
    // LDR Data
    light = map(analogRead(LDR_PIN), 0, 1023, 0, 100);
    
    // QMC5883L Data
    sVector_t mag = compass.readRaw();
    compass.setDeclinationAngle((4.0 + (26.0 / 60.0)) / (180 / PI));
    heading = atan2(mag.YAxis, mag.XAxis);
    if(heading < 0) heading += 2*PI;
    heading = heading * 180/PI;
    
    // BMP180 Data
    sensors_event_t event;
    bmp.getEvent(&event);
    pressure = event.pressure;
    float temperature;
    bmp.getTemperature(&temperature);
    float seaLevelPressure = event.pressure / pow(1.0 - (1400.0/44330.0), 5.255);
    altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure, temperature);
    
    // MPU6050 Data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

void readGPS() {
    unsigned long startTime = millis();
    while (gpsSerial.available() && (millis() - startTime < GPS_READ_TIMEOUT)) {
        char c = gpsSerial.read();
        if (gps.encode(c)) {
            gps.f_get_position(&lat, &lon);
            break;
        }
    }
}

void setup() {
    Serial.begin(9600);
    gpsSerial.begin(9600);
    Wire.begin();
    
    // Initialize sensors
    dht.begin();
    
    if (!compass.begin()) {
        Serial.println("QMC5883 sensor failed!");
    }
    
    if (!bmp.begin()) {
        Serial.println("BMP180 init failed!");
    }
    
    if(!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
    }
    mpu.initialize();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    
    // Initialize pins
    pinMode(LDR_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Initialize RF transmitter
    vw_set_tx_pin(TRANSMIT_PIN);
    vw_setup(2000); // Bits per sec
    
    Serial.println("All sensors initialized");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Read GPS data every loop cycle with timeout protection
  readGPS();
  
  // Read other sensors at specific intervals
  if (currentMillis - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
      lastSensorReadTime = currentMillis;
      readSensors();
  }
  
  // Transmit data at specific intervals
  if (currentMillis - lastTransmitTime >= TRANSMIT_INTERVAL) {
      lastTransmitTime = currentMillis;
      
      if (sendFirstPacket) {
          snprintf(dataBuffer1, sizeof(dataBuffer1), 
                   "{1,%.1f,%.1f,%d,%.1f,%.1f,%.2f",
                   (double)humidity, (double)temp_lm35, light, (double)heading, 
                   (double)pressure, (double)altitude);
          sendMessage(dataBuffer1);
          sendFirstPacket = false;
      } else {
          snprintf(dataBuffer2, sizeof(dataBuffer2), 
                   ",%d,%d,%d,%d,%d,%d,%.5f,%.5f}",
                   ax, ay, az, gx, gy, gz, (double)lat, (double)lon);
          sendMessage(dataBuffer2);
          sendFirstPacket = true;
      }
  }
}