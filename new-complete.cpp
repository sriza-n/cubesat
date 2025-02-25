#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <DHT.h>
#include <QMC5883LCompass.h>
// #include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <RH_ASK.h>

// Pin Definitions
#define GPS_RX 4
#define GPS_TX 5
#define DHTPIN 7
#define LM35_PIN A0
#define LDR_PIN A1
#define DHTTYPE DHT22

// Object Initialization
TinyGPS gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
DHT dht(DHTPIN, DHTTYPE);
QMC5883LCompass compass;
// Adafruit_BMP085 bmp;
MPU6050 mpu(0x68);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

float lat = 0.0, lon = 0.0;
float pressure, temperature, altitude;
//433MHz RF
RH_ASK driver(2000, 0, 10, 0);

bool sendMessage(const String& message, uint8_t* buf) {
  uint8_t msgLen = message.length();
 
  if (msgLen >= RH_ASK_MAX_MESSAGE_LEN) {
      Serial.println("Message too long!");
      Serial.println("Length: " + String(msgLen) + " Max: " + String(RH_ASK_MAX_MESSAGE_LEN));
      return false;
  }
 
  memset(buf, 0, sizeof(RH_ASK_MAX_MESSAGE_LEN));
  message.getBytes(buf, msgLen + 1);
 
  driver.send(buf, msgLen);
  driver.waitPacketSent();
 
  Serial.print("Sending (" + String(msgLen) + " bytes): ");
  Serial.println(message);
  
  return true;
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize GPS
  gpsSerial.begin(9600);
  
  // Initialize DHT22
  dht.begin();
  
  // Initialize QMC5883L
  compass.init();
  compass.setMode(0x01, 0x0C, 0x10, 0x00);
  
  // Initialize BMP180
  if (!bmp.begin()) {
    Serial.println("BMP180 init failed");
  }
  
  // Initialize MPU6050
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  
  // Initialize analog pins
  pinMode(LDR_PIN, INPUT);
  analogReference(DEFAULT);
//433MHz RF
  if (!driver.init())
  Serial.println("init failed");
  
  Serial.println("All sensors initialized");
}

void loop() {
  // Serial.println("\n=== SENSOR READINGS ===");
  
  // GPS Data
  // float lat = 0.0, lon = 0.0;
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    delay(5);
    if (gps.encode(c)) {
      // unsigned long age;
      gps.f_get_position(&lat, &lon);
      delay(10);
      // Serial.print("GPS: ");
      // Serial.print(lat, 6);
      // Serial.print(",");
      // Serial.println(lon, 6);
    }
  }
  
  // DHT22 Data
  float humidity = dht.readHumidity();
  float temp_dht = dht.readTemperature();
  // Serial.print("DHT22: ");
  // Serial.print(temp_dht);
  // Serial.print("°C, ");
  // Serial.print(humidity);
  // Serial.println("%");
  
  // LM35 Data
  float voltage = (analogRead(LM35_PIN) * 5.0) / 1024.0;
  float temp_lm35 = voltage * 100.0;
  // Serial.print("LM35: ");
  // Serial.print(temp_lm35);
  // Serial.println("°C");
  
  // LDR Data
  int light = map(analogRead(LDR_PIN), 0, 1023, 0, 100);
  // Serial.print("Light: ");
  // Serial.print(light);
  // Serial.println("%");
  
  // QMC5883L Data
  compass.read();
  // Serial.print("Compass: ");
  // Serial.print(compass.getX());
  // Serial.print(",");
  // Serial.print(compass.getY());
  // Serial.print(",");
  // Serial.println(compass.getZ());
  
  // BMP180 Data
  // Serial.print("BMP180: ");
  // Serial.print(bmp.readTemperature());
  // Serial.print("°C, ");
  // Serial.print(bmp.readPressure() / 100.0F);
  // Serial.print("hPa, ");
  // Serial.print(bmp.readAltitude());
  // Serial.println("m");

    // BMP180 Data
    sensors_event_t event;

    // Get pressure reading
    bmp.getEvent(&event);
    pressure = event.pressure;
      
    // Get temperature
    bmp.getTemperature(&temperature);
      
    // Calculate altitude
    altitude = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure);
  
  // MPU6050 Data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Serial.print("MPU6050 Accel: ");
  // Serial.print(ax); Serial.print(",");
  // Serial.print(ay); Serial.print(",");
  // Serial.println(az);
  // Serial.print("MPU6050 Gyro: ");
  // Serial.print(gx); Serial.print(",");
  // Serial.print(gy); Serial.print(",");
  // Serial.println(gz);
  


     // Build output string
  String output1 = "{" + 
     String(humidity, 2) + "," +
     String(temp_dht, 2) + "," + 
     String(temp_lm35, 2) + "," +
     String(light) + "," +
     String(compass.getX()) + "," +
     String(compass.getY()) + "," +
     String(compass.getZ()) + "," +
     String(temperature) + "," +
     String(pressure) + "," +
     String(altitude);
  
  String output2 = "," +
     String(ax) + "," +
     String(ay) + "," +
     String(az) + "," +
     String(gx) + "," +
     String(gy) + "," +
     String(gz) + "," +
     String(lat, 6) + "," +
     String(lon, 6) + "}";

  // Serial.println(output);

  // Create buffer and ensure it doesn't exceed max length
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  if (!sendMessage(output1, buf)) return;
  if (!sendMessage(output2, buf)) return;
  delay(100);
}