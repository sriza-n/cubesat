#include <Arduino.h>
// #include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <DHT.h>
#include <QMC5883LCompass.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
// #include <Adafruit_BMP085_U.h>
// #include <RH_ASK.h>
#include <VirtualWire.h>

// Pin Definitions
#define GPS_RX 4
#define GPS_TX 5
#define DHTPIN 7
#define LM35_PIN A0
#define LDR_PIN A1
#define DHTTYPE DHT22

#define QMC5883L_ADDR 0x1E

// Object Initialization
TinyGPS gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
DHT dht(DHTPIN, DHTTYPE);
QMC5883LCompass compass;
Adafruit_BMP085 bmp;
MPU6050 mpu(0x68);


float lat = 0.0, lon = 0.0;
// float lat = 27.656908, lon = 85.327476;

//433MHz RF
const int transmit_pin = 10;


bool sendMessage(const String& message) {
    uint8_t buf[message.length() + 1];
    message.getBytes(buf, message.length() + 1);
    
    digitalWrite(LED_BUILTIN, HIGH); // Flash LED to show transmitting
    vw_send(buf, message.length());
    vw_wait_tx(); // Wait until the whole message is gone
    digitalWrite(LED_BUILTIN, LOW);
    
    return true;
}

void setup() {
  Serial.begin(115200);
  // Initialize GPS
  gpsSerial.begin(9600);
  Wire.begin();
  
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
  pinMode(LED_BUILTIN, OUTPUT);
  analogReference(DEFAULT);

  //rf
  vw_set_tx_pin(transmit_pin);
  vw_setup(2000); // Bits per sec

  Serial.println("All sensors initialized");
  delay(1000);
}

void loop() {
  // GPS Data
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    delay(5);
    if (gps.encode(c)) {
      gps.f_get_position(&lat, &lon);
      delay(20);
    }
  }
  
  
  // DHT22 Data
  float humidity = dht.readHumidity();
  float temp_dht = dht.readTemperature();
  
  // LM35 Data
  float voltage = (analogRead(LM35_PIN) * 5.0) / 1024.0;
  float temp_lm35 = voltage * 100.0;

 
  // LDR Data
  int light = map(analogRead(LDR_PIN), 0, 1023, 0, 100);
  
  // QMC5883L Data
  compass.read();
  float compass_x = compass.getX();
  float compass_y = compass.getY();
  float compass_z = compass.getZ();
  
  // BMP180 Data
  float pressure = bmp.readPressure() / 100.0F;
  float temperature = bmp.readTemperature();
  float altitude = bmp.readAltitude();
  
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
     String(humidity, 1) + "," +
     String(temp_dht, 1) + "," + 
     String(temp_lm35, 1) + "," +
     String(light) + "," +
     String(compass_x,1) + "," +
     String(compass_y,1) + "," +
     String(compass_z,1) + "," +
     String(temperature,1) + "," +
     String(pressure,1) + "," +
     String(altitude,2);
  
  String output2 = "," +String(ax) + "," +
     String(ay) + "," +
     String(az) + "," +
     String(gx) + "," +
     String(gy) + "," +
     String(gz) + "," +
     String(lat, 5) + "," +
     String(lon, 5) + "}";

 
  Serial.print(output1);
  Serial.println(output2);
//   delay(800);
//   if (!sendMessage(output1, buf)) return;
//   delay(800);
//   if (!sendMessage(output2, buf)) return;
delay(100);
if (!sendMessage(output1)) return;
delay(200);
if (!sendMessage(output2)) return;
  delay(100);
}