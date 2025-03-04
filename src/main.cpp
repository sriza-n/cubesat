#include <Arduino.h>
// #include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <DHT.h>
// #include <QMC5883LCompass.h>
// #include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
// #include <RH_ASK.h>
#include <VirtualWire.h>
#include <DFRobot_QMC5883.h>

// Pin Definitions
#define GPS_RX 4
#define GPS_TX 5
#define DHTPIN 7
#define LM35_PIN A0
#define LDR_PIN A1
#define DHTTYPE DHT22

#define QMC5883L_ADDR 0x1E
#define BMP180_I2C_ADDRESS 0x77


DFRobot_QMC5883 compass(&Wire, QMC5883L_ADDR);

// Object Initialization
TinyGPS gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
DHT dht(DHTPIN, DHTTYPE);
// QMC5883LCompass compass;
// Adafruit_BMP085 bmp;
MPU6050 mpu(0x68);


// float lat = 0.0, lon = 0.0;
float lat = 27.656908, lon = 85.327476;

float pressure = 0.0;
float altitude = 0.0;


//433MHz RF
const int transmit_pin = 10;
// static char buffer[150];

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


float sensorData[16]; // Array to hold all sensor readings

bool sendMessage(const float dataArray[], int arraySize) {
    // Calculate buffer size: each float is 4 bytes
    uint8_t buf[arraySize * sizeof(float)];
    
    // Copy the float array to the byte buffer
    memcpy(buf, dataArray, arraySize * sizeof(float));
    
    digitalWrite(LED_BUILTIN, HIGH); // Flash LED to show transmitting
    vw_send(buf, arraySize * sizeof(float));
    vw_wait_tx(); // Wait until the whole message is gone
    digitalWrite(LED_BUILTIN, LOW);
    
    return true;
}

// bool sendMessage(const String& message) {
//     uint8_t buf[message.length() + 1];
//     message.getBytes(buf, message.length() + 1);
    
//     digitalWrite(LED_BUILTIN, HIGH); // Flash LED to show transmitting
//     vw_send(buf, message.length());
//     vw_wait_tx(); // Wait until the whole message is gone
//     digitalWrite(LED_BUILTIN, LOW);
    
//     return true;
// }

void setup() {
  Serial.begin(9600);
  // Initialize GPS
  gpsSerial.begin(9600);
  Wire.begin();
  
  // Initialize DHT22
  dht.begin();
  
  // Initialize QMC5883L
  // compass.init();
  // compass.setMode(0x01, 0x0C, 0x10, 0x00);
  if (!compass.begin()) {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
  }
  
  // Initialize BMP180
  if (!bmp.begin()) {
    Serial.println("BMP180 init failed");
  }
  
  // Initialize MPU6050
  if(!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  }
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
      delay(30);
    }
  }
  
  
  // DHT22 Data
  float humidity = dht.readHumidity();
//   float temp_dht = dht.readTemperature();
  
  // LM35 Data
  float voltage = (analogRead(LM35_PIN) * 5.0) / 1024.0;
  float temp_lm35 = voltage * 100.0;

 
  // LDR Data
  int light = map(analogRead(LDR_PIN), 0, 1023, 0, 100);
  

  // QMC5883L Data
  sVector_t mag = compass.readRaw();
  compass.setDeclinationAngle((4.0 + (26.0 / 60.0)) / (180 / PI));
  
  // Calculate heading using arctangent of y/x
  float heading = atan2(mag.YAxis, mag.XAxis);
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
  
  // Convert radians to degrees
  heading = heading * 180/PI;
  
//   String direction = getDirection(heading);
//   float compass_x = mag.XAxis;
  
  // BMP180 Data
    sensors_event_t event;
    bmp.getEvent(&event);

    pressure = event.pressure; // pressure in hPa
    float temperature;
    bmp.getTemperature(&temperature);

    // Calibrated for Kathmandu's elevation (approximately 1400m)
    float seaLevelPressure = event.pressure / pow(1.0 - (1400.0/44330.0), 5.255);
    
    // Calculate altitude using calibrated sea level pressure
    altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure, temperature);
    
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
  


//   // Build output string
//   String output1 = "{" + 
//      String(humidity, 1) + "," +
//     //  String(temp_dht, 1) + "," + 
//      String(temp_lm35, 1) + "," +
//      String(light) + "," +
//      String(heading,1) + "," +
//     //  String(compass_y,1) + "," +
//     //  String(compass_z,1) + "," +
//     //  String(temperature,1) + "," +
//      String(pressure,1) + "," +
//      String(altitude,2);
  
//   String output2 = "," +String(ax) + "," +
//      String(ay) + "," +
//      String(az) + "," +
//      String(gx) + "," +
//      String(gy) + "," +
//      String(gz) + "," +
//      String(lat, 5) + "," +
//      String(lon, 5) + "}";


 
//   Serial.print(output1);
//   Serial.println(output2);
  
//   delay(600);
//   sendMessage(output1);
//   delay(800);
//   sendMessage(output2);
  // Serial.println(buffer);
  // delay(100);
  // sendMessage(buffer);

   // Instead of creating output strings, populate the data array
   sensorData[0] = humidity;
   sensorData[1] = temp_lm35;
   sensorData[2] = light;
   sensorData[3] = heading;
   sensorData[4] = pressure;
   sensorData[5] = altitude;
   sensorData[6] = ax;
   sensorData[7] = ay;
   sensorData[8] = az;
   sensorData[9] = gx;
   sensorData[10] = gy;
   sensorData[11] = gz;
   sensorData[12] = lat;
   sensorData[13] = lon;
   // Add markers to identify start/end of packet (optional)
   sensorData[14] = 0xAAAA; // Start marker
   sensorData[15] = 0xFFFF; // End marker

//    Serial.println(sensorData);
   delay(600);
   // Send all data at once instead of splitting into two messages
   sendMessage(sensorData, 16);
  delay(100);
}