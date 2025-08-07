#include <Arduino.h>
#include <WiFi.h>

// WiFi credentials - replace with your network details
const char* ssid = "spaceresearchcenter_2";
const char* password = "CLB43A64FE";

// Variables
unsigned long previousMillis = 0;
const long interval = 10;  // Check RSSI every 5 seconds

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32 WiFi RSSI Monitor");
  
  // Connect to WiFi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  // Print connection details once connected
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check if it's time to measure RSSI
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Check if still connected
    if (WiFi.status() == WL_CONNECTED) {
      long rssi = WiFi.RSSI();
      Serial.print("Signal strength (RSSI): ");
      Serial.print(rssi);
      Serial.println(" dBm");
    } else {
      Serial.println("WiFi connection lost. Attempting to reconnect...");
      WiFi.reconnect();
    }
  }
}