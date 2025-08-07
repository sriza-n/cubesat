#include <WiFi.h>
#include <WebServer.h>

// Access Point credentials
const char* ssid = "ESP32";
const char* password = "12345678";  // Minimum 8 characters

// GPIO pin to control
const int controlPin = 2;  // GPIO2 (built-in LED on many ESP32 boards)
bool pinState = LOW;       // Initial state (OFF)

// Web server on port 80
WebServer server(80);

void handleRoot() {
    String html = "<!DOCTYPE html><html>";
    html += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
    html += "<style>";
    html += "body { font-family: Arial; text-align: center; margin-top: 50px; }";
    html += ".button { display: inline-block; padding: 15px 25px; font-size: 24px; cursor: pointer; ";
    html += "text-align: center; color: #fff; background-color: " + String(pinState ? "#4CAF50" : "#f44336") + "; ";
    html += "border: none; border-radius: 15px; box-shadow: 0 5px #999; }";
    html += ".button:active { box-shadow: 0 2px #666; transform: translateY(4px); }";
    html += "</style></head>";
    html += "<body><h1>ESP32 Control Panel</h1>";
    html += "<p>Pin State: <strong>" + String(pinState ? "ON" : "OFF") + "</strong></p>";
    html += "<a href=\"/toggle\"><button class=\"button\">" + String(pinState ? "Turn OFF" : "Turn ON") + "</button></a>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  }
  
  void handleToggle() {
    pinState = !pinState;             // Toggle the pin state
    digitalWrite(controlPin, pinState); // Apply to the physical pin
    
    Serial.print("Pin state changed to: ");
    Serial.println(pinState ? "ON" : "OFF");
    
    // Redirect back to the main page
    server.sendHeader("Location", "/");
    server.send(303);
  }

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("\nStarting ESP32 Access Point and Web Server");
  
  // Configure control pin as output
  pinMode(controlPin, OUTPUT);
  digitalWrite(controlPin, pinState);
  
  // Create WiFi Access Point
  WiFi.softAP(ssid, password);
  
  // Print AP details
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP SSID: ");
  Serial.println(ssid);
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // Set up web server routes
  server.on("/", handleRoot);         // Main page
  server.on("/toggle", handleToggle); // Toggle the pin state
  
  // Start the server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // Handle client requests
  server.handleClient();
  delay(2); // Small delay to prevent watchdog issues
}

