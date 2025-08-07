#include <VirtualWire.h>

const int receive_pin = 12;  // RF receiver pin
String partialMessage = "";  // Store incomplete messages

void setup() {
    Serial.begin(9600);    // Initialize serial communication
    while (!Serial);       // Wait for serial port to connect
    
    // Initialize VirtualWire
    vw_set_rx_pin(receive_pin);
    vw_setup(2000);  // Bits per second
    vw_rx_start();   // Start the receiver
    
    Serial.println("Status: Waiting for data");
}

void loop() {
    uint8_t buf[VW_MAX_MESSAGE_LEN];
    uint8_t buflen = VW_MAX_MESSAGE_LEN;

    if (vw_get_message(buf, &buflen)) {
        // Message received
        // Null terminate the received data
        buf[buflen] = '\0';
        
        // Convert only the valid received bytes to string
        String message = String((char*)buf);
        
        Serial.print("Message: ");
        Serial.println(message);
        
        // Check if message starts with '{'
        if (message.indexOf('{') != -1) {
            partialMessage = message; // Start new message
        }
        // Check if message ends with '}'
        else if (message.indexOf('}') != -1) {
            partialMessage += message; // Complete the message
            // Print complete message
            Serial.print("Complete JSON: ");
            Serial.println(partialMessage);
            partialMessage = ""; // Reset for next message
        }
        else {
            // Middle part of message
            partialMessage += message;
        }
    }
    else {
        Serial.println("Status: Waiting for data");
        delay(1000);
    }
     // Small delay to prevent overwhelming the serial output
}