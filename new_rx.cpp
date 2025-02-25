#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile

// Initialize with:
// Speed: 2000 bits per second (default)
// RX: pin 11 (data pin for RF433MHz receiver)
// TX: 0 (disabled since we're only receiving)
// PTT: 0 (disabled)
RH_ASK driver(4800, 12, 0, 0); 
String partialMessage = ""; // Store incomplete messages

void setup()
{
    Serial.begin(9600);    // Match transmitter baud rate
    while(!Serial);        // Wait for serial port to connect
    if (!driver.init())
    {
         Serial.println("init failed");
         while (1) { delay(1000); }  // Don't proceed if initialization fails
    }
    Serial.println("Status: Waiting for data");
}

void loop()
{
    uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

    while (!driver.recv(buf, &buflen)) // Non-blocking
    {
        Serial.println("Status: Waiting for data");
        delay(1000); // Add a delay to avoid flooding the serial output
    }

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
        Serial.print("Complete JSON:");
        Serial.println(partialMessage);
        partialMessage = ""; // Reset for next message
    }
}