#include <VirtualWire.h>

const int receive_pin = 12;  // RF receiver pin
const int FLOAT_SIZE = 4;    // Size of float in bytes
const int ARRAY_SIZE = 16;   // Size of the sensor array
const int EXPECTED_BYTES = ARRAY_SIZE * FLOAT_SIZE;  // Expected bytes (64)

uint8_t buffer[VW_MAX_MESSAGE_LEN];
float receivedData[ARRAY_SIZE];  // Array to hold reconstructed float values

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
    uint8_t buflen = VW_MAX_MESSAGE_LEN;

    if (vw_get_message(buffer, &buflen)) {
        // Data received
        Serial.print("Received packet of size: ");
        Serial.println(buflen);
        
        // Check if we received the expected amount of data
        if (buflen == EXPECTED_BYTES) {
            // Convert bytes back to float array
            memcpy(receivedData, buffer, EXPECTED_BYTES);
            
            // Check markers to ensure valid packet
            if (receivedData[14] == 0xAAAA && receivedData[15] == 0xFFFF) {
                // Valid packet received, print the values
                Serial.print("{"); 
                Serial.print("Hum"); Serial.print(receivedData[0]);
                Serial.print(",");
                Serial.print("Tem"); Serial.print(receivedData[1]);
                Serial.print(",");
                Serial.print("Lig"); Serial.print(receivedData[2]);
                Serial.print(",");
                Serial.print("Hea"); Serial.print(receivedData[3]);
                Serial.print(",");
                Serial.print("Pres"); Serial.print(receivedData[4]);
                Serial.print(",");
                Serial.print("Alt"); Serial.print(receivedData[5]);
                Serial.print(",");
                Serial.print("AX"); Serial.print(receivedData[6]);
                Serial.print(",");
                Serial.print("AY"); Serial.print(receivedData[7]);
                Serial.print(",");
                Serial.print("AZ"); Serial.print(receivedData[8]);
                Serial.print(",");
                Serial.print("GX"); Serial.print(receivedData[9]);
                Serial.print(",");
                Serial.print("GY"); Serial.print(receivedData[10]);
                Serial.print(",");
                Serial.print("GZ"); Serial.print(receivedData[11]);
                Serial.print(",");
                Serial.print("Lat"); Serial.print(receivedData[12], 6);
                Serial.print(",");
                Serial.print("Lon"); Serial.print(receivedData[13], 6);
                Serial.println("}");
            } else {
                Serial.println("Invalid packet - markers not found");
            }
        } else {
            Serial.println("Unexpected data size - packet may be corrupted");
        }
    }
    delay(200);  // Small delay to prevent overwhelming the serial output
}