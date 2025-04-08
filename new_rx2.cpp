#include <VirtualWire.h>

const int receive_pin = 12;  // RF receiver pin
const int FLOAT_SIZE = 4;    // Size of float in bytes
const int ARRAY_SIZE = 11;   // Size of the sensor array (changed from 16 to 11)
const int EXPECTED_BYTES = ARRAY_SIZE * FLOAT_SIZE;  // Expected bytes (44)

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
            
            // Check marker to ensure valid packet (only one marker now at index 10)
            if (receivedData[10] == 0xAAAA) {
                // Valid packet received, print the values
                Serial.print("{"); 
                Serial.print("Temp:"); Serial.print(receivedData[0]);
                Serial.print(",");
                Serial.print("Light:"); Serial.print(receivedData[1]);
                Serial.print(",");
                Serial.print("Press:"); Serial.print(receivedData[2]);
                Serial.print(",");
                Serial.print("Alt:"); Serial.print(receivedData[3]);
                Serial.print(",");
                Serial.print("Quat_w:"); Serial.print(receivedData[4]);
                Serial.print(",");
                Serial.print("Quat_x:"); Serial.print(receivedData[5]);
                Serial.print(",");
                Serial.print("Quat_y:"); Serial.print(receivedData[6]);
                Serial.print(",");
                Serial.print("Quat_z:"); Serial.print(receivedData[7]);
                Serial.print(",");
                Serial.print("Lat:"); Serial.print(receivedData[8], 6);
                Serial.print(",");
                Serial.print("Lon:"); Serial.print(receivedData[9], 6);
                Serial.println("}");
            } else {
                Serial.println("Invalid packet - marker not found");
            }
        } else {
            Serial.println("Unexpected data size - packet may be corrupted");
        }

    }
    else {
        // No data received
        Serial.println(" waiting for data");
        delay(200);
    }
}