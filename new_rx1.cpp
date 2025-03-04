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
                Serial.println("Valid data packet received:");
                
                Serial.print("Humidity: "); Serial.println(receivedData[0]);
                Serial.print("Temperature (LM35): "); Serial.println(receivedData[1]);
                Serial.print("Light: "); Serial.println(receivedData[2]);
                Serial.print("Heading: "); Serial.println(receivedData[3]);
                Serial.print("Pressure: "); Serial.println(receivedData[4]);
                Serial.print("Altitude: "); Serial.println(receivedData[5]);
                Serial.print("Accel X: "); Serial.println(receivedData[6]);
                Serial.print("Accel Y: "); Serial.println(receivedData[7]);
                Serial.print("Accel Z: "); Serial.println(receivedData[8]);
                Serial.print("Gyro X: "); Serial.println(receivedData[9]);
                Serial.print("Gyro Y: "); Serial.println(receivedData[10]);
                Serial.print("Gyro Z: "); Serial.println(receivedData[11]);
                Serial.print("Latitude: "); Serial.println(receivedData[12], 6);
                Serial.print("Longitude: "); Serial.println(receivedData[13], 6);
                Serial.println("------------------");
            } else {
                Serial.println("Invalid packet - markers not found");
            }
        } else {
            Serial.println("Unexpected data size - packet may be corrupted");
        }
    }
    delay(200);  // Small delay to prevent overwhelming the serial output
}