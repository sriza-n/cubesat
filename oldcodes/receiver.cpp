/*
  Simple example for receiving
  
  https://github.com/sui77/rc-switch/
*/

#include <RCSwitch.h>
#include <Arduino.h>
#include <SPI.h>

RCSwitch mySwitch = RCSwitch();

void setup() {
  Serial.begin(9600);
  mySwitch.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2
}

// void loop() {
//   if (mySwitch.available()) {
    
//     int value = mySwitch.getReceivedValue();
    
//     if (value == 0) {
//       Serial.print("Unknown encoding");
//     } else {
//       Serial.print("Received ");
//       Serial.print( mySwitch.getReceivedValue() );
//       Serial.print(" / ");
//       Serial.print( mySwitch.getReceivedBitlength() );
//       Serial.print("bit ");
//       Serial.print("Protocol: ");
//       Serial.println( mySwitch.getReceivedProtocol() );
//     }

//     mySwitch.resetAvailable();
//   }
// }

// void loop() {
//   if (mySwitch.available()) {
//     unsigned long value = mySwitch.getReceivedValue();
    
//     if (value == 0) {
//       Serial.print("Unknown encoding");
//     } else {
//       String binaryString = String(value, BIN);
//       while (binaryString.length() < 8) {
//         binaryString = "0" + binaryString; // Pad with leading zeros to ensure 8 bits
//       }
//       char receivedChar = (char) strtol(binaryString.c_str(), NULL, 2); // Convert binary string to character
//       Serial.print("Received character: ");
//       Serial.println(receivedChar);
//     }

//     mySwitch.resetAvailable();
//   }
// }

void loop() {
  if (mySwitch.available()) {
    unsigned long value = mySwitch.getReceivedValue();
    
    if (value == 0) {
      Serial.print("Unknown encoding");
    } else {
      String receivedString = String(value, DEC); // Convert received value to string
      Serial.print("Received string: ");
      Serial.println(receivedString);
    }

    mySwitch.resetAvailable();
  }
}
