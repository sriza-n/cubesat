#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();

void setup() {
  Serial.begin(9600);
  mySwitch.enableTransmit(10);  // Transmitter on pin 10
  
  // Optimize protocol settings
  mySwitch.setProtocol(1);  // Use protocol 1 for better reliability
  mySwitch.setPulseLength(180);  // Shorter pulse length for faster transmission
  mySwitch.setRepeatTransmit(3);  // Minimum repeats for reliability
}

void loop() {
  const char* message = "hello";
  
  // Convert each character to a 24-bit transmission 
  // (8 bits per char * 3 for redundancy)
  for(int i = 0; message[i] != '\0'; i++) {
    unsigned long code = 0;
    char c = message[i];
    
    // Create 24-bit code with character repeated 3 times
    code = ((unsigned long)c << 16) | ((unsigned long)c << 8) | c;
    
    mySwitch.send(code, 24);  // Send 24 bits
    delay(50);  // Brief delay between characters
  }
  
  delay(1000);  // Delay before next transmission
}

// ---------------------------------------------------

#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();

void setup() {
  Serial.begin(9600);
  mySwitch.enableReceive(0);  // Receiver on interrupt 0 (pin 2)
  
  // Set higher tolerance for better reception
  mySwitch.setReceiveTolerance(60);
}

void loop() {
  if (mySwitch.available()) {
    unsigned long value = mySwitch.getReceivedValue();
    
    if (value != 0) {
      // Extract the character (lowest 8 bits)
      char receivedChar = value & 0xFF;
      
      // Verify redundancy (all 3 copies should match)
      char copy1 = (value >> 8) & 0xFF;
      char copy2 = (value >> 16) & 0xFF;
      
      if(receivedChar == copy1 && receivedChar == copy2) {
        Serial.print(receivedChar);
      } else {
        Serial.print("?"); // Indicate error
      }
    }
    
    mySwitch.resetAvailable();
  }
}