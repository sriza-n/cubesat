/*
  Example for different sending methods
  
  https://github.com/sui77/rc-switch/
  
*/

#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();

void setup() {

  Serial.begin(9600);
  
  // Transmitter is connected to Arduino Pin #10  
  mySwitch.enableTransmit(10);
  
  // Optional set protocol (default is 1, will work for most outlets)
  // mySwitch.setProtocol(2);

  // Optional set pulse length.
  // mySwitch.setPulseLength(320);
  
  // Optional set number of transmission repetitions.
  // mySwitch.setRepeatTransmit(15);
   mySwitch.setRepeatTransmit(1);
  
}

void loop() {

  /* See Example: TypeA_WithDIPSwitches */
  // mySwitch.switchOn("11111", "00010");
  // delay(1000);
  // mySwitch.switchOff("11111", "00010");
  // delay(1000);

  /* Same switch as above, but using decimal code */
  // mySwitch.send(5393, 24);
  // delay(1000);  
  // mySwitch.send(5396, 24);
  // delay(1000);  

  /* Same switch as above, but using binary code */
  // mySwitch.send("000000000001010100010001");
  // delay(1000);  
  // mySwitch.send("000000000001010100010100");
  // delay(1000);

  /* Same switch as above, but tri-state code */ 
  // mySwitch.sendTriState("00000FFF0F0F");
  // delay(1000);  
  // mySwitch.sendTriState("00000FFF0FF0");
  // delay(1000);

  // delay(20000);
    const char* message = "hello";
  for (int i = 0; message[i] != '\0'; ++i) {
    char c = message[i];
    String binaryString = String(c, BIN); // Convert character to binary string
    while (binaryString.length() < 8) {
      binaryString = "0" + binaryString; // Pad with leading zeros to ensure 8 bits
    }
    mySwitch.send(binaryString.c_str());
    delay(1000); // Delay between sending each character
  }
} 
