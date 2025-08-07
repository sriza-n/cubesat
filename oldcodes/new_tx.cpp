#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile

// Initialize with:
// Speed: 2000 bits per second (default)
// RX: 0 (disabled since we're only transmitting)
// TX: pin 10
// PTT: 0 (disabled, not needed for basic transmitter)
RH_ASK driver(2000, 0, 10, 0);

bool sendMessage(const String& message, uint8_t* buf) {
  uint8_t msgLen = message.length();
 
  // if (msgLen >= RH_ASK_MAX_MESSAGE_LEN) {
  //     Serial.println("Message too long!");
  //     Serial.println("Length: " + String(msgLen) + " Max: " + String(RH_ASK_MAX_MESSAGE_LEN));
  //     return false;
  // }
 
  memset(buf, 0, sizeof(RH_ASK_MAX_MESSAGE_LEN));
  message.getBytes(buf, msgLen + 1);
 
  driver.send(buf, msgLen);
  delay(10); // Wait for the packet to be sent
  driver.waitPacketSent();
 
  // Serial.print("Sending (" + String(msgLen) + " bytes): ");
  // Serial.println(message);
  
  return true;
}

void setup()
{
    Serial.begin(9600);    // Debugging only
    if (!driver.init())
         Serial.println("init failed");
}

void loop()
{
  // String output = "Hello world!" ;
  String output1 = "{" + 
  String(random(0, 100) + random(0, 100) / 100.0, 2) + "," +  // humidity 0-100%
  String(random(20, 35) + random(0, 100) / 100.0, 2) + "," +  // DHT temp 20-35°C
  String(random(20, 35) + random(0, 100) / 100.0, 2) + "," +  // LM35 temp 20-35°C
  String(random(0, 101)) + "," +                              // light 0-100%
  String(random(-4000, 4001)) + "," +                        // compass X
  String(random(-4000, 4001)) + "," +                        // compass Y
  String(random(-4000, 4001)) + "," +                        // compass Z
  String(random(20, 35) + random(0, 100) / 100.0, 2) + "," + // BMP temp 20-35°C
  String(random(95000, 102000)) + "," +                      // pressure 950-1020 hPa
  String(random(0, 1000) + random(0, 100) / 100.0, 2) ;  // altitude 0-1000m

  String output2 =  "," + String(random(-16000, 16001)) + "," +                      // accel X
  String(random(-16000, 16001)) + "," +                      // accel Y
  String(random(-16000, 16001)) + "," +                      // gyro X
  String(random(-16000, 16001)) + "," +                      // gyro Y
  String(random(-16000, 16001)) + "," +                      // gyro Z
  String(random(-16000, 16001)) + "," +                      // gyro Z
  String(random(-90, 90) + random(0, 1000000) / 1000000.0, 6) + "," + // latitude
  String(random(-180, 180) + random(0, 1000000) / 1000000.0, 6) + "}"; // longitude
  
  Serial.print(output1);
  Serial.println(output2);
  delay(100);
   // Create buffer and ensure it doesn't exceed max length
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  if (!sendMessage(output1, buf)) return;
    // delay(500);
  if (!sendMessage(output2, buf)) return;
   delay(100);
}