#include <RCSwitch.h>

RCSwitch rf = RCSwitch();

union FloatUnion {
    float f;
    uint32_t i;
};

float receiveFloat() {
    FloatUnion u;
    while (!rf.available()) delay(1);
    u.i = (uint32_t)rf.getReceivedValue() << 16;
    rf.resetAvailable();
    
    while (!rf.available()) delay(1);
    u.i |= rf.getReceivedValue() & 0xFFFFFF;
    rf.resetAvailable();
    
    return u.f;
}

int32_t receiveInt32() {
    int32_t value;
    while (!rf.available()) delay(1);
    value = (int32_t)rf.getReceivedValue() << 16;
    rf.resetAvailable();
    
    while (!rf.available()) delay(1);
    value |= rf.getReceivedValue() & 0xFFFFFF;
    rf.resetAvailable();
    
    return value;
}

void setup() {
    Serial.begin(9600);
    rf.enableReceive(0);  // Receiver on interrupt 0 (pin 2)
    rf.setReceiveTolerance(60);
    Serial.println("Receiver initialized");
}

void loop() {
    // Wait for start marker
    while (!rf.available() || rf.getReceivedValue() != 0xFFFFFE) {
        if (rf.available()) rf.resetAvailable();
        delay(1);
    }
    rf.resetAvailable();

    // Receive all sensor data
    float humidity = receiveFloat();
    float temp_dht = receiveFloat();
    float temp_lm35 = receiveFloat();
    int32_t light = receiveInt32();
    int32_t compass_x = receiveInt32();
    int32_t compass_y = receiveInt32();
    int32_t compass_z = receiveInt32();
    float bmp_temp = receiveFloat();
    float bmp_pressure = receiveFloat();
    float bmp_altitude = receiveFloat();
    int32_t ax = receiveInt32();
    int32_t ay = receiveInt32();
    int32_t az = receiveInt32();
    int32_t gx = receiveInt32();
    int32_t gy = receiveInt32();
    int32_t gz = receiveInt32();
    float lat = receiveFloat();
    float lon = receiveFloat();

    // Format output string
    String output = "{" + 
        String(humidity, 2) + "," +
        String(temp_dht, 2) + "," + 
        String(temp_lm35, 2) + "," +
        String(light) + "," +
        String(compass_x) + "," +
        String(compass_y) + "," +
        String(compass_z) + "," +
        String(bmp_temp, 2) + "," +
        String(bmp_pressure, 2) + "," +
        String(bmp_altitude, 2) + "," +
        String(ax) + "," +
        String(ay) + "," +
        String(az) + "," +
        String(gx) + "," +
        String(gy) + "," +
        String(gz) + "," +
        String(lat, 6) + "," +
        String(lon, 6) + "}";

    Serial.println(output);
}