#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <DFRobot_QMC5883.h>

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/*Conversion variables*/
#define EARTH_GRAVITY_MS2 9.80665  //m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---MPU6050 Control/Status Variables---*/
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gg;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            World-frame gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

// Time tracking variables
unsigned long previousTime = 0;
float dt = 0.0;  // Time difference in seconds
float filteredAccZ = 0;

// Magnetometer variables
DFRobot_QMC5883 compass(&Wire, 0x1E);

void setup() {
  Serial.begin(115200);

  // Initialize time tracking
  previousTime = millis();

  //for mpu6050
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }
  
  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
   
   //* Initialize the compass */
    if(!compass.begin())
      {
        Serial.println("Could not find a valid 5883 sensor, check wiring!");
        delay(500);
      }
  }
}

void loop() {
  if (!DMPReady) return;

  /* Calculate time between updates */
  unsigned long currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0; // Convert to seconds
  previousTime = currentTime;

  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    /*Display quaternion values in easy matrix form: w x y z */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    Serial.print("quat:");
    Serial.print(q.w);
    Serial.print(",");
    Serial.print(q.x);
    Serial.print(",");
    Serial.print(q.y);
    Serial.print(",");
    Serial.print(q.z);
    Serial.print(",");

    /* Display initial world-frame acceleration, adjusted to remove gravity
    and rotated based on known orientation from Quaternion */
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
    mpu.dmpGetGravity(&gravity, &q);

    // Calculate accelerations in m/s²
    float accX = aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    float accY = aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    float accZ = (aaWorld.z * mpu.get_acce_resolution() - gravity.z) * EARTH_GRAVITY_MS2;


    // Apply threshold to filter out noise
    const float ACC_THRESHOLD = 0.1; // m/s² - adjust based on testing
    if (abs(accX) < ACC_THRESHOLD) accX = 0;
    if (abs(accY) < ACC_THRESHOLD) accY = 0;
    if (abs(accZ) < ACC_THRESHOLD) accZ = 0;


    Serial.print("aworld:");
    Serial.print(accX, 2);
    Serial.print(",");
    Serial.print(accY, 2);
    Serial.print(",");
    Serial.print(accZ, 2);
    Serial.print(",");
        // Time tracking information
    Serial.print("dt:");
    Serial.print(dt, 4);
    Serial.print(",");
    

    //magnetometer
    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
    compass.setDeclinationAngle(declinationAngle);
    sVector_t mag = compass.readRaw();
    compass.getHeadingDegrees();
    Serial.print("Degress:");
    Serial.println(mag.HeadingDegress);
    

    delay(250); // Delay for readability
  }
}
