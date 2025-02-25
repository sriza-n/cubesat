#include <VirtualWire.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Arduino.h>

#include "DHT.h"
#define DHTPIN 7 
#define DHTTYPE DHT22 
DHT dht(DHTPIN, DHTTYPE);
//
#include <SoftwareSerial.h>
#include <TinyGPS.h>
TinyGPS gps;
SoftwareSerial ss(5,4);
Adafruit_BMP085 bmp;
const int transmit_pin = 10;
int numbers[13]; //Number of variables to send out
int statusLED = 13;
bool newData = false;

void setup()
{
  delay(50);
  Serial.begin(19200);
  vw_set_tx_pin(transmit_pin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(4000);
  dht.begin();
  ss.begin(19200);
  pinMode(LED,OUTPUT);
  bmp.begin();

  //vw_set_tx_pin(6); // pin 6 is used as the transmit data out into the TX Link module, change this to suit your needs.
}

void loop()
{

  // unsigned long chars;
  // unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
     //  Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  float flat,flon;
  unsigned long age;
   gps.f_get_position(&flat, &flon, &age);
   flat = (flat == TinyGPS::GPS_INVALID_F_ANGLE) ? 0.0 : flat;
  flon = (flon == TinyGPS::GPS_INVALID_F_ANGLE) ? 0.0 : flon;
  
    int nitrogen=analogRead(A6);
    int ldr=analogRead(A0)/100;
    int ax=analogRead(A1);
    int ay=analogRead(A2);
    int az=analogRead(A3);

    
    float temp = (bmp.readTemperature()); //Read Temperature and convert to fahrenheit
    int temp1 = (int)temp; //Read Temperature digits on the left of the decimal place
    int temp2 = (int)((temp - temp1) * 100.0); //Read Temperature digits on the right of the decimal place
    

    float alt = bmp.readAltitude(103149) * 3.280839895/3.280839895; //Read Altitude with correction
    int alt1 = (int)alt; //Read Altitude digits on the left of the decimal place
    int alt2 = (int)((alt - alt1) * 100.0); //Read Altitude digits on the right of the decimal place
    
    
    float bar = bmp.readPressure(); //Read Barometric Pressure
    int  bar1 = (bar); //Read Altitude digits on the left of the decimal place
  //  int bar2 = (int)((bar - bar1) * 100.0); //Read Altitude digits on the right of the decimal place
    float h = dht.readHumidity();
    int h1=h;

    float lat= flat;
    int lat1=lat;
    int lat2=(int)((lat-lat1)*10000.0);
    
    float lon= flon;
    int lon1=lon;
    int lon2=(int)((lon-lon1)*10000.0);


    
    //-------------------------------End of Readings from BMP085------------------------------------------------
    

    Serial.print (temp1); //Print Temperature digits on the left of the decimal place
    Serial.print (","); //Print decimal
  
    Serial.print (alt1); //Print Altitude digits on the left of the decimal place
    Serial.print (","); //Print decimal
    Serial.print (bar1); //Print Barometric Pressure digits on the left of the decimal place
//    Serial.print (bar2); //Print Barometric Pressure digits on the left of the decimal place
       //Serial.println (" inHg  "); //Print unit of measure
    Serial.print(",");
    Serial.print("humidity:");
    Serial.print(h1);

    Serial.print(",");


       Serial.print(ax);
   Serial.print(",");
      Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
   Serial.println(",");
   Serial.print(ldr);
    Serial.print(",");
Serial.print("nitrogen:");
   Serial.print(nitrogen);
   Serial.print(",");
    Serial.print(lat2);
    Serial.print(",");
    Serial.print(lon2);
    Serial.print(",");
    Serial.print(alt2);
    Serial.print(",");
    Serial.print(temp2);
     
   
} 
