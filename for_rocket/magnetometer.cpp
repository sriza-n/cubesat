#include <DFRobot_QMC5883.h>

DFRobot_QMC5883 compass(&Wire, 0x1E);

String getDirection(float heading) {
  if (heading >= 337.5 || heading < 22.5)  return "N";
  if (heading >= 22.5  && heading < 67.5)   return "NE";
  if (heading >= 67.5  && heading < 112.5)  return "E";
  if (heading >= 112.5 && heading < 157.5)  return "SE";
  if (heading >= 157.5 && heading < 202.5)  return "S";
  if (heading >= 202.5 && heading < 247.5)  return "SW";
  if (heading >= 247.5 && heading < 292.5)  return "W";
  if (heading >= 292.5 && heading < 337.5)  return "NW";
  return "?";
}

void setup()
{
  Serial.begin(115200);
  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }
  delay(1000);
}
void loop()
{
  /**
   * @brief  Set declination angle on your location and fix heading
   * @n      You can find your declination on: http://magnetic-declination.com/
   * @n      (+) Positive or (-) for negative
   * @n      For Bytom / Poland declination angle is 4'26E (positive)
   * @n      Formula: (deg + (min / 60.0)) / (180 / PI);
   */
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  Serial.print("X:");
  Serial.print(mag.XAxis);
  Serial.print(" Y:");
  Serial.print(mag.YAxis);
  Serial.print(" Z:");
  Serial.println(mag.ZAxis);
  Serial.print("Degress = ");
  Serial.println(mag.HeadingDegress);
  Serial.print("Direction = ");
  Serial.println(getDirection(mag.HeadingDegress));
  delay(100);
}
