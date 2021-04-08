/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low magnetometer data registers.
They can be converted to units of gauss using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).

Example: An LIS3MDL gives a magnetometer X axis reading of 1292 with its
default full scale setting of +/- 4 gauss. The GN specification
in the LIS3MDL datasheet (page 8) states a conversion factor of 6842
LSB/gauss (where LSB means least significant bit) at this FS setting, so the raw
reading of 1292 corresponds to 1292 / 6842 = 0.1888 gauss.
*/

#include <Wire.h>
#include <LIS3MDL.h>
#include <LPS.h>
#include <LSM6.h>
char report[80];

LSM6 imu;
LPS ps;

LIS3MDL mag;


void setup()
{
  Serial.begin(9600);
  Wire.begin();

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  mag.enableDefault();
  ps.enableDefault();
}

void loop()
{
  mag.read();

  snprintf(report, sizeof(report), "M: %6dgauss %6dgauss %6dgauss",
    mag.m.x/6842.0, mag.m.y/6842.0, mag.m.z/6842.0);
  Serial.println(report);

  delay(1000);



  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  float temperature = ps.readTemperatureC();
  
  Serial.print("p: ");
  Serial.print(pressure);
  Serial.print(" mbar\ta: ");
  Serial.print(altitude);
  Serial.print(" m\tt: ");
  Serial.print(temperature);
  Serial.println(" deg C");

  delay(1000);


imu.read();

  snprintf(report, sizeof(report), "A: %6dDPS  %6dDPS %6dDPS   G: %6dm/s2 %6dm/s2 %6dm/s2",
    imu.a.x* 0.061, imu.a.y* 0.061, imu.a.z* 0.061,
    imu.g.x * 8.75, imu.g.y* 8.75, imu.g.z* 8.75);
  Serial.println(report);

  delay(1000);
  
}
