/*
The sensor outputs provided by the library are the raw
16-bit values obtained by concatenating the 8-bit high and
low accelerometer and gyro data registers. They can be
converted to units of g and dps (degrees per second) using
the conversion factors specified in the datasheet for your
particular device and full scale setting (gain).

Example: An LSM6DS33 gives an accelerometer Z axis reading
of 16276 with its default full scale setting of +/- 2 g. The
LA_So specification in the LSM6DS33 datasheet (page 11)
states a conversion factor of 0.061 mg/LSB (least
significant bit) at this FS setting, so the raw reading of
16276 corresponds to 16276 * 0.061 = 992.8 mg = 0.9928 g.
*/
#include <Servo.h> 
#include <Wire.h>
#include <LSM6.h>
Servo myservo;
LSM6 imu;
int kiirus = 1500;

char report[80];

void setup()
{
  Serial.begin(9600);
  myservo.attach(5);
  myservo.writeMicroseconds(kiirus);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
}

void loop()
{
  imu.read();

  Serial.print(imu.a.x);
  Serial.print(" ");
  Serial.print(imu.a.y);
  Serial.print(" ");
  Serial.print(imu.a.z);
  Serial.print(" ");
  Serial.print(imu.g.x);
  Serial.print(" ");
  Serial.print(imu.g.y);
  Serial.print(" ");
  Serial.println(imu.g.z);
  kiirus = map(imu.a.x, -6000, 16000, 1400, 1600);
  myservo.writeMicroseconds(kiirus);
  delay(100);
}
