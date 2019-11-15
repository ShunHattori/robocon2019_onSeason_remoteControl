#include "MPU9250.h"
byte MPU9250::read_I2C(byte reg)
{
  byte data;
  Wire.beginTransmission(gyro_address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 1);
  data = Wire.read();
  return data;
}

void MPU9250::calibration()
{
  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();

  while (read_I2C(reg_WAI) != 0x71)
  {
    Serial.println("IMU couldn'd initialized, check your wire connections.");
  }

  for (int i = 0; i < 3000; i++)
  {
    int16_t raw_gz;
    Wire.beginTransmission(gyro_address);
    Wire.write(reg_yaw);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 2);

    raw_gz = Wire.read() << 8 | Wire.read();

    offset_gz += raw_gz;
  }

  offset_gz /= 3000 * 131.0f;

  digitalWrite(LEDpin, HIGH);
}


void MPU9250::update()
{
  Wire.beginTransmission(gyro_address);
  Wire.write(reg_yaw);
  Wire.endTransmission(false);
  Wire.requestFrom(gyro_address, 2, true);

  if (Wire.available())
  {
    gz = Wire.read() << 8 | Wire.read();
  }

  elapsed_time = millis() - preterit_time;
  preterit_time = millis();

  dps_gz = (double)gz / 131;
  if (!(dps_gz - offset_gz < 1 && dps_gz - offset_gz > -1))
    yaw += (dps_gz - offset_gz) * elapsed_time * 0.001;
}
