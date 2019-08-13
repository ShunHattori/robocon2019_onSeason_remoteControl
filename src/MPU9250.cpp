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

  Wire.beginTransmission(mag_address);
  Wire.write(0x0A);
  Wire.write(0X06);
  Wire.endTransmission();

  while (read_I2C(reg_WAI) != 0x71)
  {
    Serial.println("IMU couldn'd initialized, check your wire connections.");
  }

  for (int i = 0; i < 3000; i++)
  {
    int16_t raw_gx, raw_gy, raw_gz;
    int16_t raw_mx, raw_my, raw_mz;
    int16_t magdata[7];
    int count = 0;
    Wire.beginTransmission(gyro_address);
    Wire.write(reg_roll);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 6);

    raw_gx = Wire.read() << 8 | Wire.read();
    raw_gy = Wire.read() << 8 | Wire.read();
    raw_gz = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(mag_address);
    Wire.write(reg_mag);
    Wire.endTransmission();
    Wire.requestFrom(0x0c, 7);

    while (Wire.available())
    {
      magdata[count++] = Wire.read();
    }

    if (!(magdata[6] & 0x08))
    {
      raw_mx = magdata[1] << 8 | magdata[0];
      raw_my = magdata[3] << 8 | magdata[2];
      raw_mz = magdata[5] << 8 | magdata[4];
    }

    offset_gx += raw_gx / 131.0;
    offset_gy += raw_gy / 131.0;
    offset_gz += raw_gz / 131.0;
    offset_mx += (double)raw_mx * 0.15;
    offset_my += (double)raw_my * 0.15;
    offset_mz += (double)raw_mz * 0.15;
  }

  offset_gx /= 3000;
  offset_gy /= 3000;
  offset_gz /= 3000;
  offset_mx /= 3000;
  offset_my /= 3000;
  offset_mz /= 3000;
  offset_mag = atan2(offset_mx, offset_my) * RAD_TO_DEG;
  if (offset_mag >= 0)
  {
    offset_mag_plus = offset_mag;
    offset_mag_minus = offset_mag - 180;
  }
  else
  {
    offset_mag_plus = offset_mag + 180;
    offset_mag_minus = offset_mag;
  }
  digitalWrite(LEDpin, HIGH);
}

double MPU9250::complement_Yaw()
{
  gyro_yaw = gyro_Yaw();
  compass_angle = compass_Yaw();
  complement_angle = 0.9 * gyro_yaw + 0.1 * compass_angle;
  return complement_angle;
}

void MPU9250::read_accel(double *accel_roll, double *accel_pitch)
{
  Wire.beginTransmission(gyro_address);
  Wire.write(reg_accel_X);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 6);
  if (Wire.available())
  {
    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
  }
  *accel_roll = atan2(ay, az);
  *accel_pitch = atan2(ax, sqrt(pow(ay, 2) + pow(az, 2)));
}

void MPU9250::read_gyro(double *gyro_roll, double *gyro_pitch, double *gyro_yaw)
{
  static double roll = 0, pitch = 0, yaw = 0;

  Wire.beginTransmission(gyro_address);
  Wire.write(reg_roll);
  Wire.endTransmission(false);
  Wire.requestFrom(gyro_address, 6, true);

  if (Wire.available())
  {
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
  }

  elapsed_time = millis() - preterit_time;
  preterit_time = millis();

  dps_gx = (double)gx / 131;
  dps_gy = (double)gy / 131;
  dps_gz = (double)gz / 131;

  read_accel(&accel_roll, &accel_pitch);
  accel_roll = accel_roll * RAD_TO_DEG;
  accel_pitch = accel_pitch * RAD_TO_DEG;
  if (!(dps_gx - offset_gx < 1 && dps_gx - offset_gx > -1))
    roll += (dps_gx - offset_gx) * elapsed_time * 0.001;

  if (!(dps_gy - offset_gy < 1 && dps_gy - offset_gy > -1))
    pitch += (dps_gy - offset_gy) * elapsed_time * 0.001;

  if (!(dps_gz - offset_gz < 1 && dps_gz - offset_gz > -1))
    yaw += (dps_gz - offset_gz) * elapsed_time * 0.001;

  *gyro_roll = 0.995 * roll + (0.005 * accel_roll);
  *gyro_pitch = 0.995 * pitch + (0.005 * accel_pitch);

  if (yaw > 180)
  {
    *gyro_yaw = -180 + (yaw - 180);
    if (yaw > 360)
      yaw = yaw - 360;
  }
  else if (yaw < -180)
  {
    *gyro_yaw = 180 + (yaw + 180);
    if (yaw < 360)
      yaw = yaw + 360;
  }
  else
    *gyro_yaw = yaw;
}

double MPU9250::gyro_Yaw()
{
  static double yaw = 0, yaw_angle = 0;
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
  return yaw;
  /*if (yaw > 180)
  {
    yaw_angle = -180 + (yaw - 180);
    if (yaw > 360)
      yaw = yaw - 360;
  }
  else if (yaw < -180)
  {
    yaw_angle = 180 + (yaw + 180);
    if (yaw < -360)
      yaw = yaw + 360;
  }
  else
    yaw_angle = yaw;
  return yaw_angle;*/
}

void MPU9250::read_compass(double *MX, double *MY, double *MZ)
{
  int count = 0;
  Wire.beginTransmission(mag_address);
  Wire.write(reg_mag);
  Wire.endTransmission();
  Wire.requestFrom(0x0C, 7);

  while (Wire.available())
  {
    compassData[count++] = Wire.read();
  }
  if (compassData[6] != 0x18)
  {
    mx = compassData[1] << 8 | compassData[0];
    my = compassData[3] << 8 | compassData[2];
    mz = compassData[5] << 8 | compassData[4];
    tmx = (double)mx * 0.15;
    tmy = (double)my * 0.15;
    tmz = (double)mz * 0.15;
  }

  *MX = tmx;
  *MY = tmy;
  *MZ = tmz;
}

double MPU9250::compass_Yaw()
{
  double yaw;
  int count = 0;
  Wire.beginTransmission(mag_address);
  Wire.write(reg_mag);
  Wire.endTransmission();
  Wire.requestFrom(0x0C, 7);

  while (Wire.available())
  {
    compassData[count++] = Wire.read();
  }
  if (compassData[6] != 0x18)
  {
    mx = compassData[1] << 8 | compassData[0];
    my = compassData[3] << 8 | compassData[2];
    tmx = (double)mx * 0.15;
    tmy = (double)my * 0.15;
  }
  yaw = atan2(tmx, tmy) * RAD_TO_DEG;

  Serial.print(yaw);
  if (offset_mag >= 0)
  {
    Serial.print(yaw - offset_mag_plus);
    if (yaw - offset_mag_plus < -180)
      yaw = 180 + (yaw - offset_mag_minus);
    else
      yaw = yaw - offset_mag_plus;
  }
  else
  {
    if (yaw - offset_mag_minus > 180)
      yaw = -180 + (yaw - offset_mag_plus);
    else
      yaw = (yaw - offset_mag_minus);
  }
  Serial.print("comapss_yaw = ");
  Serial.print(yaw);
  return yaw;
}
