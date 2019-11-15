#ifndef MPU9250_h
#define MPU9250_h

#include <Wire.h>
#include "Arduino.h"

class MPU9250
{
public:
  MPU9250(int baud)
  {
    Wire.begin();
    Wire.setClock(baud);
  }
  void addStatsLED(int pin)
  {
    LEDpin = pin;
    pinMode(LEDpin, OUTPUT);
  }
  void calibration();                                                      //センサーの設定＋オフセット値の取得

  double getYaw()
  {
    update();
    return yaw - userOffset;
  }
  void setYaw(double targetYaw) { userOffset = targetYaw; }
  void setBias(double yaw) { userOffset += yaw; }

private:
  byte read_I2C(byte reg);
  int LEDpin;

  void update(); //ジャイロセンサから算出したYaw軸の回転角を返す
  enum address
  {
    gyro_address = 0x68, //address of MPU9025(加速度、ジャイロ)
    reg_yaw = 0x47,      //register of gyro_Z_H
    reg_WAI = 0x75,      //register of "Who am I"
  };

  double  offset_gz = 0;
  double gyro_yaw = 0;
  double yaw = 0;

  unsigned long int elapsed_time = 0, preterit_time = 0;
  int16_t gx, gy, gz;
  double dps_gx, dps_gy, dps_gz;
  double userOffset = 0;
};
#endif
