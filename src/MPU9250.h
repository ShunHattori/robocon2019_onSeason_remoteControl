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
  void read_accel(double *accel_roll, double *accel_pitch);                //指定したアドレスに加速度センサから算出したroll軸,pitch軸の回転角を返す
  void read_gyro(double *gyro_roll, double *gyro_pitch, double *gyro_yaw); //指定したアドレスにジャイロセンサから算出した各軸の回転角を返す
  void read_compass(double *Mx, double *My, double *Mz);                   //指定したアドレスに地磁気センサの生値を返す
  double complement_Yaw();                                                 //各センサから補正したYaw軸の回転角を返す
  double gyro_Yaw();                                                       //ジャイロセンサから算出したYaw軸の回転角を返す
  double compass_Yaw();                                                    //地磁気センサから算出したYaw軸の回転角を返す
  double getYaw()
  {
    //gyro_yaw = gyro_Yaw();
    //compass_angle = compass_Yaw();
    //yaw = 0.9 * gyro_yaw + 0.1 * compass_angle;
    yaw = gyro_Yaw();
    return yaw - userOffset;
  }
  void setYaw(double targetYaw) { userOffset = targetYaw; }
  void setBias(double yaw) { userOffset += yaw; }

private:
  byte read_I2C(byte reg);
  int LEDpin;
  enum address
  {
    gyro_address = 0x68, //address of MPU9025(加速度、ジャイロ)
    mag_address = 0x0c,  //address of AK8963(地磁気)
    reg_accel_X = 0x3B,  //register of accel_X_H 未使用
    reg_accel_Y = 0x3D,  //register of accel_Y_H 未使用
    reg_accel_Z = 0x3f,  //register of accel_Z_H
    reg_temp = 0x41,     //register of temp 未使用
    reg_roll = 0x43,     //register of gyro_X_H 未使用
    reg_pitch = 0x45,    //register of gyro_Y_H 未使用
    reg_yaw = 0x47,      //register of gyro_Z_H
    reg_WAI = 0x75,      //register of "Who am I"
    reg_mag = 0x03,      //reister of Magnetometer
  };

  double offset_gx = 0, offset_gy = 0, offset_gz = 0;
  double offset_mx = 0, offset_my = 0, offset_mz = 0;
  double offset_mag, offset_mag_plus, offset_mag_minus;
  double gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
  double Mx, My, Mz;
  double compass_angle;
  double yaw = 0;

  double complement_angle;
  int16_t ax, ay, az;
  double accel_roll, accel_pitch;
  unsigned long int elapsed_time = 0, preterit_time = 0;
  int16_t gx, gy, gz;
  double dps_gx, dps_gy, dps_gz;
  int16_t mx, my, mz;
  int16_t tmx, tmy, tmz;
  int16_t compassData[7];
  double userOffset = 0;
};
#endif
