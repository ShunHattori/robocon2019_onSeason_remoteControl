#pragma once

#include <Arduino.h>
#include <Wire.h>

#define gyro_adress 0x68
#define reg_accel_X 0x3B //register of accel_X_H
#define reg_accel_Y 0x3D //register of accel_Y_H
#define reg_accel_Z 0x3f //register of accel_Z_H
#define reg_temp 0x41    //register of temp
#define reg_roll 0x43    //register of gyro_X_H
#define reg_pitch 0x45   //register of gyro_Y_H
#define reg_yaw 0x47     //register of gyro_Z_H
#define reg_WAI 0x75     //register of "Who am I"

class MPU6050
{
public:
    MPU6050(uint8_t bootPin) : bootLEDPin(bootPin){};
    void setup();
    void updateIMU();
    double getYaw()
    {
        return memory_yaw / 2;
    }
    void setYaw(double userYaw)
    {
        memory_yaw = userYaw * 2;
    }

private:
    void init();
    byte read_gyro(byte);
    uint8_t bootLEDPin;
    double memory_yaw;
    double offset_yaw, dps_yaw, yawAngle;
    double outputYaw, errorYaw, targetYaw;
};