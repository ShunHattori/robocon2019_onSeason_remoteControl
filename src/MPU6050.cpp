#include "MPU6050.h"

void MPU6050::init()
{
    memory_yaw = 0;
    offset_yaw = 0;
    dps_yaw = 0;
    yawAngle = 0;
    outputYaw = 0;
    errorYaw = 0;
    targetYaw = 0;
}

byte MPU6050::read_gyro(byte reg)
{
    byte data;
    Wire.beginTransmission(gyro_adress);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(gyro_adress, 1);
    data = Wire.read();
    return data;
}

void MPU6050::setup()
{
    init();
    Wire.begin();
    Wire.setClock(400000L);
    Wire.beginTransmission(gyro_adress);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    if (read_gyro(reg_WAI) != 0x68)
    {
        Serial.println("connect Error");
        while (true)
        {
        }
    }
    Serial.println("Calibration start");
    for (int i = 0; i < 3000; i++)
    {
        int16_t raw_yaw;
        Wire.beginTransmission(gyro_adress);
        Wire.write(reg_yaw);
        Wire.endTransmission(false);
        Wire.requestFrom(gyro_adress, 2, true);
        raw_yaw = Wire.read() << 8 | Wire.read();
        dps_yaw = (double)raw_yaw / 65.5;
        offset_yaw += dps_yaw;
    }
    offset_yaw /= 3000;
    Serial.println("Device ready");
    pinMode(bootLEDPin, OUTPUT);
    digitalWrite(bootLEDPin, HIGH);
}

void MPU6050::updateIMU()
{
    int16_t raw_yaw;
    static unsigned long elapsed_time, preterit_time = 0;
    Wire.beginTransmission(gyro_adress);
    Wire.write(reg_yaw);
    Wire.endTransmission(false);
    Wire.requestFrom(gyro_adress, 2, true);
    raw_yaw = Wire.read() << 8 | Wire.read();
    dps_yaw = (double)raw_yaw / 65.5;
    elapsed_time = millis() - preterit_time;
    preterit_time = millis();
    memory_yaw += (dps_yaw - offset_yaw) * (elapsed_time * 0.001); //(degree / sec) * sec
}