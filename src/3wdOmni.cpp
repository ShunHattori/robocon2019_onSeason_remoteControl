#include <Arduino.h>
#include <Wire.h>
#include <PS4BT.h>
#include "PS4_HEAD.h"
#include "SoftwareSerial.h"
SoftwareSerial Nucleo(22, 23);

//#define USING_6DOF_IMU
#ifdef USING_6DOF_IMU
#include "MPU6050.h"
#define IMUBootLED 33
MPU6050 myIMU(IMUBootLED);
#endif //USING_6DOF_IMU

#define USING_9DOF_IMU
#ifdef USING_9DOF_IMU
#include "MPU9250.h"
#define IMUBootLED 33
MPU9250 myIMU(IMUBootLED);
#endif //USING_9DOF_IMU

#include "OmniKinematics3WD.h"
#define MaxPWM 200
OmniKinematics3WD kinematics(MaxPWM);

#include "MotorDriverAdapter3WD.h"
MotorDriverAdapter3WD driveWheel(44, 46, 5, 6, 8, 7);

#include "MEGA2560PWMFreqencyChanger.h"

#define controllerStatsLED 35

#define CONTROLLER_CONNECTED 0x01
#define CONTROLLER_DISCONNECTED 0x02
#define OPEN_ARM_RIGHT 0x03
#define OPEN_ARM_LEFT 0x04
#define CLOSE_ARM_RIGHT 0x05
#define CLOSE_ARM_LEFT 0x06
#define EXTEND_ARM_RIGHT 0x07
#define EXTEND_ARM_LEFT 0x08
#define REDUCE_ARM_RIGHT 0x09
#define REDUCE_ARM_LEFT 0x0a

int motorOutput[3];
int outputX = 0, outputY = 0, outputYaw = 0;
bool REVERSE = 0;     //コントローラーが反転モードかどうか　OPTIONSボタンで反転＆１８０度自動旋回
bool CONTROLMODE = 1; //CONTROLMODE 1:Bottle reference, 0:Meal reference (Switch with "SHARE" button)
bool initialConnect = true;

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);

void setup()
{ // put your setup code here, to run once:
    Nucleo.begin(9600);
    Serial.begin(115200);
    while (!Serial) //waiting for opening hardware Serial port 0
    {
    }
    if (Usb.Init() == -1) //initialize USB device
    {
        Serial.print(F("\nArduino hasn't attached USB_HOST_SHIELD.\n"));
        while (1)
        {
        }
    }
    Serial.print(F("\nUSB_HOST_SHIELD detected, Success opening Serial port.\n"));

    setPwmFrequencyMEGA2560(5, 1); //initialize PWM timer Pre-Scaler
    setPwmFrequencyMEGA2560(6, 1);
    setPwmFrequencyMEGA2560(11, 1);
    setPwmFrequencyMEGA2560(44, 1);
    setPwmFrequencyMEGA2560(45, 1);
    setPwmFrequencyMEGA2560(46, 1);

    myIMU.Setup(); //initialize 9-DOF IMU sensor and calclating bias
    //kinematics.setMaxPWM(MaxPWM);        //set 3wheel direction omni kinematics MAX PWM LIMIT
    pinMode(controllerStatsLED, OUTPUT); //pinmode setup(PS4 Dual Shock Controller stats LED)
}

void loop()
{ // put your main code here, to run repeatedly:

    Usb.Task(); //running USB tasks
    if (PS4.connected())
    {
        static uint8_t redElement = 255;
        static uint8_t greenElement = 128;
        static uint8_t blueElement = 0;
        PS4.setLed(redElement, greenElement, blueElement);
        digitalWrite(controllerStatsLED, HIGH);

        outputY = (PS4.getAnalogHat(LeftHatX) - 127);
        if (-3.5 < outputY && outputY < 3.5)
        {
            outputY = 0;
        }
        outputX = (PS4.getAnalogHat(LeftHatY) - 127);
        if (-3.5 < outputX && outputX < 3.5)
        {
            outputX = 0;
        }
        outputYaw = (PS4.getAnalogButton(R2) - PS4.getAnalogButton(L2)) * 0.12;

        static unsigned long prevTime = 0;
        if ((millis() - prevTime) > 10)
        {
            Nucleo.write(CONTROLLER_CONNECTED);
            prevTime = millis();
        }
        if (CLICK_UP)
        {
            Nucleo.write(EXTEND_ARM_RIGHT);
        }
        else if (CLICK_DOWN)
        {
            Nucleo.write(REDUCE_ARM_RIGHT);
        }
        else if (CLICK_RIGHT)
        {
            //Nucleo.write('c');
        }
        else if (CLICK_LEFT)
        {
            //Nucleo.write('d');
        }
        if (CLICK_CIRCLE)
        {
            Nucleo.write(OPEN_ARM_RIGHT);
        }
        else if (CLICK_CROSS)
        {
            Nucleo.write(CLOSE_ARM_RIGHT);
        }
        if (PUSH_TRIANGLE)
        {
            analogWrite(11, 32);
            analogWrite(12, 0);
        }
        else if (PUSH_SQUARE)
        {
            analogWrite(11, 0);
            analogWrite(12, 32);
        }
        else
        {
            analogWrite(11, 0);
            analogWrite(12, 0);
        }

        kinematics.getOutput(-outputX, -outputY, outputYaw, myIMU.getYaw(), motorOutput);
        driveWheel.apply(motorOutput);

        if (PS4.getButtonPress(PS))
        {
            if (CLICK_SHARE)
            {
                Nucleo.write(CONTROLLER_DISCONNECTED);
                PS4.disconnect();
                digitalWrite(controllerStatsLED, LOW); //set controller-LED OFF
                initialConnect = true;                 //set a flag for the next connect
            }
        }
    }
}