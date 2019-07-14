#include <Arduino.h>
#include <Wire.h>
#include <PS4BT.h>
#include "PS4_HEAD.h"

//#define USING_6DOF_IMU
#ifdef USING_6DOF_IMU
#include "MPU6050.h"
#define IMUBootLED 33
MPU6050 myIMU(IMUBootLED);
#endif //USING_6DOF_IMU

#define USING_9DOF_IMU
#ifdef USING_9DOF_IMU
#include "MPU9250.h"
#define IMUBootLED 26
MPU9250 myIMU(IMUBootLED);
#endif //USING_9DOF_IMU

#include "OmniKinematics3WD.h"
#define MaxPWM 200
OmniKinematics3WD kinematics(MaxPWM);

#include "PagodaUnitProtocol.hpp"
PagodaUnitProtocol Nucleo(&Serial1);

#define controllerStatsLED 25

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
    Serial1.begin(115200);
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

    myIMU.Setup(); //initialize 9-DOF IMU sensor and calclating bias
    //kinematics.setMaxPWM(MaxPWM);        //set 3wheel direction omni kinematics MAX PWM LIMIT
    pinMode(controllerStatsLED, OUTPUT); //pinmode setup(PS4 Dual Shock Controller stats LED)
}

void loop()
{ // put your main code here, to run repeatedly:

    Usb.Task(); //running USB tasks
    if (PS4.connected())
    {
        digitalWrite(controllerStatsLED, HIGH);
        /*static uint8_t redElement = 255;
           static uint8_t greenElement = 128;
           static uint8_t blueElement = 0;
           PS4.setLed(redElement, greenElement, blueElement);
           digitalWrite(controllerStatsLED, HIGH);*/

        outputX = (PS4.getAnalogHat(LeftHatX) - 127) * 0.3;
        if (-3.5 < outputX && outputX < 3.5)
        {
            outputX = 0;
        }
        outputY = (PS4.getAnalogHat(LeftHatY) - 127) * 0.3;
        if (-3.5 < outputY && outputY < 3.5)
        {
            outputY = 0;
        }
        outputYaw = (PS4.getAnalogButton(R2) - PS4.getAnalogButton(L2)) * 0.10;

        kinematics.getOutput(outputX, outputY, outputYaw, myIMU.gyro_Yaw(), motorOutput);

        int term = 6;
        int packet[term] = {
            motorOutput[0] < 0 ? 0 : motorOutput[0],
            motorOutput[0] > 0 ? 0 : -motorOutput[0],
            motorOutput[1] < 0 ? 0 : motorOutput[1],
            motorOutput[1] > 0 ? 0 : -motorOutput[1],
            motorOutput[2] < 0 ? 0 : motorOutput[2],
            motorOutput[2] > 0 ? 0 : -motorOutput[2],
        };
        Nucleo.transmit(term, packet);

        if (PS4.getButtonPress(PS))
        {
            if (CLICK_SHARE)
            {
                PS4.disconnect();
                digitalWrite(controllerStatsLED, LOW); //set controller-LED OFF
                initialConnect = true;                 //set a flag for the next connect
            }
        }
    }
}
