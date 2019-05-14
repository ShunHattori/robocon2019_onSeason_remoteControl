#include <Arduino.h>
#include <Wire.h>
#include <PS4BT.h>
#include "PS4_HEAD.h"

#define MaxPWM 40

//#include "MPU6050.h"
//#define gyroBootLED 33
//MPU6050 myIMU(gyroBootLED);

#include "MPU9250.h"
MPU9250 imu;

#include "OmniKinematics3WD.h"
OmniKinematics3WD kinematics(MaxPWM);

#include "MotorDriverAdapter3WD.h"
MotorDriverAdapter3WD driveWheel(44, 46, 5, 6, 8, 7);

#include "MEGA2560PWMFreqencyChanger.h"

#define controllerStatsLED 35

int motorOutput[3];
int outputX = 0, outputY = 0, outputYaw = 0;
bool REVERSE = 0;     //コントローラーが反転モードかどうか　OPTIONSボタンで反転＆１８０度自動旋回
bool CONTROLMODE = 1; //CONTROLMODE 1:Bottle reference, 0:Meal reference (Switch with "SHARE" button)
bool initialConnect = true;

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    while (!Serial)
        ;
    if (Usb.Init() == -1)
    {
        Serial.print(F("\nArduino hasn't attached USB_HOST_SHIELD.\n"));
        while (1)
            ;
    }
    Serial.print(F("\nUSB_HOST_SHIELD detected, Success opening Serial port.\n"));

    setPwmFrequencyMEGA2560(5, 1);
    setPwmFrequencyMEGA2560(6, 1);
    setPwmFrequencyMEGA2560(11, 1);
    setPwmFrequencyMEGA2560(44, 1);
    setPwmFrequencyMEGA2560(45, 1);
    setPwmFrequencyMEGA2560(46, 1);
    imu.Setup(); //it takes a while
    kinematics.setMaxPWM(MaxPWM);
    pinMode(controllerStatsLED, OUTPUT);
}

void loop()
{

    Usb.Task();
    // put your main code here, to run repeatedly:
    if (PS4.connected())
    {
        digitalWrite(controllerStatsLED, HIGH);
        if (initialConnect)
        {
            initialConnect = false;
            if (!REVERSE)
            {
                imu.setYaw(0);
            }
        }

        if (PUSH_RIGHT)
        {
            outputX = -MaxPWM;
            outputY = 0;
        }
        else if (PUSH_LEFT)
        {
            outputX = MaxPWM;
            outputY = 0;
        }
        else if (PUSH_UP)
        {
            outputX = 0;
            outputY = -MaxPWM;
        }
        else if (PUSH_DOWN)
        {
            outputX = 0;
            outputY = MaxPWM;
        }
        else
        {
            outputX = 0;
            outputY = 0;
        }

        //myIMU.updateIMU();
        if (PUSH_R2)
        {
            outputYaw = MaxPWM;
            imu.setYaw(0);
        }
        else if (PUSH_L2)
        {
            outputYaw = -MaxPWM;
            imu.setYaw(0);
        }
        else
        {
            double errorYaw = -imu.gyro_Yaw();
            outputYaw = errorYaw * 5;
            outputYaw = constrain(outputYaw, -MaxPWM, MaxPWM);
            if (-5 < outputYaw && outputYaw < 5)
            {
                outputYaw = 0;
            }
            outputX = (outputX * cos(errorYaw * DEG_TO_RAD)) + (outputY * sin(errorYaw * DEG_TO_RAD));
            outputY = (outputX * sin(errorYaw * DEG_TO_RAD)) + (outputY * cos(errorYaw * DEG_TO_RAD));
        }
        kinematics.getOutput(outputX, outputY, outputYaw, motorOutput);
        driveWheel.apply(motorOutput);

        if (PS4.getButtonPress(PS))
        {
            if (CLICK_SHARE)
            {
                PS4.disconnect();
                digitalWrite(controllerStatsLED, LOW);
                initialConnect = true;
                kinematics.getOutput(0, 0, 0, motorOutput);
                driveWheel.apply(motorOutput);
            }
        }
    }
    else
    {
        kinematics.getOutput(0, 0, 0, motorOutput);
        driveWheel.apply(motorOutput);
    }
}

/*
void LEFTconvertStickValue() {
    HatX_S  = (PS4.getAnalogHat(LeftHatX) - 127 ) *multi;
    if(-10 < HatX_S && HatX_S < 10) HatX_S = 0;
    HatY_S  = (PS4.getAnalogHat(LeftHatY) - 127 ) *multi;
    if(-10 < HatY_S && HatY_S < 10) HatY_S = 0;
    //R2Hat_S =  PS4.getAnalogButton(L2) - PS4.getAnalogButton(R2);
    if(PUSH_L1) memory_yaw += 0.2;
    if(PUSH_R1) memory_yaw -= 0.2;
    errorYaw = -(targetYaw + getYawAngle('n'));
    outputYaw = errorYaw * 2;
    outputYaw = constrain(outputYaw,-MaxAllocateOutput,MaxAllocateOutput);
    if(-10 < outputYaw && outputYaw < 10) outputYaw = 0;

    outputX = (HatX_S * cos(errorYaw * DEG_TO_RAD)) + (HatY_S * sin(errorYaw * DEG_TO_RAD));
    outputY = (HatX_S * sin(errorYaw * DEG_TO_RAD)) + (HatY_S * cos(errorYaw * DEG_TO_RAD));
}
void RIGHTconvertStickValue() {
    HatX_S  = (PS4.getAnalogHat(RightHatX) - 127 ) *multi;
    if(-10 < HatX_S && HatX_S < 10) HatX_S = 0;
    HatY_S  = (PS4.getAnalogHat(RightHatY) - 127 ) *multi;
    if(-10 < HatY_S && HatY_S < 10) HatY_S = 0;
    //R2Hat_S =  PS4.getAnalogButton(L2) - PS4.getAnalogButton(R2);
    if(PUSH_L1) memory_yaw += 0.05;
    if(PUSH_R1) memory_yaw -= 0.05;
    errorYaw = -(targetYaw + getYawAngle('n'));
    outputYaw = errorYaw * 2;
    outputYaw = constrain(outputYaw,-MaxAllocateOutput,MaxAllocateOutput);
    if(-10 < outputYaw && outputYaw < 10) outputYaw = 0;

    outputX = (HatX_S * cos(errorYaw * DEG_TO_RAD)) + (HatY_S * sin(errorYaw * DEG_TO_RAD));
    outputY = (HatX_S * sin(errorYaw * DEG_TO_RAD)) + (HatY_S * cos(errorYaw * DEG_TO_RAD));
}
*/