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

    myIMU.Setup();                       //initialize 9-DOF IMU sensor and calclating bias
    //kinematics.setMaxPWM(MaxPWM);        //set 3wheel direction omni kinematics MAX PWM LIMIT
    pinMode(controllerStatsLED, OUTPUT); //pinmode setup(PS4 Dual Shock Controller stats LED)
}

void loop()
{ // put your main code here, to run repeatedly:

    Usb.Task(); //running USB tasks
    if (PS4.connected())
    {
        static uint8_t redElement = 255;
        static uint8_t greenElement = 0;
        static uint8_t blueElement = 0;
        PS4.setLed(redElement, greenElement, blueElement);
        digitalWrite(controllerStatsLED, HIGH);
        /*if (initialConnect)
        {
            initialConnect = false;
            if (!REVERSE)
            {
                myIMU.setYaw(0);
            }
        }*/
        outputX = (PS4.getAnalogHat(LeftHatX) - 127);
        if (-3.5 < outputX && outputX < 3.5)
        {
            outputX = 0;
        }
        outputY = (PS4.getAnalogHat(LeftHatY) - 127);
        if (-3.5 < outputY && outputY < 3.5)
        {
            outputY = 0;
        }
        outputYaw = (PS4.getAnalogButton(R2) - PS4.getAnalogButton(L2)) * 0.12;
        if (CLICK_UP)
        {
            Nucleo.write('a');
        }
        else if (CLICK_DOWN)
        {
            Nucleo.write('b');
        }
        else if (CLICK_RIGHT)
        {
            Nucleo.write('c');
        }
        else if (CLICK_LEFT)
        {
            Nucleo.write('d');
        }
        if (CLICK_CIRCLE)
        {
            Nucleo.write('e');
        }
        /*else if(CLICK_CROSS){
                        Nucleo.write('d');
        }*/
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
        /*if (PUSH_RIGHT)
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

        if (PUSH_R2)
        {
            outputYaw = MaxPWM / 1.5;
        }
        else if (PUSH_L2)
        {
            outputYaw = -MaxPWM / 1.5;
        }
        else
        {
            outputYaw = 0;
        }*/

        kinematics.getOutput(-outputX, outputY, outputYaw, myIMU.getYaw(), motorOutput);
        driveWheel.apply(motorOutput);

        if (PS4.getButtonPress(PS))
        {
            if (CLICK_SHARE)
            {
                PS4.disconnect();
                digitalWrite(controllerStatsLED, LOW); //set controller-LED OFF
                initialConnect = true;                 //set a flag for the next connect
                //kinematics.getOutput(0, 0, 0, 0, motorOutput);
                //driveWheel.apply(motorOutput);
            }
        }
    }
    else
    {                                                  //apply to motor driver stop by disconnecting PS4 Controller
        //kinematics.getOutput(0, 0, 0, 0, motorOutput); //calculate PWMs from 3-DOF vectors by kinematics
        //driveWheel.apply(motorOutput);                 //apply PWMs to motor drivers
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