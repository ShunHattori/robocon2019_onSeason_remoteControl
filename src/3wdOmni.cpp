#include "PS4_HEAD.h"
#include <Arduino.h>
#include <PS4BT.h>
#include <Wire.h>


//#define USING_6DOF_IMU
#ifdef USING_6DOF_IMU
#include "MPU6050.h"
#define IMUBootLED 33
MPU6050 myIMU(IMUBootLED);
#endif // USING_6DOF_IMU

#define USING_9DOF_IMU
#ifdef USING_9DOF_IMU
#include "MPU9250.h"
#define IMUBootLED 26
MPU9250 myIMU(IMUBootLED);
#endif // USING_9DOF_IMU

#include "OmniKinematics3WD.h"
#define MaxPWM 250
OmniKinematics3WD kinematics(MaxPWM);

#include "PagodaUnitProtocol.hpp"
PagodaUnitProtocol Nucleo(&Serial3);

#define controllerStatsLED 25

typedef enum cmd {
  CONTROLLER_CONNECTED = 0x01,
  CONTROLLER_DISCONNECTED = 0x02,
  OPEN_ARM_RIGHT = 0x03,
  OPEN_ARM_LEFT = 0x04,
  CLOSE_ARM_RIGHT = 0x05,
  CLOSE_ARM_LEFT = 0x06,
  EXTEND_ARM_RIGHT = 0x07,
  EXTEND_ARM_LEFT = 0x08,
  REDUCE_ARM_RIGHT = 0x09,
  REDUCE_ARM_LEFT = 0x0a
} CmdTypes;

int motorOutput[3];
double rawPWM[3];
bool REVERSE =
    0;                //コントローラーが反転モードかどうか　OPTIONSボタンで反転＆１８０度自動旋回
bool CONTROLMODE = 1; // CONTROLMODE 1:Bottle reference, 0:Meal reference
                      // (Switch with "SHARE" button)
bool initialConnect = true;

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);

/*
            circle:右側開く
            triangle:左側開く
            cross:右側閉じる
            square:左側閉じる
            up:右側伸ばす
            right:右側縮小
            left:左側伸ばす
            down:左側縮小
 */
void ButtonClick(bool *state) {
  if (PS4.getButtonPress(CIRCLE)) {
    state[0] = 1;
  } else if (PS4.getButtonPress(CROSS)) {
    state[0] = 0;
  } else if (PS4.getButtonPress(TRIANGLE)) {
    state[1] = 1;
  } else if (PS4.getButtonPress(SQUARE)) {
    state[1] = 0;
  } else if (PS4.getButtonPress(UP)) {
    state[2] = 1;
  } else if (PS4.getButtonPress(RIGHT)) {
    state[2] = 0;
  } else if (PS4.getButtonPress(LEFT)) {
    state[3] = 1;
  } else if (PS4.getButtonPress(DOWN)) {
    state[3] = 0;
  }
  return;
}

void setup() { // put your setup code here, to run once:
  Serial3.begin(115200);
  Serial.begin(115200);
  while (!Serial) // waiting for opening hardware Serial port 0
  {
  }
  if (Usb.Init() == -1) // initialize USB device
  {
    Serial.print(F("\nArduino hasn't attached USB_HOST_SHIELD.\n"));
    while (1) {
    }
  }
  Serial.print(F("\nUSB_HOST_SHIELD detected, Success opening Serial port.\n"));
  myIMU.Setup(); // initialize 9-DOF IMU sensor and calclating bias
  pinMode(controllerStatsLED,
          OUTPUT); // pinmode setup(PS4 Dual Shock Controller stats LED)
}

void loop() {
  Usb.Task(); // running USB tasks
  if (PS4.connected()) {
    digitalWrite(controllerStatsLED, HIGH);
    /*static uint8_t redElement = 255;
       static uint8_t greenElement = 128;
       static uint8_t blueElement = 0;
       PS4.setLed(redElement, greenElement, blueElement);
       digitalWrite(controllerStatsLED, HIGH);*/

    /*
        circle:右側開く
        triangle:左側開く
        cross:右側閉じる
        square:左側閉じる
        up:右側伸ばす
        down:左側縮小
        right:右側縮小
        left:左側伸ばす
     */
    // static bool armState[4];
    // ButtonClick(armState); //states store in armstate array

    static double multiply = 0.3;
    if (PS4.getButtonPress(OPTIONS) && PS4.getButtonClick(R1))
      multiply += 0.05;
    else if (PS4.getButtonPress(OPTIONS) && PS4.getButtonClick(L1))
      multiply -= 0.05;

    multiply = (multiply > 1.0) ? 1.0 : (multiply < 0) ? 0 : multiply;

    rawPWM[0] = (PS4.getAnalogHat(LeftHatX) - 127) * multiply;
    if (-3.5 < rawPWM[0] && rawPWM[0] < 3.5) {
      rawPWM[0] = 0;
    }
    rawPWM[1] = (PS4.getAnalogHat(LeftHatY) - 127) * multiply;
    if (-3.5 < rawPWM[1] && rawPWM[1] < 3.5) {
      rawPWM[1] = 0;
    }
    static double RCconstant = 0.92;
    static double modifiedPWM[2], prevPWM[2];
    for (int i = 0; i < 2; i++) {
      modifiedPWM[i] =
          (RCconstant * prevPWM[i]) + ((1 - RCconstant) * rawPWM[i]);
      prevPWM[i] = modifiedPWM[i];
    }

    rawPWM[2] = (PS4.getAnalogButton(R2) - PS4.getAnalogButton(L2)) * 0.04;

    kinematics.getOutput(modifiedPWM[0], modifiedPWM[1], -rawPWM[2],
                         myIMU.gyro_Yaw(), motorOutput);
    /*
            int term = 10;
            int packet[term] = {
                motorOutput[0] < 0 ? 0 : motorOutput[0],
                motorOutput[0] > 0 ? 0 : -motorOutput[0],
                motorOutput[1] < 0 ? 0 : motorOutput[1],
                motorOutput[1] > 0 ? 0 : -motorOutput[1],
                motorOutput[2] < 0 ? 0 : motorOutput[2],
                motorOutput[2] > 0 ? 0 : -motorOutput[2],
                armState[0],
                armState[1],
                armState[2],
                armState[3],
            };
     */
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

    if (PS4.getButtonPress(PS)) {
      if (CLICK_SHARE) {
        PS4.disconnect();
        digitalWrite(controllerStatsLED, LOW); // set controller-LED OFF
        initialConnect = true; // set a flag for the next connect
      }
    }
  }
}
