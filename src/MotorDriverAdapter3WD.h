#pragma once

#include <Arduino.h>

class MotorDriverAdapter3WD
{
public:
  /*
    *   desc:   出力ピンを設定し初期化処理を行う
    *   param:  出力するピンx6
    *   return: none
    */
  MotorDriverAdapter3WD(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

  /*
    *   desc:   入力されたPWMを元にピンに出力する
    *   param:  pwm[3](int)
    *   return: none
    */
  void apply(int pwm[3]);

private:
  uint8_t FCWPin, FCCWPin, BRCWPin, BRCCWPin, BLCWPin, BLCCWPin;
  float duty[3];

  const float RCconstant = 0.90;
  float prevPWM[3];
};