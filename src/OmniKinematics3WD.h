#pragma once

#include <Arduino.h>

class OmniKinematics3WD
{
public:
  /*
    *   desc:   三輪ロボットコンストラクタ
    *   param:  e.g. OmniKinematics3WD Omni;
    */
  OmniKinematics3WD(int max = 32) : maxAllocateOutput(max){};

  /*
    *   desc:   MDにかかる最大PWMを設定
    *   param:  maxPWM(int)
    *   return: none
    */
  void setMaxPWM(int maxPwm)
  {
    maxAllocateOutput = maxPwm;
  }

  /*
    *   desc:   ロボット全体の移動方向を指定し、MDにかけるPWMを取得
    *   param:  x,y,yaw方向の移動量(int) 駆動輪数に応じたint配列
    *   return: none(引数に代入)
    */
  void getOutput(int x, int y, int yaw, float yawAngle, int pwm[]);

private:
  int XVector, YVector, YawVector;
  float YawAngle, userBias;
  int maxAllocateOutput;
};