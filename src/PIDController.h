#pragma once

class PIDController
{
public:
  PIDController(double pGain, double iGain, double dGain);
  PIDController(double pGain);                            
  
  typedef enum gainType
  {
    proportional,
    integral,
    differential,
  } gainType;

  void update(double target, double current);

  double getTerm();

  double getTerm(gainType);

  void modifyGain(gainType, double);

  void setOutputLimit(double);

private:
  struct
  {
    double gain, term, outMax;
    bool isEnable;
  } p, i, d;

  double errorStack; // 毎回のターゲット値と現在値の差の蓄積(I制御で使用)
  double prevError;  // 前回の偏差
  double maxValue;   // 容認する最大出力
};
