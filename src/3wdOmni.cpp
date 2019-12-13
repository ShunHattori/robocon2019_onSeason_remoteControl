#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>
#include "PWMfrequency.h"

typedef enum statsLEDs
{
  No1 = 22,
  No2,
  No3,
  No4,
  No5,
} onBoardLEDs;

struct
{
  const int armUpDownCW = 8;
  const int armUpDownCCW = 7;
} RobotArmMotorPin;

struct
{
  const int VROut = A0;
} RobotArmVRPin;

struct
{
  const long HardwareSerial = 256000; //256kHz
  const int SoftwareSerial = 38400;
  const long I2C = 400000; //400kHz
} SerialBaud;

struct parameter
{
  const int MaxPWM = 200;
  const double RCfilterIntensity = 0.5;
  const double pwmMultiplyIncreaseRate = 0.05;
  const double solenoidValueOpenTime = 250; //in ms
  double pwmMultiply = 0.32;
} RobotParam;

#include <PS4BT.h>
USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);

#include "MPU9250.h"
MPU9250 IMU(SerialBaud.I2C);

#include "OmniKinematics3WD.h"
OmniKinematics3WD kinematics(RobotParam.MaxPWM);

#include "UnitProtocol.hpp"
UnitProtocol SB1(&Serial1);   //SensorBoard
UnitProtocol MDD1(&Serial2);  //motorDriverDriver
UnitProtocol GenIO(&Serial3); //Solenoid Valves

#include "PIDController.h"
PIDController VRPID(5, 0, 0);

int *driverPWMOutput = new int[3];
double *rawPWM = new double[3];
bool communicationStatsLED[3];

int getbuttonClickDouble(ButtonEnum);
bool getButtonClickOnce(ButtonEnum);
void initializeOnBoardLEDs();
void updateOnBoardLEDs();
void RCfilter(const int, const double, double *, double *, double *);
void watchDogReset(uint8_t);
bool dengerKeybinds();
void commonArmLogic(int *);
void sequencedArmLogic(int *, bool);
void jammingArmLogic(int *);

void setup()
{
  pinMode(RobotArmMotorPin.armUpDownCW, OUTPUT);
  pinMode(RobotArmMotorPin.armUpDownCCW, OUTPUT);
  VRPID.setOutputLimit(50);
  setPwmFrequencyMEGA2560(RobotArmMotorPin.armUpDownCW, 1);
  setPwmFrequencyMEGA2560(RobotArmMotorPin.armUpDownCCW, 1);
  Serial.begin(SerialBaud.HardwareSerial);
  Serial1.begin(SerialBaud.HardwareSerial);
  Serial2.begin(SerialBaud.HardwareSerial);
  Serial3.begin(SerialBaud.HardwareSerial);
  while (!Serial)
    ;                   // waiting for opening hardware Serial port
  if (Usb.Init() == -1) // initialize USB device
  {
    Serial.print(F("\nArduino hasn't attached USB_HOST_SHIELD.\n"));
    while (1)
      ;
  }
  Serial.print(F("\nUSB_HOST_SHIELD detected, Success opening Serial port.\n"));
  IMU.addStatsLED(onBoardLEDs::No5);
  IMU.calibration(); // initialize 9-DOF IMU sensor and calclating bias
  initializeOnBoardLEDs();
}

void loop()
{
  for (int i = 0; i < 100; i++)
  {
    Usb.Task(); // running USB tasks
  }

  updateOnBoardLEDs();
  if (!PS4.connected()) //未接続の場合以下の処理を弾く
    return;

  if (dengerKeybinds()) //PSボタンが押されたらすべての以下の処理を弾く
    return;
  /*
        足回り処理:コントローラー&IMU読み取り、MDD出力
  */
  if (PS4.getButtonPress(SHARE) && getButtonClickOnce(R1))
    RobotParam.pwmMultiply += RobotParam.pwmMultiplyIncreaseRate;
  else if (PS4.getButtonPress(SHARE) && getButtonClickOnce(L1))
    RobotParam.pwmMultiply -= RobotParam.pwmMultiplyIncreaseRate;
  RobotParam.pwmMultiply = (RobotParam.pwmMultiply > 1.0) ? 1.0 : (RobotParam.pwmMultiply < 0) ? 0 : RobotParam.pwmMultiply;

  rawPWM[0] = (PS4.getAnalogHat(LeftHatX) - 127) * RobotParam.pwmMultiply;
  if (-3.5 < rawPWM[0] && rawPWM[0] < 3.5)
    rawPWM[0] = 0;
  rawPWM[1] = (PS4.getAnalogHat(LeftHatY) - 127) * RobotParam.pwmMultiply;
  if (-3.5 < rawPWM[1] && rawPWM[1] < 3.5)
    rawPWM[1] = 0;

  static double modifiedPWM[2], prevPWM[2];
  RCfilter(2, RobotParam.RCfilterIntensity, modifiedPWM, rawPWM, prevPWM);
  rawPWM[2] = (PS4.getAnalogButton(R2) - PS4.getAnalogButton(L2)) * 0.04;
  kinematics.getOutput(modifiedPWM[0], modifiedPWM[1], -rawPWM[2], -IMU.getYaw(), driverPWMOutput);
  int MDD1Packet[8] = {
      driverPWMOutput[0] < 0 ? 0 : driverPWMOutput[0],
      driverPWMOutput[0] > 0 ? 0 : -driverPWMOutput[0],
      driverPWMOutput[1] < 0 ? 0 : driverPWMOutput[1],
      driverPWMOutput[1] > 0 ? 0 : -driverPWMOutput[1],
      driverPWMOutput[2] < 0 ? 0 : driverPWMOutput[2],
      driverPWMOutput[2] > 0 ? 0 : -driverPWMOutput[2],
  };

  /*
      SBからセンサ値取得
      コントローラーとセンサの値から出力値を計算
      MDD1データキューに上下機構の出力値を格納
  */
  static int SensorRawData[2];
  static bool extendToggleFlag = 0;
  communicationStatsLED[1] = SB1.receive(SensorRawData); //arm, upper

  static int UDArmState = 0; //state = -1:DOWN, 0:FREE, 1:UP;
  bool stickState[2];
  stickState[0] = (20 > PS4.getAnalogHat(RightHatY)) ? 1 : 0;
  stickState[1] = (PS4.getAnalogHat(RightHatY) > 220) ? 1 : 0;
  if (stickState[0] && SensorRawData[0])
  {
    UDArmState = 1;
  }
  if (stickState[1])
  {
    UDArmState = -1;
  }
  if (SensorRawData[0] && UDArmState == -1)
  {
    UDArmState = 0;
  }

  if (UDArmState == 1)
  {
    VRPID.update(787, analogRead(RobotArmVRPin.VROut));
    int VRPIDPWM = VRPID.getTerm();
    analogWrite(RobotArmMotorPin.armUpDownCCW, VRPIDPWM > 0 ? VRPIDPWM : 0);
    analogWrite(RobotArmMotorPin.armUpDownCW, VRPIDPWM < 0 ? -VRPIDPWM : 0);
  }
  else if (UDArmState == -1)
  {
    digitalWrite(RobotArmMotorPin.armUpDownCW, LOW);
    analogWrite(RobotArmMotorPin.armUpDownCCW, 50);
  }
  else
  {
    digitalWrite(RobotArmMotorPin.armUpDownCW, LOW);
    digitalWrite(RobotArmMotorPin.armUpDownCCW, LOW);
  }

  if (getButtonClickOnce(OPTIONS))
    extendToggleFlag = !extendToggleFlag;
  if (extendToggleFlag && !SensorRawData[1])
  {
    MDD1Packet[6] = 45;
    MDD1Packet[7] = 0;
  }
  else
  {
    MDD1Packet[6] = 0;
    MDD1Packet[7] = 0;
  }
  communicationStatsLED[0] = MDD1.transmit(8, MDD1Packet);

  int GenIOPacket[4];
  commonArmLogic(GenIOPacket);
  jammingArmLogic(GenIOPacket);
  sequencedArmLogic(GenIOPacket, UDArmState);
  communicationStatsLED[2] = GenIO.transmit(4, GenIOPacket);
}

int getbuttonClickDouble(ButtonEnum b)
{
  static long samplingStartedTime[16];
  static bool isSampling[16], buttonStatus[16];
  constexpr int doubleClickSamplingTime = 180;
  buttonStatus[b] = getButtonClickOnce(b);
  if (buttonStatus[b] && !isSampling[b])
  {
    isSampling[b] = 1;
    samplingStartedTime[b] = millis();
    return 0;
  }
  if (!isSampling[b])
    return 0;
  if ((millis() - samplingStartedTime[b]) < doubleClickSamplingTime)
  {
    if (buttonStatus[b])
    {
      isSampling[b] = 0;
      return 2;
    }
    return 0;
  }
  if ((millis() - samplingStartedTime[b]) > doubleClickSamplingTime)
  {
    isSampling[b] = 0;
    return 1;
  }
  return 0;
}

bool getButtonClickOnce(ButtonEnum b)
{
  static bool previousButtonStats[16];
  bool currentStats = PS4.getButtonPress(b);
  if (!currentStats)
  {
    previousButtonStats[b] = currentStats;
    return 0;
  }
  if (currentStats != previousButtonStats[b])
  {
    previousButtonStats[b] = currentStats;
    return 1;
  }
  return 0;
}

void initializeOnBoardLEDs()
{
  pinMode(onBoardLEDs::No1, OUTPUT);
  pinMode(onBoardLEDs::No2, OUTPUT);
  pinMode(onBoardLEDs::No3, OUTPUT);
  pinMode(onBoardLEDs::No4, OUTPUT); // pinmode setup(PS4 Dual Shock Controller connection stats LED)
}

void updateOnBoardLEDs()
{
  if (PS4.connected())
  {
    digitalWrite(onBoardLEDs::No4, HIGH);
  }
  else
  {
    digitalWrite(onBoardLEDs::No4, LOW);
  }
  if (communicationStatsLED[0] && PS4.connected())
  {
    digitalWrite(onBoardLEDs::No1, HIGH);
  }
  else
  {
    digitalWrite(onBoardLEDs::No1, LOW);
  }
  if (communicationStatsLED[1] && PS4.connected())
  {
    digitalWrite(onBoardLEDs::No2, HIGH);
  }
  else
  {
    digitalWrite(onBoardLEDs::No2, LOW);
  }
  if (communicationStatsLED[2] && PS4.connected())
  {
    digitalWrite(onBoardLEDs::No3, HIGH);
  }
  else
  {
    digitalWrite(onBoardLEDs::No3, LOW);
  }
}

void RCfilter(const int arraySize, const double filterIntensity, double *arrayToContain, double *rawDataArray, double *prevFilteredData)
{
  for (int i = 0; i < arraySize; i++)
  {
    arrayToContain[i] = (filterIntensity * prevFilteredData[i]) + ((1 - filterIntensity) * rawDataArray[i]);
    prevFilteredData[i] = arrayToContain[i];
  }
}

void watchDogReset(uint8_t prescaller)
{
  wdt_enable(prescaller);
  while (1)
  {
  }
}

bool dengerKeybinds()
{
  if (!PS4.getButtonPress(PS))
  {
    return 0;
  }
  if (PS4.getButtonPress(SHARE) && PS4.getButtonPress(OPTIONS))
  {
    PS4.disconnect();
    PS4.setLed(0xF2, 0x46, 0x07);
    for (int i = 0; i < 200; i++)
    {
      Usb.Task(); // running USB tasks
      _delay_ms(1);
    }
    watchDogReset(WDTO_15MS);
    return 0;
  }
  if (PS4.getButtonPress(OPTIONS))
  {
    PS4.disconnect();
    return 0;
  }
  return 1;
}

void commonArmLogic(int *IOPacket)
{
  /*
  * 1はソレノイドバルブ右側オープン
  * 2はソレノイドバルブ左側オープン
  * 時間経過後0を代入
  * この値をGenIO基板に転送する
  */
  static unsigned long solenoidActivatedPeriod[4]; //バルブ開放開始時間保存用
  static int armState[4];                          //rightHand, leftHand, rightExtend, leftExtend
  if (getButtonClickOnce(CIRCLE))
  {
    armState[0] = 1;
    solenoidActivatedPeriod[0] = millis();
  }
  else if (getButtonClickOnce(CROSS))
  {
    armState[0] = 2;
    solenoidActivatedPeriod[0] = millis();
  }
  if (getButtonClickOnce(TRIANGLE))
  {
    armState[1] = 1;
    solenoidActivatedPeriod[1] = millis();
  }
  else if (getButtonClickOnce(SQUARE))
  {
    armState[1] = 2;
    solenoidActivatedPeriod[1] = millis();
  }
  int extendRightArmState = getbuttonClickDouble(R1);
  if (extendRightArmState == 1)
  {
    armState[2] = 1;
    solenoidActivatedPeriod[2] = millis();
  }
  else if (extendRightArmState == 2)
  {
    armState[2] = 2;
    solenoidActivatedPeriod[2] = millis();
  }
  int extendLeftArmState = getbuttonClickDouble(L1);
  if (extendLeftArmState == 1)
  {
    armState[3] = 1;
    solenoidActivatedPeriod[3] = millis();
  }
  else if (extendLeftArmState == 2)
  {
    armState[3] = 2;
    solenoidActivatedPeriod[3] = millis();
  }
  for (int i = 0; i < 4; i++)
  {
    if ((millis() - solenoidActivatedPeriod[i]) < (i < 2 ? RobotParam.solenoidValueOpenTime : RobotParam.solenoidValueOpenTime * 2 + 120))
    {
      IOPacket[i] = armState[i];
    }
    else
    {
      IOPacket[i] = 0;
    }
  }

  static bool littleExtendFlag;
  static unsigned long userButtonPressedTime;
  if (getButtonClickOnce(UP) && !littleExtendFlag)
  {
    userButtonPressedTime = millis();
    littleExtendFlag = 1;
  }
  if (littleExtendFlag)
  {
    if ((millis() - userButtonPressedTime) > 35)
    {
      IOPacket[2] = 0;
      IOPacket[3] = 0;
    }
    else
    {
      IOPacket[2] = 1;
      IOPacket[3] = 1;
    }
    if ((millis() - userButtonPressedTime) > 1000)
    {
      littleExtendFlag = 0;
    }
  }
}

void sequencedArmLogic(int *IOPacket, bool UDState)
{
  static unsigned long userButtonPressedTime[2];
  static unsigned int armLogicState[2];
  if (220 < PS4.getAnalogHat(RightHatX))
  {
    userButtonPressedTime[0] = millis();
    armLogicState[0] = 1;
  }
  if (PS4.getAnalogHat(RightHatX) < 20)
  {
    userButtonPressedTime[1] = millis();
    armLogicState[1] = 1;
  }
  for (int i = 0; i < 2; i++)
  {
    switch (armLogicState[i])
    {
      case 1:
        if (UDState) //アームシーケンスの最初の動作(ハンド伸ばす)の時だけ動作を無効化する
        {
          return;
        }
        IOPacket[i + 2] = 1; //ハンド伸ばす
        if ((millis() - userButtonPressedTime[i]) > 350)
        {
          userButtonPressedTime[i] = millis();
          armLogicState[i]++;
        }
        break;
      case 2:
        IOPacket[i] = 2; //手先閉める
        if ((millis() - userButtonPressedTime[i]) > 200)
        {
          userButtonPressedTime[i] = millis();
          armLogicState[i]++;
        }
        break;
      case 3:
        IOPacket[i + 2] = 0; //電磁弁閉める
        if ((millis() - userButtonPressedTime[i]) > 250)
        {
          userButtonPressedTime[i] = millis();
          armLogicState[i]++;
        }
        break;
      case 4:
        IOPacket[i] = 0;     //電磁弁閉める
        IOPacket[i + 2] = 2; //ハンド縮める
        if ((millis() - userButtonPressedTime[i]) > 600)
        {
          userButtonPressedTime[i] = millis();
          armLogicState[i] = 0;
        }
        break;
      default:
        break;
    }
  }
}

void jammingArmLogic(int *IOPacket)
{
  static unsigned long jammingActivatedPeriod[2];
  static unsigned int currentJammingSequenceState[2];                   //妨害シーケンスのどの段階にいるかを格納
  if (getButtonClickOnce(RIGHT) && currentJammingSequenceState[0] == 0) //右ハンド妨害
  {
    jammingActivatedPeriod[0] = millis();
    currentJammingSequenceState[0] = 1;
  }
  if (getButtonClickOnce(LEFT) && currentJammingSequenceState[1] == 0) //左ハンド妨害
  {
    jammingActivatedPeriod[1] = millis();
    currentJammingSequenceState[1] = 1;
  }
  if (getbuttonClickDouble(RIGHT)) //右ハンド妨害強制終了
  {
    jammingActivatedPeriod[0] = millis();
    currentJammingSequenceState[0] = 10;
  }
  if (getbuttonClickDouble(LEFT)) //左ハンド妨害強制終了
  {
    jammingActivatedPeriod[1] = millis();
    currentJammingSequenceState[1] = 10;
  }
  for (int i = 0; i < 2; i++)
  {
    switch (currentJammingSequenceState[i])
    {
      case 1:
        IOPacket[i] = 2;     //手先閉じる
        IOPacket[i + 2] = 1; //ハンド伸ばす
        if ((millis() - jammingActivatedPeriod[i]) > 600)
        {
          jammingActivatedPeriod[i] = millis();
          currentJammingSequenceState[i]++;
        }
        break;
      case 2:
        IOPacket[i] = 1;     //手先開く
        IOPacket[i + 2] = 0; //電磁弁閉める
        if ((millis() - jammingActivatedPeriod[i]) > 250)
        {
          jammingActivatedPeriod[i] = millis();
          currentJammingSequenceState[i]++;
        }
        break;
      case 3:
        IOPacket[i] = 0;     //電磁弁閉める
        IOPacket[i + 2] = 2; //ハンド縮める
        if ((millis() - jammingActivatedPeriod[i]) > 600)
        {
          jammingActivatedPeriod[i] = millis();
          currentJammingSequenceState[i]++;
        }
        break;
      case 4:
        IOPacket[i] = 2;     //手先閉じる
        IOPacket[i + 2] = 0; //電磁弁閉める
        if ((millis() - jammingActivatedPeriod[i]) > 250)
        {
          jammingActivatedPeriod[i] = millis();
          currentJammingSequenceState[i] = 0;
          IOPacket[i] = 0; //電磁弁閉める
        }
        break;
      case 10:               //強制終了
        IOPacket[i] = 2;     //電磁弁閉める
        IOPacket[i + 2] = 2; //電磁弁閉める
        if ((millis() - jammingActivatedPeriod[i]) > 400)
        {
          jammingActivatedPeriod[i] = millis();
          currentJammingSequenceState[i] = 0;
          IOPacket[i] = 0;     //電磁弁閉める
          IOPacket[i + 2] = 0; //電磁弁閉める
        }
        break;
      default:
        break;
    }
  }
}