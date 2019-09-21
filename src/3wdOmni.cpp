#include <Arduino.h>
#include <Wire.h>

#include <PS4BT.h>
USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);

typedef enum statsLEDs {
  No1 = 22,
  No2,
  No3,
  No4,
  No5,
} onBoardLEDs;

struct
{
  long HardwareSerial = 256000;
  int SoftwareSerial = 38400;
  long I2C = 400000;
} SerialBaud;

#define USING_9DOF_IMU
#ifdef USING_9DOF_IMU
#include "MPU9250.h"
MPU9250 IMU(SerialBaud.I2C);
#endif // USING_9DOF_IMU

struct parameter
{
  const int MaxPWM = 250;
  const int DriveWheel = 3;
  const int HeadRotationEncoderPulse = 147;
  const double RCfilterIntensity = 0;
  const double pwmMultiplyIncreaseRate = 0.05;
  const double solenoidValueOpenTime = 200; //in ms
  double pwmMultiply = 0.3;
  bool reversed = 0;
} Robot;

#include "OmniKinematics3WD.h"
OmniKinematics3WD kinematics(Robot.MaxPWM);

#include "UnitProtocol.hpp"
UnitProtocol SB1(&Serial1); //SensorBoard
UnitProtocol MDD1(&Serial2);
UnitProtocol GenIO(&Serial3);

int *driverPWMOutput = new int[Robot.DriveWheel];
double *rawPWM = new double[Robot.DriveWheel];
bool communicationStatsLED[3];

int getbuttonClickDouble(ButtonEnum);
bool getButtonClickOnce(ButtonEnum);
void initializeOnBoardLEDs();
void updateOnBoardLEDs();
inline void RCfilter(const int, const double, double *, double *, double *);
/*
    LeftStick:全方位移動    
    RightStick:ロボットヘッディング変更(4方向)
    R2:右旋回
    L2:左旋回
    circle:右側開く
    triangle:左側開く
    cross:右側閉じる
    square:左側閉じる
    up:none
    right:none
    left:none
    down:none
    share(toggle):上下展開
    R1x1:右腕伸ばす
    R1x2(100ms):右腕縮小
    L1x1:左腕伸ばす
    L1x2(100ms):左腕縮小   
    OPTION + R1:最大速度上昇
    OPTION + L1:最大速度減少
    PS:接続
    PS + OPTION:切断
 */

void setup()
{ // put your setup code here, to run once:
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
  /*
        足回り処理:コントローラー&IMU読み取り、MDD出力
  */
  if (PS4.getButtonPress(OPTIONS) && getButtonClickOnce(R1))
    Robot.pwmMultiply += Robot.pwmMultiplyIncreaseRate;
  else if (PS4.getButtonPress(OPTIONS) && getButtonClickOnce(L1))
    Robot.pwmMultiply -= Robot.pwmMultiplyIncreaseRate;
  Robot.pwmMultiply = (Robot.pwmMultiply > 1.0) ? 1.0 : (Robot.pwmMultiply < 0) ? 0 : Robot.pwmMultiply;

  rawPWM[0] = (PS4.getAnalogHat(LeftHatX) - 127) * Robot.pwmMultiply;
  if (-3.5 < rawPWM[0] && rawPWM[0] < 3.5)
    rawPWM[0] = 0;
  rawPWM[1] = (PS4.getAnalogHat(LeftHatY) - 127) * Robot.pwmMultiply;
  if (-3.5 < rawPWM[1] && rawPWM[1] < 3.5)
    rawPWM[1] = 0;

  //static double modifiedPWM[2], prevPWM[2];
  //RCfilter(2, Robot.RCfilterIntensity, modifiedPWM, rawPWM, prevPWM);
  rawPWM[2] = (PS4.getAnalogButton(R2) - PS4.getAnalogButton(L2)) * 0.04;
  kinematics.getOutput(rawPWM[0], rawPWM[1], rawPWM[2], IMU.gyro_Yaw(), driverPWMOutput);
  int MDD1Packet[Robot.DriveWheel * 2 + 2] = {
      driverPWMOutput[0] < 0 ? 0 : driverPWMOutput[0],
      driverPWMOutput[0] > 0 ? 0 : -driverPWMOutput[0],
      driverPWMOutput[1] < 0 ? 0 : driverPWMOutput[1],
      driverPWMOutput[1] > 0 ? 0 : -driverPWMOutput[1],
      driverPWMOutput[2] < 0 ? 0 : driverPWMOutput[2],
      driverPWMOutput[2] > 0 ? 0 : -driverPWMOutput[2],
  };
  /*static unsigned long timer; //ループ時間測定コード・rawPWMの表示タイミングでコントローラ側の遅延を見れる
  Serial.print(rawPWM[0]);
  Serial.print("\t");
  Serial.print(rawPWM[1]);
  Serial.print("\t");
  Serial.print(getButtonClickOnce(UP));
  Serial.print("\t");
  Serial.print(getButtonClickOnce(DOWN));
  Serial.print("\t");
  Serial.println(millis() - timer);
  timer = millis();*/

  /*
      SBからセンサ値取得
      コントローラーとセンサの値から出力値を計算
      MDD1データキューに上下機構の出力値を格納
  */
  static int SensorRawData[2];
  static bool extendToggleFlag = 0;
  communicationStatsLED[1] = SB1.receive(SensorRawData); //LimitSW, LimitSW
  if (getButtonClickOnce(SHARE))
    extendToggleFlag = !extendToggleFlag;
  if (extendToggleFlag && !SensorRawData[0])
  { //展開したい&&上側のリミットスイッチが押されてない
    MDD1Packet[6] = 50;
    MDD1Packet[7] = 0;
  }
  else if (!extendToggleFlag && !SensorRawData[1])
  { //縮小したい&&下側のリミットスイッチが押されてない
    MDD1Packet[6] = 0;
    MDD1Packet[7] = 50;
  }
  else
  {
    MDD1Packet[6] = 0;
    MDD1Packet[7] = 0;
  }
  communicationStatsLED[0] = MDD1.transmit(Robot.DriveWheel * 2 + 2, MDD1Packet);

  /*
  * 1はソレノイドバルブ右側オープン
  * 2はソレノイドバルブ左側オープン
  * 時間経過後0を代入
  * この値をGenIO基板に転送する
  */
  static unsigned long solenoidActivatedPeriod[4]; //バルブ開放開始時間保存用
  static int armState[4];                          //rightHand, leftHand, rightExtend, leftExtend
  int GenIOPacket[4];
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
  static int jammingSequenceFlag[2];
  static int jammingSequenceState[2];
  static unsigned long jammingSequenceTime[2];
  if (getButtonClickOnce(UP) && !jammingSequenceFlag[0])
  {
    jammingSequenceFlag[0] = 1;
    jammingSequenceState[0] = 1;
    jammingSequenceTime[0] = millis();
  }
  else if (getButtonClickOnce(DOWN) && !jammingSequenceFlag[1])
  {
    jammingSequenceFlag[1] = 1;
    jammingSequenceState[1] = 1;
    jammingSequenceTime[1] = millis();
  }

  for (int i = 0; i < 4; i++)
  {
    if ((millis() - solenoidActivatedPeriod[i]) < (i < 2 ? Robot.solenoidValueOpenTime : Robot.solenoidValueOpenTime * 2))
    {
      GenIOPacket[i] = armState[i];
    }
    else
    {
      GenIOPacket[i] = 0;
    }
  }
  static bool initialFlag;
  static unsigned long initialTime;
  if (20 > PS4.getAnalogHat(RightHatY) && !initialFlag)
  {
    initialTime = millis();
    initialFlag = 1;
  }

  if (initialFlag)
  {
    if ((millis() - initialTime) > 35)
    {
      GenIOPacket[2] = 0;
      GenIOPacket[3] = 0;
    }
    else
    {
      GenIOPacket[2] = 1;
      GenIOPacket[3] = 1;
    }
    if ((millis() - initialTime) > 1000)
    {
      initialFlag = 0;
    }
  }
  communicationStatsLED[2] = GenIO.transmit(4, GenIOPacket);

  /*
  コントローラー切断処理
  */
  if (PS4.getButtonPress(PS))
  {
    if (getButtonClickOnce(OPTIONS))
    {
      PS4.disconnect();
    }
  }
}

int getbuttonClickDouble(ButtonEnum b)
{
  static long samplingStartedTime[16];
  static bool isSampling[16], buttonStatus[16];
  constexpr int doubleClickSamplingTime = 200;
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

inline void RCfilter(const int arraySize, const double filterIntensity, double *arrayToContain, double *rawDataArray, double *prevFilteredData)
{
  for (int i = 0; i < arraySize; i++)
  {
    arrayToContain[i] = (filterIntensity * prevFilteredData[i]) + ((1 - filterIntensity) * rawDataArray[i]);
    prevFilteredData[i] = arrayToContain[i];
  }
}
