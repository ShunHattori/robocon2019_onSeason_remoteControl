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
UnitProtocol MDD2(&Serial3);

int *driverPWMOutput = new int[Robot.DriveWheel];
double *rawPWM = new double[Robot.DriveWheel];

void initializeOnBoardLEDs();
void updateOnBoardLEDs();
inline void RCfilter(const int, const double, double *, double *, double *);
/*
    circle:右側開く
    triangle:左側開く
    cross:右側閉じる
    square:左側閉じる
    up:右側伸ばす
    right:右側縮小
    left:左側伸ばす
    down:左側縮小
    share(toggle):上下展開
    R1:頭右回転
    L1:頭左回転
    OPTION + R1:最大速度上昇
    OPTION + L1:最大速度減少
    R2:右旋回
    L2:左旋回
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
  IMU.addStatsLED(onBoardLEDs::No1);
  IMU.calibration(); // initialize 9-DOF IMU sensor and calclating bias
  initializeOnBoardLEDs();
}

void loop()
{
  Usb.Task(); // running USB tasks
  updateOnBoardLEDs();
  if (!PS4.connected()) //未接続の場合以下の処理を弾く
    return;
  /*
        足回り処理:コントローラー&IMU読み取り、MDD出力
  */
  if (PS4.getButtonPress(OPTIONS) && PS4.getButtonClick(R1))
    Robot.pwmMultiply += Robot.pwmMultiplyIncreaseRate;
  else if (PS4.getButtonPress(OPTIONS) && PS4.getButtonClick(L1))
    Robot.pwmMultiply -= Robot.pwmMultiplyIncreaseRate;
  Robot.pwmMultiply = (Robot.pwmMultiply > 1.0) ? 1.0 : (Robot.pwmMultiply < 0) ? 0 : Robot.pwmMultiply;

  rawPWM[0] = (PS4.getAnalogHat(LeftHatX) - 127) * Robot.pwmMultiply;
  if (-3.5 < rawPWM[0] && rawPWM[0] < 3.5)
    rawPWM[0] = 0;
  rawPWM[1] = (PS4.getAnalogHat(LeftHatY) - 127) * Robot.pwmMultiply;
  if (-3.5 < rawPWM[1] && rawPWM[1] < 3.5)
    rawPWM[1] = 0;

  static double modifiedPWM[2], prevPWM[2];
  RCfilter(2, Robot.RCfilterIntensity, modifiedPWM, rawPWM, prevPWM);
  rawPWM[2] = (PS4.getAnalogButton(R2) - PS4.getAnalogButton(L2)) * 0.04;
  kinematics.getOutput(modifiedPWM[0], modifiedPWM[1], -rawPWM[2], -IMU.gyro_Yaw(), driverPWMOutput);

  int drivePacket[Robot.DriveWheel * 2] = {
      driverPWMOutput[0] < 0 ? 0 : driverPWMOutput[0],
      driverPWMOutput[0] > 0 ? 0 : -driverPWMOutput[0],
      driverPWMOutput[1] < 0 ? 0 : driverPWMOutput[1],
      driverPWMOutput[1] > 0 ? 0 : -driverPWMOutput[1],
      driverPWMOutput[2] < 0 ? 0 : driverPWMOutput[2],
      driverPWMOutput[2] > 0 ? 0 : -driverPWMOutput[2],
  };
  MDD2.transmit(Robot.DriveWheel * 2, drivePacket);

  /*
        SBからセンサ値取得
  */
  static int SensorRawData[5];
  static int SensorModifiedData[3];
  SB1.receive(SensorRawData); //LimitSW, LimitSW, Encoder_HIGH, Encoder_LOW
  for (int i = 0; i < 3; i++)
  {
    if (i < 2)
    {
      SensorModifiedData[i] = SensorRawData[i];
      continue;
    }
    SensorModifiedData[i] = SensorRawData[i + 1] + (255 * abs((SensorRawData[i + 2] - 128)));
    if (SensorRawData[i])
      SensorModifiedData[i] = -SensorModifiedData[i];
  }
  /*
    Serial.print(SensorModifiedData[0]);
    Serial.print("\t");
    Serial.print(SensorModifiedData[1]);
    Serial.print("\t");
    Serial.println(SensorModifiedData[2]);
    */
  /*
        コントローラーとセンサの値から出力値を計算
  */
  static bool extendToggleFlag = 0;
  static int rotationMecaTartget = 0;
  static int armState[4]; //rightHand, leftHand, rightExtend, leftExtend
  if (PS4.getButtonClick(SHARE))
    extendToggleFlag = !extendToggleFlag;
  if (PS4.getButtonClick(R1))
    rotationMecaTartget += Robot.HeadRotationEncoderPulse;
  else if (PS4.getButtonClick(L1))
    rotationMecaTartget -= Robot.HeadRotationEncoderPulse;
  /*
  * 1はソレノイドバルブ右側オープン
  * 2はソレノイドバルブ左側オープン
  * 時間経過後0を代入
  * この値をMDDに転送する
  */
  static unsigned long solenoidActivatedPeriod[4]; //バルブ開放開始時間保存用
  int MDD1Packet[10];
  if (PS4.getButtonClick(CIRCLE))
  {
    armState[0] = 1;
    solenoidActivatedPeriod[0] = millis();
  }
  else if (PS4.getButtonClick(CROSS))
  {
    armState[0] = 2;
    solenoidActivatedPeriod[0] = millis();
  }
  if (PS4.getButtonClick(TRIANGLE))
  {
    armState[1] = 1;
    solenoidActivatedPeriod[1] = millis();
  }
  else if (PS4.getButtonClick(SQUARE))
  {
    armState[1] = 2;
    solenoidActivatedPeriod[1] = millis();
  }
  if (PS4.getButtonClick(UP))
  {
    armState[2] = 1;
    solenoidActivatedPeriod[2] = millis();
  }
  else if (PS4.getButtonClick(RIGHT))
  {
    armState[2] = 2;
    solenoidActivatedPeriod[2] = millis();
  }
  if (PS4.getButtonClick(LEFT))
  {
    armState[3] = 1;
    solenoidActivatedPeriod[3] = millis();
  }
  else if (PS4.getButtonClick(DOWN))
  {
    armState[3] = 2;
    solenoidActivatedPeriod[3] = millis();
  }
  for (int i = 0; i < 4; i++)
  {
    if ((millis() - solenoidActivatedPeriod[i]) < Robot.solenoidValueOpenTime)
    {
      MDD1Packet[i + 4] = armState[i];
    }
    else
    {
      MDD1Packet[i + 4] = 0;
    }
  }

  /*
        MDD1データキューに回転、上下機構の出力値を格納
        TODO:ソレノイド処理を追加
    */
  /*if (extendToggleFlag && !SensorModifiedData[0])
  { //展開したい&&上側のリミットスイッチが押されてない
    MDD1Packet[0] = 50;
    MDD1Packet[1] = 0;
  }
  else if (!extendToggleFlag && !SensorModifiedData[1])
  { //縮小したい&&下側のリミットスイッチが押されてない
    MDD1Packet[0] = 0;
    MDD1Packet[1] = 50;
  }
  else
  {
    MDD1Packet[0] = 0;
    MDD1Packet[1] = 0;
  }*/
  if (7 > (rotationMecaTartget - SensorModifiedData[2]) && (rotationMecaTartget - SensorModifiedData[2]) > -7)
  {
    MDD1Packet[0] = 0;
    MDD1Packet[1] = 0;
  }
  else if ((rotationMecaTartget - SensorModifiedData[2]) > 0)
  {
    MDD1Packet[0] = 50;
    MDD1Packet[1] = 0;
  }
  else if (0 > (rotationMecaTartget - SensorModifiedData[2]))
  {
    MDD1Packet[0] = 0;
    MDD1Packet[1] = 50;
  }
  MDD1.transmit(8, MDD1Packet);
  Serial.print(rotationMecaTartget);
  Serial.print(",");
  Serial.print(SensorModifiedData[2]);
  Serial.print(",");
  Serial.print(MDD1Packet[2]);
  Serial.print(",");
  Serial.println(MDD1Packet[3]);
  /*
        コントローラー切断処理
  */
  if (PS4.getButtonPress(PS))
  {
    if (PS4.getButtonClick(OPTIONS))
    {
      PS4.disconnect();
    }
  }
}

void initializeOnBoardLEDs()
{
  pinMode(onBoardLEDs::No2, OUTPUT); // pinmode setup(PS4 Dual Shock Controller connection stats LED)
}

void updateOnBoardLEDs()
{
  if (PS4.connected())
  {
    digitalWrite(onBoardLEDs::No2, HIGH);
  }
  else
  {
    digitalWrite(onBoardLEDs::No2, LOW);
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
