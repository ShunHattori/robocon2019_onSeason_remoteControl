#include "UnitProtocol.hpp"

UnitProtocol::UnitProtocol(Stream *str)
{
  _myStream = str;
  _watchDogInitTime = 0;
  _timeoutMs = 20; // set the timeout time to  20ms
  _isTransmittable = 1;
  _isReceivable = 1;
}

void UnitProtocol::transmit(int arrayLenght, int *packet)
{
  _arrayLenght = arrayLenght;
  _myStream->write(ENQ); //送信してもいいですか
  _watchDogComparePrevTime = millis();
  while (1) //返事待ち
  {
    if ((millis() - _watchDogComparePrevTime) > _timeoutMs) //_timeoutMs間返答なし
    {
      return; //タイムアウト
    }
    if (!(_myStream->available())) //バッファに何もない
    {
      continue; //タイムアウト判定に戻る
    }
    if (_myStream->read() == ACK) //バッファの中身がACKだったら
    {
      break; //次へゴー！
    }
  }
  _myStream->write(_arrayLenght); //データの長さはこれだけです。
  char _checkSum = 0;
  for (int i = 0; i < arrayLenght; i++)
  {
    _myStream->write(packet[i]); //データの中身
    _checkSum ^= packet[i];      //チェックサム計算
  }
  _myStream->write(_checkSum); //これと比較して破損確認
}

void UnitProtocol::receive(int *variableToStore)
{
  if (!(_myStream->available())) //バッファに何もない
  {
    return;
  }
  if (_myStream->read() != ENQ) //中身がENQじゃなかったら
  {
    return;
  }
  _myStream->write(ACK); //中身がENQじゃなかったらACK送信
  int _incomingCounter = 0;
  char _buffer[250], _bufferSum = 0;
  _watchDogComparePrevTime = millis();
  while (1) //返事待ち
  {
    if ((millis() - _watchDogComparePrevTime) > _timeoutMs) //_timeoutMs間返答なし
    {
      return; //タイムアウト
    }
    if (!(_myStream->available())) //バッファに何もない
    {
      continue; //タイムアウト判定に戻る
    }
    break; //次へゴー！
  }
  _arrayLenght = _myStream->read();
  for (int i = 0; i < _arrayLenght; i++)
  {
    _watchDogComparePrevTime = millis();
    while (1)
    {
      if ((millis() - _watchDogComparePrevTime) > _timeoutMs) //_timeoutMs間返答なし
      {
        return; //タイムアウト
      }
      if (!(_myStream->available())) //バッファに何もない
      {
        continue; //タイムアウト判定に戻る
      }
      _buffer[_incomingCounter++] = _myStream->read();
      break; //次へゴー！
    }
  }
  if (_arrayLenght != _incomingCounter) //あらかじめ教えられていたデータ長と実際に来たデータ長が違う場合を弾く
  {
    return;
  }
  for (int i = 0; i < _incomingCounter; i++)
  {
    _bufferSum ^= _buffer[i];
  }
  while (1)
  {
    if ((millis() - _watchDogComparePrevTime) > _timeoutMs) //_timeoutMs間返答なし
    {
      return; //タイムアウト
    }
    if (!(_myStream->available())) //バッファに何もない
    {
      continue; //タイムアウト判定に戻る
    }
    if (_myStream->read() != _bufferSum)
    {
      return;
    }
    break;
  }
  for (int i = 0; i < _incomingCounter; i++)
  {
    variableToStore[i] = _buffer[i];
  }
}

void UnitProtocol::setTimeout(int timeoutTimeInMs)
{
  _timeoutMs = timeoutTimeInMs;
}